#include "fatfs.h"   // FatFs header files for file system operations
#include "stdio.h"
#include "string.h"

#define FILE_NAME "SPAINDRIFT.TXT"

// Assume these constants for motor direction
#define DIR_FORWARD  1
#define DIR_REVERSE  0

void spain_drift(int n_cycles, float stroke_mm)
{
    FIL file;
    FRESULT res;
    FATFS fs;
    TCHAR buffer[64];
    UINT bw;
   
    float voltage_forward, voltage_reverse;
    float max_pos = -10.0f, min_pos = 10.0f;      // Track max/min positive voltages
    float max_neg = -10.0f, min_neg = 10.0f;      // Track max/min negative voltages
   
    int steps = distance_to_steps(stroke_mm);
   
    // Mount SD card filesystem
    res = f_mount(&fs, "", 1);
    if (res != FR_OK)
    {
        I2C_LCD_Clear();
        I2C_LCD_SetCursor(0,0);
        I2C_LCD_WriteString("SD Mount Fail");
        return;
    }
   
    // Open file for writing, create if doesn't exist, overwrite existing
    res = f_open(&file, FILE_NAME, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        I2C_LCD_Clear();
        I2C_LCD_SetCursor(0,0);
        I2C_LCD_WriteString("File Open Fail");
        return;
    }
   
    // Move file pointer to start just for clarity, file just created
    f_lseek(&file, 0);
   
    for (int cycle = 1; cycle <= n_cycles; cycle++)
    {
        // Move forward
        set_motor_direction(DIR_FORWARD);
        move_motor_forward(steps);
       
        voltage_forward = read_voltage_ad7606();
       
        // Update max/min positive voltage (assuming forward voltage is positive)
        if (voltage_forward > max_pos) max_pos = voltage_forward;
        if (voltage_forward < min_pos) min_pos = voltage_forward;
       
        // Log forward voltage
        int len = snprintf((char*)buffer, sizeof(buffer), "Cycle %d +: %+1.4f V\r\n", cycle, voltage_forward);
        res = f_write(&file, buffer, len, &bw);
        if (res != FR_OK || bw < len)
        {
            I2C_LCD_Clear();
            I2C_LCD_SetCursor(0,0);
            I2C_LCD_WriteString("Write Error Fwd");
            break;
        }
       
        // Move reverse
        set_motor_direction(DIR_REVERSE);
        move_motor_forward(steps);
       
        voltage_reverse = read_voltage_ad7606();
       
        // Update max/min negative voltage (assuming reverse voltage is negative)
        if (voltage_reverse > max_neg) max_neg = voltage_reverse;
        if (voltage_reverse < min_neg) min_neg = voltage_reverse;
       
        // Log reverse voltage
        len = snprintf((char*)buffer, sizeof(buffer), "Cycle %d -: %+1.4f V\r\n", cycle, voltage_reverse);
        res = f_write(&file, buffer, len, &bw);
        if (res != FR_OK || bw < len)
        {
            I2C_LCD_Clear();
            I2C_LCD_SetCursor(0,0);
            I2C_LCD_WriteString("Write Error Rev");
            break;
        }
       
        // Flush SD card buffer to ensure data safety every 100 cycles (tune this number as needed)
        if (cycle % 100 == 0)
        {
            res = f_sync(&file);
            if (res != FR_OK)
            {
                I2C_LCD_Clear();
                I2C_LCD_SetCursor(0, 0);
                I2C_LCD_WriteString("Sync Fail");
                break;
            }
        }
    }
   
    // Final sync and close file
    f_sync(&file);
    f_close(&file);
    f_mount(NULL, "", 1);  // Unmount
   
    // Display max/min voltages on LCD
    I2C_LCD_Clear();
   
    I2C_LCD_SetCursor(0,0);
    char line1[20];
    snprintf(line1, sizeof(line1), "+ Max:%1.3f Min:%1.3f", max_pos, min_pos);
    I2C_LCD_WriteString(line1);
   
    I2C_LCD_SetCursor(0,1);
    char line2[20];
    snprintf(line2, sizeof(line2), "- Max:%1.3f Min:%1.3f", max_neg, min_neg);
    I2C_LCD_WriteString(line2);
}
