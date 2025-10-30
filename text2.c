/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM_Model_3
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "stdint.h"
#include "string.h"
#include "../../ECUAL/I2C_LCD/I2C_LCD.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MyI2C_LCD I2C_LCD_1

//Limit Switch Pins
#define LIMIT_PIN GPIO_PIN_4
#define LIMIT_PORT GPIOA

#define DIR_PIN GPIO_PIN_6
#define DIR_PORT GPIOA

#define STEP_PIN GPIO_PIN_8
#define STEP_PORT GPIOA

#define ENABLE_PIN GPIO_PIN_7
#define ENABLE_PORT GPIOA

//keypad pins
#define ROW1_PIN GPIO_PIN_0
#define ROW1_PORT GPIOB
#define ROW2_PIN GPIO_PIN_1
#define ROW2_PORT GPIOB
#define ROW3_PIN GPIO_PIN_2
#define ROW3_PORT GPIOB
#define ROW4_PIN GPIO_PIN_10
#define ROW4_PORT GPIOB

#define COL1_PIN GPIO_PIN_3
#define COL1_PORT GPIOB
#define COL2_PIN GPIO_PIN_4
#define COL2_PORT GPIOB
#define COL3_PIN GPIO_PIN_5
#define COL3_PORT GPIOB
#define COL4_PIN GPIO_PIN_6
#define COL4_PORT GPIOB

#define ROWS 4
#define COLS 4

//stepper parameters
#define STEPS_PER_REV 200
#define BALLSCREW_PITCH_MM 0.08f
#define  MICROSTEPS 16

//ADC Channels
// ADC Pins (AD7606)
#define AD7606_NUM_CHANNELS 8
#define AD7606_DATA_SIZE (AD7606_NUM_CHANNELS * 8)

#define AD7606_RESET_PIN GPIO_PIN_1
#define AD7606_RESET_PORT GPIOA

#define CONVST_PIN GPIO_PIN_0
#define AD7606_CONVST_PORT GPIOA

#define BUSY_PIN GPIO_PIN_12
#define AD7606_BUSY_PORT GPIOA

#define CHIP_SELECT_PIN GPIO_PIN_12
#define AD7606_CHIP_SELECT_PORT GPIOB

#define UART_BUFFER_SIZE 64
#define INPUT_SIZE 16


#define UART_BUFFER_SIZE 64
#define INPUT_SIZE 16

#define BALLSCREW_FULL_STROKE_MM 300.0f
#define MAX_VOLTAGE 10.0f // ±10 Volts full-scale output
#define KEY_PLUS_MINUS 0xB1  // Example code for '±' key, define according to your keypad


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


FATFS FatFs;
FIL Fil;
bool sd_mounted = false;


volatile int step_count =0;
volatile int steps_target = 0;
volatile uint8_t limit_triggered=0;
volatile uint8_t null_reached = 0;
float total_strike_length_mm =0;
float total_distance_travelled = 0.0f;// track the ball screw track

uint8_t input_index = 0;

char value1_str[INPUT_SIZE];
char value2_str[INPUT_SIZE];
char input_buffer[UART_BUFFER_SIZE];
// Key mapping for 4x4 keypad
GPIO_TypeDef* rowPorts[ROWS] = {ROW1_PORT, ROW2_PORT, ROW3_PORT, ROW4_PORT};
uint16_t rowPins[ROWS] = {ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN};
GPIO_TypeDef* colPorts[COLS] = {COL1_PORT, COL2_PORT, COL3_PORT, COL4_PORT};
uint16_t colPins[COLS] = {COL1_PIN, COL2_PIN, COL3_PIN, COL4_PIN};
char keys[ROWS][COLS] = {
  {'1', '2', '3', '+'},
  {'4', '5', '6', '-'},
  {'7', '8', '9', '.'},
  {'*', '0', '#', '.'}
};
int half;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//To check the sdcard mount status
bool SD_Mount(void) {
    if (!sd_mounted) {
        if (f_mount(&FatFs, "", 1) == FR_OK) {
            sd_mounted = true;
        } else {
            sd_mounted = false;
        }
    }
    return sd_mounted;
}


















// Perform homing routine - move motor until limit switch hit
void Homing() {
    // Set the direction pin as needed for homing (typically "back" towards the limit switch)
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    // Move until the limit switch is triggered (assume limit switch is active HIGH when hit)
    while (HAL_GPIO_ReadPin(LIMIT_PORT, LIMIT_PIN) == GPIO_PIN_RESET) {
            // Generate a step pulse
            HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);//Reverse direction to move forward one step
            HAL_Delay(2);  // Pulse width, adjust if needed
            HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
            HAL_Delay(2);
        }

    //once limit switch triggered move forward  one step to back off the switch
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN,GPIO_PIN_SET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN,GPIO_PIN_RESET);
    HAL_Delay(2);
    //Stop movement by not continuing  to step
}

void rotate_stepper(int direction, int steps){

	//steps_target = steps * 2;//two pulses per full step

	steps_target = steps;
	limit_triggered = 0; //reset limit flag
	step_count=0; //reset step counter
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET); // Set direction
	 HAL_TIM_Base_Start_IT(&htim2);       //start timer interrupts that generate steps.

	 // Wait while timer is running and limit switch is NOT triggered
	 while(HAL_TIM_Base_GetState(&htim2) != HAL_TIM_STATE_READY && !limit_triggered) {

	         if(HAL_GPIO_ReadPin(LIMIT_PORT, LIMIT_PIN) == GPIO_PIN_SET) {
	             limit_triggered = 1;         // Limit switch hit, set flag
	             HAL_TIM_Base_Stop_IT(&htim2); // stop stepping immediately
	             break;
	         }
	   }
}

void calibrate_ballscrew() {
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "CALIBRATING...");

    // Rotate forward one revolution
    rotate_stepper(1, STEPS_PER_REV);

    // If limit switch not triggered, rotate backward one revolution
    if(!limit_triggered) {
        HAL_Delay(1000);
        rotate_stepper(0, STEPS_PER_REV);
    }

    HAL_Delay(1000);
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "CALIBRATION... COMPLETED");
}

void keypad_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Initialize row pins as output push-pull, default HIGH
	    for (int i = 0; i < ROWS; i++){

	    	GPIO_InitStruct.Pin = rowPins[i];
	    	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
	    	GPIO_InitStruct.Pull =GPIO_NOPULL;
	    	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    	HAL_GPIO_Init(rowPorts[i], &GPIO_InitStruct);
	    	HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);  // set rows HIGH

	    }

	    // Initialize column pins as input with pull-up
	        for (int i = 0; i < COLS; i++)
	        {
	            GPIO_InitStruct.Pin = colPins[i];
	            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	            GPIO_InitStruct.Pull = GPIO_PULLUP;
	            HAL_GPIO_Init(colPorts[i], &GPIO_InitStruct);
	        }
}
char Keypad_GetKey(void){
	for (int row = 0; row < ROWS; row++)
	    {
	        // Drive one row LOW at a time
	        for (int i = 0; i < ROWS; i++)
	        {
	            HAL_GPIO_WritePin(rowPorts[i], rowPins[i], (i == row) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	        }
	        HAL_Delay(1); // Small delay to settle

	        // Read columns
	        for (int col = 0; col < COLS; col++)
	        {
	            if (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET)
	            {
	                // Wait for key release (debounce)
	                while (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET);
	                return keys[row][col];
	            }
	        }
	    }
	return 0;
}

int read_total_stroke_length() {
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Enter stroke length:");
    char key;
    int length = 0;
    int digitCount = 0;
    // Cursor to second line
    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
    while(1){
        key = Keypad_GetKey();
        if(key){
         if (key >= '0' && key <= '9'){
                if (digitCount < 5) {
                    length = length * 10 + (key - '0');
                    digitCount++;
                    char displayChar[2] = {key, '\0'};
                    I2C_LCD_WriteString(MyI2C_LCD, displayChar);
                }
            } else if (key == '#'){
                if (digitCount > 0)
                    break;  // Finish input only if digits entered
            } else if (key == '*') {
                length = 0;
                digitCount = 0;
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
                I2C_LCD_WriteString(MyI2C_LCD, "    ");  // Clear line
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
            }
        }
        HAL_Delay(50); // debounce delay
    }
    // Display completed length before returning
    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
    char strLength[5];
    sprintf(strLength, "%d   ", length);
    I2C_LCD_WriteString(MyI2C_LCD, strLength);
    return length;
}
/*
int read_total_stroke_length() {
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Enter stroke length:");

    char key;
    int length = 0;
    int digitCount = 0;
    int isNegative = 0;
    int signEntered = 0;

    // Cursor at second line
    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);

    while (1) {
        key = Keypad_GetKey();
        if (key) {
            // Check sign keys - ensure sign entered only once
            if (!signEntered && (key == KEY_PLUS_MINUS || key == '+' || key == '-')) {
                signEntered = 1;
                isNegative = (key == '-' || key == KEY_PLUS_MINUS) ? 1 : 0;

                // Display sign on LCD (use string for '±' to avoid warnings)
                if (key == KEY_PLUS_MINUS) {
                    I2C_LCD_WriteString(MyI2C_LCD, "±");
                } else {
                    char signStr[2] = {key, '\0'};
                    I2C_LCD_WriteString(MyI2C_LCD, signStr);
                }
            }
            // Accept digits only after sign entered
            else if (signEntered && key >= '0' && key <= '9') {
                if (digitCount < 3) { // max 3 digits
                    length = length * 10 + (key - '0');
                    digitCount++;
                    char digitChar[2] = {key, '\0'};
                    I2C_LCD_WriteString(MyI2C_LCD, digitChar);
                }
            }
            // Confirm input
            else if (key == '#') {
                if (digitCount > 0) break;
            }
            // Clear input
            else if (key == '*') {
                length = 0;
                digitCount = 0;
                isNegative = 0;
                signEntered = 0;
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
                I2C_LCD_WriteString(MyI2C_LCD, "    ");
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
            }
        }
        HAL_Delay(50); // debounce delay
    }

    int finalLength = isNegative ? -length : length;

    // Display final length with sign
    char outputStr[8];
    sprintf(outputStr, "%c%d   ", (isNegative ? '-' : '+'), length);
    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
    I2C_LCD_WriteString(MyI2C_LCD, outputStr);

    return finalLength;
}
*/

float voltage_to_distance(float voltage_out)
{
    // Clamp voltage to valid range to avoid incorrect displacement
    if (voltage_out > MAX_VOLTAGE)
        voltage_out = MAX_VOLTAGE;
    else if (voltage_out < -MAX_VOLTAGE)
        voltage_out = -MAX_VOLTAGE;

    // Convert voltage to displacement:
    // voltage_out / max_voltage gives normalized displacement (-1 to +1)
    // multiplied by half stroke to get mm displacement from center
    float displacement_mm = (voltage_out / MAX_VOLTAGE) * (BALLSCREW_FULL_STROKE_MM / 2.0f);
    return displacement_mm;
}
// Convert given linear distance to motor steps
int distance_to_steps(float distance_mm) {
    //float revs = distance_mm / BALLSCREW_PITCH_MM;
    float revs = (STEPS_PER_REV * MICROSTEPS)/ BALLSCREW_PITCH_MM ;
   // return (int)(revs * STEPS_PER_REV);
    return (int)(distance_mm *  revs);
    // it returns the value   to the  distance to steps.
}

// Step generation - pulse step pin steps times forward
void move_motor_forward(int steps) {
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);  //forward

    for (int i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
        HAL_Delay(1); // adjust pulse width
        HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
        HAL_Delay(1);
    }
}

float read_distance_from_keypad(){
	char key;
	char input[23] = {0};
	int idx = 0;
	int decimal_entered =0;
		I2C_LCD_Clear(MyI2C_LCD);
	    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	    I2C_LCD_WriteString(MyI2C_LCD, "Enter dist (mm):");
	    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);

	    while(1){
	    	key = Keypad_GetKey();
	    	if (key) {
	    	            if ((key >= '0' && key <= '9') && idx < 5) {
	    	                input[idx++] = key;
	    	                input[idx] = '\0';
	    	                I2C_LCD_WriteString(MyI2C_LCD, (char[]){key,'\0'});
	    	            }else if(key == '.' && !decimal_entered && idx < 5) {
	    	            	input[idx++]=key;
	    	            	input[idx]='\0';
	    	            	decimal_entered =1;
	    	            	 I2C_LCD_WriteString(MyI2C_LCD, (char[]){key,'\0'});

	    	            }else if(key == '#') {

	    	                if (idx > 0) break;  // finish input

	    	            } else if (key == '*') {
	    	                idx = 0;
	    	                decimal_entered =0;
	    	                memset(input, 0, sizeof(input));
	    	                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
	    	                I2C_LCD_WriteString(MyI2C_LCD, "   ");
	    	                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
	    	            }
	    	        }
	    	        HAL_Delay(50);
	    	    }
	    	return strtof(input, NULL);
}

void set_motor_direction(int direction) {
    if (direction == 0) {  // backward
    	 HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    } else {               // forward
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    }
}

// read voltage from AD7606 channel 0
float read_voltage_ad7606() {
    uint16_t adc_raw = 0;
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, CONVST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD7606_CONVST_PORT, CONVST_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    while (HAL_GPIO_ReadPin(AD7606_BUSY_PORT, BUSY_PIN) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD7606_CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_RESET);

    uint8_t rx_buf[2] = {0};
    HAL_SPI_Receive(&hspi2, rx_buf, 2, 10);

    HAL_GPIO_WritePin(AD7606_CHIP_SELECT_PORT, CHIP_SELECT_PIN, GPIO_PIN_SET);

    adc_raw = ((rx_buf[0] << 8) | rx_buf[1]);  // Correct usage

    float voltage = ((float)((int16_t)adc_raw) / 32768.0f) * 10.0f;
    return voltage;
}


float read_excitation_voltage_from_keypad() {
    // Similar to your float reading function, prompt user for excitation voltage
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Excitation V (2-7):");
    float exc_voltage = read_distance_from_keypad();

    // Validate voltage range
    if (exc_voltage < 2.0f || exc_voltage > 7.0f) {
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_WriteString(MyI2C_LCD, "Invalid Exc V");
        HAL_Delay(1500);
        return 0.0f; // Or retry as needed
    }
    return exc_voltage;
}

float read_required_output_voltage() {
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Req Output V:");
    return read_distance_from_keypad();  // Reuse float input function
}

// Handles signed displacement input when NULL position is reached by user via keypad input.
// Allows input of positive or negative millimeter values within allowed range 'half'.
void handle_null_displacement(void) {
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Enter dist (+/-):");

    char displacement_str[6] = {0};  // 1 sign + 3 digits + NULL + safety
    int disp_idx = 0;
    int negative_allowed = 1;
    int positive_allowed = 1;

    while (1) {
        char key = Keypad_GetKey();
        if (key) {
            // Handle negative sign
            if (key == '-' && negative_allowed && disp_idx == 0) {
                displacement_str[disp_idx++] = '-';
                displacement_str[disp_idx] = '\0';
                negative_allowed = 0;
                positive_allowed = 0;
            }
            // Handle positive sign
            else if (key == '+' && positive_allowed && disp_idx == 0) {
                displacement_str[disp_idx++] = '+';
                displacement_str[disp_idx] = '\0';
                positive_allowed = 0;
                negative_allowed = 0;
            }
            // Handle digits
            else if (key >= '0' && key <= '9') {
                if (disp_idx < 5) { // Max 3 digits + optional sign
                    displacement_str[disp_idx++] = key;
                    displacement_str[disp_idx] = '\0';
                }
            }
            // Handle backspace '*'
            else if (key == '*' && disp_idx > 0) {
                disp_idx--;
                displacement_str[disp_idx] = '\0';
                // Re-allow sign if deleted
                if (disp_idx == 0) {
                    negative_allowed = 1;
                    positive_allowed = 1;
                }
            }
            // Handle enter '#'
            else if (key == '#') {
                if ((disp_idx > 0 && displacement_str[0] != '-') &&
                    (disp_idx > 0 && displacement_str[0] != '+')) {
                    break;
                } else if ((disp_idx > 1 && (displacement_str[0] == '-' || displacement_str[0] == '+'))) {
                    break;
                }
            }
            // Update LCD display
            I2C_LCD_Clear(MyI2C_LCD);
            I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
            I2C_LCD_WriteString(MyI2C_LCD, displacement_str);
        }
        HAL_Delay(100); // debounce
    }

    // Convert input string to integer
    int displacement_mm = atoi(displacement_str);

    // Check valid range
    if (displacement_mm < -half || displacement_mm > half) {
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_WriteString(MyI2C_LCD, "Invalid Input");
        HAL_Delay(1500);
        return; // Can re-prompt if needed
    }
    // Convert mm to steps and set direction
    int steps = distance_to_steps(abs(displacement_mm));
    int direction = (displacement_mm >= 0) ? 1 : 0;

    // Display movement info
    char move_msg[32];
    sprintf(move_msg, "Moving %d mm %s", abs(displacement_mm), direction ? "forward" : "backward");
    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_WriteString(MyI2C_LCD, move_msg);

    // Start motor
    set_motor_direction(direction);
    move_motor_forward(steps);

    // Wait until movement finishes or limit triggered
    while (HAL_TIM_Base_GetState(&htim2) != HAL_TIM_STATE_READY &&
           !limit_triggered && !null_reached) {
        __NOP();
    }

    // Read voltage from LVDT ADC after motor move
        float voltage = read_voltage_ad7606();
        char voltage_str[32];
        sprintf(voltage_str, "OUTPUT-Volt: %.3f V", voltage);
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_WriteString(MyI2C_LCD, voltage_str);
       // HAL_Delay(2000);
        float voltage_out = read_voltage_ad7606();             // Output voltage (V)
        float displacement = voltage_to_distance(voltage_out); // Distance (mm)
        float excitation_voltage = read_excitation_voltage_from_keypad(); // User input V

        if (excitation_voltage > 0.0f && displacement != 0.0f) {
            // Sensitivity in V/mm/V
            float sensitivity_v = voltage_out / (excitation_voltage * displacement);

            // Sensitivity in mV/V/mm
            float sensitivity_mv = (voltage_out * 1000.0f) / (excitation_voltage * displacement);

            char display_str[32];
            // Display mV/V/mm on first line
            sprintf(display_str, "Sens: %.2f mV/V/mm", sensitivity_mv);
            I2C_LCD_Clear(MyI2C_LCD);
            I2C_LCD_SetCursor(MyI2C_LCD, 2, 0);
            I2C_LCD_WriteString(MyI2C_LCD, display_str);

            // Display V/mm/V on second line
            sprintf(display_str, "Sens: %.5f V/mm/V", sensitivity_v);
            I2C_LCD_SetCursor(MyI2C_LCD, 3, 0);
            I2C_LCD_WriteString(MyI2C_LCD, display_str);

            HAL_Delay(3000);  // Display both for 3 seconds
        } else {
            I2C_LCD_Clear(MyI2C_LCD);
            I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
            I2C_LCD_WriteString(MyI2C_LCD, "Sens calc failed");
            HAL_Delay(1500);


        }

        float measured_output_voltage = read_voltage_ad7606();            // Measured LVDT output voltage
        float required_output_voltage = read_required_output_voltage();   // User input expected output voltage

        if (required_output_voltage > 0.0f) {
            float linearity = measured_output_voltage / required_output_voltage;

            char linearity_str[32];
            // Display as percentage (%), multiplied by 100
            sprintf(linearity_str, "Linearity: %.2f%%", linearity * 100);

            I2C_LCD_Clear(MyI2C_LCD);
            I2C_LCD_SetCursor(MyI2C_LCD, 4, 0);
            I2C_LCD_WriteString(MyI2C_LCD, linearity_str);

            HAL_Delay(2000);
        }
        else
        {
            I2C_LCD_Clear(MyI2C_LCD);
            I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
            I2C_LCD_WriteString(MyI2C_LCD, "Invalid required V");
            HAL_Delay(1500);
        }
}



void cyclic_test()
{

	// Prompt for number of cycles
    char cycle_buf[6] = {0};  // corrected buffer size
    int idx = 0, entry_complete = 0;

    /*continous data Monitoring for calculating*/
    float Ltend_Vin = 0;
    float Rtend_Vin = 0;
    float Zero_Vin = 0;

    (void)Ltend_Vin;
    (void)Rtend_Vin;
    (void)Zero_Vin;

    /*voltage at start/initial*/
    float Ltstart_Vin = 0;
    float Rtstart_Vin = 0;
    float Zerost_Vin = 0;

    (void)Ltstart_Vin;

    float AvgRt = 0;
    float AvgLt = 0;


    float SpanDrift = 0;
    float ZeroDrift = 0;


    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "No. of cycles:");
    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);

    while (!entry_complete) {
        char key = Keypad_GetKey();
        if (key >= '0' && key <= '9' && idx < 5) {
            cycle_buf[idx++] = key;
            cycle_buf[idx] = '\0';
            I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
            I2C_LCD_WriteString(MyI2C_LCD, cycle_buf);
        } else if (key == '#') {
            if (idx > 0) entry_complete = 1;
        } else if (key == '*') {
            if (idx > 0) {
                idx--;
                cycle_buf[idx] = '\0';
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
                I2C_LCD_WriteString(MyI2C_LCD, "     "); // clear line
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
                I2C_LCD_WriteString(MyI2C_LCD, cycle_buf);
            }
        }
        HAL_Delay(40);
    }

    int n_cycles = atoi(cycle_buf);


    int OneScale_mm = total_strike_length_mm / 2 ;
    int step_count = distance_to_steps(OneScale_mm);

    if (n_cycles <= 0 || step_count <= 0) {
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_WriteString(MyI2C_LCD, "Invalid input");
        HAL_Delay(1200);
        return;
    }

    // Move for N cycles (fwd and back)
    for (int c = 1; c <= n_cycles; c++)
    {
    	/*Displaying the No of cycle*/
        char stat[32];  // corrected buffer size
        snprintf(stat, sizeof(stat), "Cycle:%d/%d ", c, n_cycles);
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, stat);


        /*hope its in Null and getting voltage value at Zero*/
                if(c==1 &&  read_voltage_ad7606() == 0) //if it is not exact zero here the condition can be like (-0.25v to 0.25v)
                {
                	 Zerost_Vin = read_voltage_ad7606();
                }

        // Forward movement
        I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Forward");
        set_motor_direction(1);
        move_motor_forward(OneScale_mm);
        HAL_Delay(300);


        /*first right end value for span drift*/
        if(c==1)
        {
             Rtstart_Vin = read_voltage_ad7606();
        }
        else
        {
        	AvgRt = ((read_voltage_ad7606() - Rtstart_Vin)/Rtstart_Vin) * 100;
        }


        // Reverse movement
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, stat);
        I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Reverse     ");
        set_motor_direction(0);
        move_motor_forward(OneScale_mm);
        HAL_Delay(300);


        if(c>1)
        {
        	ZeroDrift = ((read_voltage_ad7606() - Zerost_Vin )/ Zerost_Vin ) * 100;
        }

        // Reverse movement
        set_motor_direction(0);
                move_motor_forward(OneScale_mm);
                HAL_Delay(300);

        if(c==1)
        {
             Ltstart_Vin = read_voltage_ad7606();
        }
        else
        {
        	AvgLt = ((read_voltage_ad7606() - Rtstart_Vin)/Rtstart_Vin) * 100;
        }

        //Forward movement
        I2C_LCD_Clear(MyI2C_LCD);
                I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
                I2C_LCD_WriteString(MyI2C_LCD, stat);
                I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
                I2C_LCD_WriteString(MyI2C_LCD, "forward     ");
                set_motor_direction(1);
                move_motor_forward(OneScale_mm);
                HAL_Delay(300);


        SpanDrift = (AvgLt+AvgRt)/2;



        if(SpanDrift > 0.25)
        {
        	I2C_LCD_Clear(MyI2C_LCD);
        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        	snprintf(stat,sizeof(stat),"SpanDrift is %.2f", SpanDrift);
        	I2C_LCD_WriteString(MyI2C_LCD,stat);
        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
        	I2C_LCD_WriteString(MyI2C_LCD, "The cyclic test is Halting");
        	break;
        }
        else if (ZeroDrift > 0.25)
        {
        	I2C_LCD_Clear(MyI2C_LCD);
        	        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        	        	snprintf(stat,sizeof(stat),"SpanDrift is %.2f", ZeroDrift);
        	        	I2C_LCD_WriteString(MyI2C_LCD,stat);
        	        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
        	        	I2C_LCD_WriteString(MyI2C_LCD, "The cyclic test is Halting");
        	        	break;
        }
        else
        {
        	I2C_LCD_Clear(MyI2C_LCD);
        	        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        	        	snprintf(stat,sizeof(stat),"SpanDrift is %.2f", SpanDrift);
        	        	I2C_LCD_WriteString(MyI2C_LCD,stat);
        	        	I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
        	        	snprintf(stat,sizeof(stat),"ZeroDrift is %.2f", ZeroDrift);
        	        	I2C_LCD_WriteString(MyI2C_LCD, stat);
        }



    }

    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_WriteString(MyI2C_LCD, "Cyclic done!");
    HAL_Delay(2000);

}

void display_menu(void)
{
	I2C_LCD_Clear(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD,0,0);
	I2C_LCD_WriteString(MyI2C_LCD, "1.RANDOM TEST");
	I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
	I2C_LCD_WriteString(MyI2C_LCD, "2.CYCLIC");
	    // Some 16x2 LCD only has 2 lines, so show third option shifted alternatively
	I2C_LCD_SetCursor(MyI2C_LCD, 2, 0); // If your LCD supports 4 lines, else ignore
	I2C_LCD_WriteString(MyI2C_LCD, "3.CALIBRATION");
}

void perform_task(int option)
{
	I2C_LCD_Clear(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	switch(option){
			case 1:
	            I2C_LCD_WriteString(MyI2C_LCD, "Random Test Running");
	            // Place your Random Test code here
	            handle_null_displacement();
	            break;
	        case 2:
	            I2C_LCD_WriteString(MyI2C_LCD, "Cyclic Test Running");
	            //  Place your Cyclic Test code here
	            cyclic_test();

	            break;
	        case 3:
	            I2C_LCD_WriteString(MyI2C_LCD, "Calibration Running");
	            //Place your Calibration code here
	            handle_null_displacement();
	            break;
	        default:
	            I2C_LCD_WriteString(MyI2C_LCD, "Invalid Option");

	            break;
	}
}

// Menu loop runs infinitely to show menu and capture input
void menu_loop(void)
{
    char key = 0;
    display_menu();
    while (1) {
        key = Keypad_GetKey();
        if (key) {
            if (key == '1' || key == '2' || key == '3') {
                int option = key - '0';
                perform_task(option);
                display_menu(); // Redisplay after task ends
            }
        }
        HAL_Delay(100);
    }
}

void Null_Process()
{
    while (1) {
    	// 1. Read distance value to move
        float dist_mm = read_distance_from_keypad(); // Prompt user for distance
        if(dist_mm < 0.0f || dist_mm > 300.0f)
        {
        	I2C_LCD_Clear(MyI2C_LCD);
        	I2C_LCD_WriteString(MyI2C_LCD, "Invalid Distance");
        	HAL_Delay(1500);
        	continue;
        }
        int steps = distance_to_steps(dist_mm);
        move_motor_forward(steps);

        //2.Read ADC and display voltage
        float voltage = read_voltage_ad7606();
        char voltage_str[16];
        sprintf(voltage_str, "V=%.3f", voltage);

        //distance the distance
        /*
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Dist(mm):");
        char d_str[10];
        sprintf(d_str, "%.2f", dist_mm);
        I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
        I2C_LCD_WriteString(MyI2C_LCD, d_str);

        HAL_Delay(1500);
        */
        // Calculate distance in mm
        float distance_traveled_mm = ((float)step_count / (STEPS_PER_REV * MICROSTEPS)) * BALLSCREW_PITCH_MM;

        // Display distance on the LCD
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Dist(mm):");

        char d_str[10];
        sprintf(d_str, "%.2f", distance_traveled_mm);
        I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
        I2C_LCD_WriteString(MyI2C_LCD, d_str);

        HAL_Delay(1500);

        //Display voltage
        I2C_LCD_Clear(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Voltage:");
        I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);
        I2C_LCD_WriteString(MyI2C_LCD, voltage_str);

        HAL_Delay(2000);

        // 3. Reference check
                if(fabsf(voltage) == 0.0f)
                {
                    I2C_LCD_Clear(MyI2C_LCD);
                    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
                    I2C_LCD_WriteString(MyI2C_LCD, "Reference point");
                    HAL_Delay(2000);
                    break;
                }
    /* USER CODE BEGIN 3 */
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  keypad_Init();               // Configure keypad GPIOs
  I2C_LCD_Init(MyI2C_LCD);


  I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
  I2C_LCD_WriteString(MyI2C_LCD, "STARTING");
  HAL_Delay(1000);

  I2C_LCD_Clear(MyI2C_LCD);
  I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
  I2C_LCD_WriteString(MyI2C_LCD,"HOMING");
  HAL_Delay(1000);

  Homing();

  I2C_LCD_Clear(MyI2C_LCD);
  I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
  I2C_LCD_WriteString(MyI2C_LCD,"HOMING COMPLETED");
  HAL_Delay(1000);


 //Callibrating function
  calibrate_ballscrew();

 //To read total stroke length  from the user

    int totalLength = read_total_stroke_length();
    char inputStr[24];
    sprintf(inputStr, "%d", totalLength);

    // Clear LCD, set cursor, write label and value
    I2C_LCD_Clear(MyI2C_LCD);                          // Clear display
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);                // First row, first column
    I2C_LCD_WriteString(MyI2C_LCD, "Stroke Length:");  // Display label

    I2C_LCD_SetCursor(MyI2C_LCD, 1, 0);                // Second row, first column
    I2C_LCD_WriteString(MyI2C_LCD, inputStr);          // Display value

    I2C_LCD_Clear(MyI2C_LCD);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "System Ready");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    I2C_LCD_Clear(MyI2C_LCD);

    I2C_LCD_SetCursor(MyI2C_LCD , 0 , 0);

    I2C_LCD_WriteString(MyI2C_LCD,"Starting the process");



    Null_Process();

  menu_loop();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CONVST_Pin|RESET_Pin|LED2_Pin|Direction_pin_Pin
                          |Enable_pin_Pin|Step_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Chip_select_1_Pin|COL1_Pin|COL2_Pin|COL3_Pin
                          |COL4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONVST_Pin RESET_Pin LED2_Pin Direction_pin_Pin
                           Enable_pin_Pin Step_pin_Pin */
  GPIO_InitStruct.Pin = CONVST_Pin|RESET_Pin|LED2_Pin|Direction_pin_Pin
                          |Enable_pin_Pin|Step_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Limit_pin_Pin BUSY_Pin */
  GPIO_InitStruct.Pin = Limit_pin_Pin|BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW4_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW3_Pin */
  GPIO_InitStruct.Pin = ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROW3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Chip_select_1_Pin */
  GPIO_InitStruct.Pin = Chip_select_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Chip_select_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin COL4_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin|COL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
