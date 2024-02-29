/**
 ****************************************************************
 @brief		project_functions, functions for the main program tasks
 @file		project_functions.c
 @author	Brian Onoszko
 @version 	1.0
 @date		06-December-2023
 @brief		Functions and variables associated with
 	 	 	the logic behind the project
 ****************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "project_functions.h"
#include "freertos.h"

/**
 * @brief	Array with 3 8-bit words, used for shift register
 */
uint8_t shift_reg_data[] = {0b00000000, 0b00000000, 0b00000000};	//North&East, South, West

/**
 * @brief	Variables associated with crossing logic and delays
 */
uint8_t waiting_pl_north; 	//1 == Pedestrian, 0 == No pedestrian
uint8_t waiting_pl_west; 	//1 == Pedestrian, 0 == No pedestrian
uint8_t interrupt_pl_north;
uint8_t interrupt_pl_west;
uint8_t greenMax = 0;

uint8_t waiting_tl_vertical; 	//1 == Car, 0 == No car
uint8_t waiting_tl_horizontal; 	//1 == Car, 0 == No car


uint8_t status_tl_vertical = 1;
uint8_t status_tl_horizontal = 0;
uint8_t status_pl_north = 0; 		//1 == Green, 0 == Red
uint8_t status_pl_west = 0;

uint32_t orangeDelay = 500; 	//How long traffic lights remain yellow
uint32_t walkingDelay = 3000;	//How long pedestrian lights should be green
uint32_t greenDelay = 8000; 	//How often the green lane changes when no cars are waiting
uint32_t redDelayMax = 5000;	//The max time a car waits before it gets a green light
uint32_t toggleFreq = 500;	//How quickly the pedestrian light blinks
uint32_t pedestrianDelay = 8000;

/**
 * @brief 	Testing associated variable for interrupt
 */
int test_var;

/**
 * @brief write to SPI, shortcut for transmitting and clocking the shift register
 *
 * @param null
 *
 * @retval void
 */
void write_to_SPI(){
	HAL_SPI_Transmit(&hspi3, shift_reg_data, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(STCP_595_GPIO_Port, STCP_595_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STCP_595_GPIO_Port, STCP_595_Pin, GPIO_PIN_RESET);
}

/**
 * @brief clear vertical traffic lights, clears bits associated with vertical traffic lights
 *
 * @param null
 *
 * @retval void
 */
void clear_tl_vertical(){
	shift_reg_data[0] &= 0b11000111;	//North - East
	shift_reg_data[1] &= 0b11111000;	//South
}

/**
 * @brief clear horizontal traffic lights, clears bits associated with horizontal traffic lights
 *
 * @param null
 *
 * @retval void
 */
void clear_tl_horizontal(){
	shift_reg_data[0] &= 0b11111000;	//North - East
	shift_reg_data[2] &= 0b11111000;	//West
}

/**
 * @brief change pedestrian lights north, changes the bits for the northern crossing to opposite state, checks the "status_pl_north" flag to know which state it currently is in
 *
 * @param null
 *
 * @retval void
 */
void change_pl_north(){
	if(status_pl_north == 1){
		shift_reg_data[1] &= 0b11000111;
		shift_reg_data[1] |= 0b00001000;
		write_to_SPI();
	} else if(status_pl_north == 0){
		shift_reg_data[1] &= 0b11000111;
		shift_reg_data[1] |= 0b00010000;
		write_to_SPI();
	}
}

/**
 * @brief change pedestrian lights west, changes the bits for the west crossing to opposite state, checks the "status_pl_west" flag to determine current state
 *
 * @param null
 *
 * @retval void
 */
void change_pl_west(){
	if(status_pl_west == 1){
		shift_reg_data[2] &= 0b11000111;
		shift_reg_data[2] |= 0b00001000;
		write_to_SPI();
	} else if(status_pl_west == 0){
		shift_reg_data[2] &= 0b11000111;
		shift_reg_data[2] |= 0b00010000;
		write_to_SPI();
	}
}

/**
 * @brief change traffic lights vertical, changes state of vertical traffic lights by checking state of "status_tl_vertical", employs freertos osDelay to allow the yellow signal to have proper delay
 *
 * @param null
 *
 * @retval void
 */
void change_tl_vertical(){
	if(status_tl_vertical == 1){
		clear_tl_vertical();
		shift_reg_data[0] |= 0b00010000;
		shift_reg_data[1] |= 0b00000010;
		write_to_SPI();
		osDelay(orangeDelay);
		clear_tl_vertical();
		shift_reg_data[0] |= 0b00001000;
		shift_reg_data[1] |= 0b00000001;
		write_to_SPI();
		status_tl_vertical = 0;
	}else if(status_tl_vertical == 0){
		clear_tl_vertical();
		shift_reg_data[0] |= 0b00010000;
		shift_reg_data[1] |= 0b00000010;
		write_to_SPI();
		osDelay(orangeDelay);
		clear_tl_vertical();
		shift_reg_data[0] |= 0b00100000;
		shift_reg_data[1] |= 0b00000100;
		write_to_SPI();
		status_tl_vertical = 1;
	}
}

/**
 * @brief change traffic lights horizontal, changes state of horizontal traffic lights by checking state of "status_tl_horizontal", employs freertos osDelay to allow the yellow signal to have proper delay
 *
 * @param null
 *
 * @retval void
 */
void change_tl_horizontal(){
	if(status_tl_horizontal == 1){
		clear_tl_horizontal();
		shift_reg_data[0] |= 0b00000010;
		shift_reg_data[2] |= 0b00000010;
		write_to_SPI();
		osDelay(orangeDelay);
		clear_tl_horizontal();
		shift_reg_data[0] |= 0b00000001;
		shift_reg_data[2] |= 0b00000001;
		write_to_SPI();
		status_tl_horizontal = 0;
	}else if(status_tl_horizontal == 0){
		clear_tl_horizontal();
		shift_reg_data[0] |= 0b00000010;
		shift_reg_data[2] |= 0b00000010;
		write_to_SPI();
		osDelay(orangeDelay);
		clear_tl_horizontal();
		shift_reg_data[0] |= 0b00000100;
		shift_reg_data[2] |= 0b00000100;
		write_to_SPI();
		status_tl_horizontal = 1;
	}
}

/**
 * @brief toggle pedestrian lights north, toggles the waiting light for pedestrians, simply checks state of the bit associated with the blue light
 *
 * @param null
 *
 * @retval void
 */
void toggle_pl_north(){
	if((shift_reg_data[1] & 0b00100000) == 0b00100000){
		shift_reg_data[1] &= 0b11011111;
		write_to_SPI();
	} else {
		shift_reg_data[1] |= 0b00100000;
		write_to_SPI();
	}
}

/**
 * @brief toggle pedestrian lights west, toggles the waiting light for pedestrians, simply checks state of the bit associated with the blue light
 *
 * @param null
 *
 * @retval void
 */
void toggle_pl_west(){
	if((shift_reg_data[2] & 0b00100000) == 0b00100000){
		shift_reg_data[2] &= 0b11011111;
		write_to_SPI();
	} else {
		shift_reg_data[2] |= 0b00100000;
		write_to_SPI();
	}
}

/**
 * @brief	HAL GPIO external interrupt callback, used instead of the weak predefined one, crosschecks which pin set of the interrupt and triggers a flag acordingly
 *
 * @param	uint16_t GPIO_Pin, address of external interrupt pin
 *
 * @return	void
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//test_var = 1;
	if(GPIO_Pin == PL_North_Switch_Pin){
		if(interrupt_pl_north == 0){
			interrupt_pl_north = 1;
		}
	}
	if(GPIO_Pin == PL_West_Switch_Pin){
		if(interrupt_pl_west == 0){
			interrupt_pl_west = 1;
		}
	}
}



