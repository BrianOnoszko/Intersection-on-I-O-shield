/**
******************************************************
@brief		test, functions for test-Program
@file		test.c
@author		Brian Onoszko
@version	1.0
@date		6-December-2023
@brief		Functions and structures made for or
			associated with testing
******************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "project_functions.h"
#include "spi.h"
#include "test.h"
#include "gpio.h"

/**
 * @brief	Test_program, main program to run tests
 *
 * @param	null
 *
 * @return 	void
 */
void Test_program(){
	Test_Led_SPI();
	Test_Switches();
	Test_Interrupt();
}

/**
 * @brief	Test_Led_SPI, tests several case that ensures that communication via SPI works
 * @brief	Order: Flash all, all off, all traffic lights one by one and both pedestrian light pairs, no user interaction simply tests that SPI can communicate with all LEDs and that all LEDs are in working order
 * @param 	null
 *
 * @return 	void
 */
void Test_Led_SPI(){
	shift_reg_data[0] = 0b11111111;
	shift_reg_data[1] = 0b11111111;
	shift_reg_data[2] = 0b11111111;
	write_to_SPI();

	HAL_Delay(1000);

	shift_reg_data[0] = 0b0;
	shift_reg_data[1] = 0b0;
	shift_reg_data[2] = 0b0;
	write_to_SPI();

	HAL_Delay(200);

	//Shift reg 1 - U3
	shift_reg_data[0] = 0b1;
	write_to_SPI();
	HAL_Delay(200);
	for(int i = 0; i < 7; i++){
		shift_reg_data[0] <<= 1;
		write_to_SPI();
		HAL_Delay(200);
	}
	shift_reg_data[0] = 0b0;


	//shift reg 2 - U2
	shift_reg_data[1] = 0b1;
	write_to_SPI();
	HAL_Delay(200);
	for(int i = 0; i < 2; i++){
		shift_reg_data[1] <<= 1;
		write_to_SPI();
		HAL_Delay(200);
	}
	shift_reg_data[1] = 0b0;


	//shift reg 2 - U1
	shift_reg_data[2] = 0b1;
	write_to_SPI();
	HAL_Delay(200);
	for(int i = 0; i < 2; i++){
		shift_reg_data[2] <<= 1;
		write_to_SPI();
		HAL_Delay(200);
	}
	shift_reg_data[2] = 0b0;

	shift_reg_data[1] = 0b1000;
	write_to_SPI();
	HAL_Delay(200);
	for(int i = 0; i < 2; i++){
		shift_reg_data[1] <<= 1;
		write_to_SPI();
		HAL_Delay(200);
	}
	shift_reg_data[1] = 0b0;

	shift_reg_data[2] = 0b1000;
	write_to_SPI();
	HAL_Delay(200);
	for(int i = 0; i < 2; i++){
		shift_reg_data[2] <<= 1;
		write_to_SPI();
		HAL_Delay(200);
	}
	shift_reg_data[2] = 0b0;
	write_to_SPI();
}

/**
 * @brief	Test_Switches, function to test input peripherals
 * @brief	Switches change nearby traffic light to green or red depending on state, if all switches have triggered a green light then test passes
 *
 * @param 	null
 *
 * @return 	void
 */
void Test_Switches(){
	int tl_north = 0;
	int tl_south = 0;
	int tl_west = 0;
	int tl_east = 0;

	int i = 0;



	shift_reg_data[0] = 0b1001;
	shift_reg_data[1] = 0b1;
	shift_reg_data[2] = 0b1;
	write_to_SPI();
	while(i == 0){
		if(HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin) == GPIO_PIN_RESET){
			shift_reg_data[0] &= 0b11000111;
			shift_reg_data[0] |= 0b00100000;
			tl_north = 1;
		} else {
			shift_reg_data[0] &= 0b11000111;
			shift_reg_data[0] |= 0b00001000;
		}
		if(HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin) == GPIO_PIN_RESET){
			shift_reg_data[0] &= 0b11111000;
			shift_reg_data[0] |= 0b00000100;
			tl_east = 1;
		} else {
			shift_reg_data[0] &= 0b11111000;
			shift_reg_data[0] |= 0b00000001;
		}
		if(HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin) == GPIO_PIN_RESET){
			shift_reg_data[1] &= 0b11111000;
			shift_reg_data[1] |= 0b00000100;
			tl_south = 1;
		} else {
			shift_reg_data[1] &= 0b11111000;
			shift_reg_data[1] |= 0b00000001;
		}
		if(HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin) == GPIO_PIN_RESET){
			shift_reg_data[2] &= 0b11111000;
			shift_reg_data[2] |= 0b00000100;
			tl_west = 1;
		} else {
			shift_reg_data[2] &= 0b11111000;
			shift_reg_data[2] |= 0b00000001;
		}
		write_to_SPI();
		i = tl_north*tl_west*tl_east*tl_south;
	}
	HAL_Delay(1000);
	clear_tl_vertical();
	clear_tl_horizontal();
	write_to_SPI();
}

/**
 * @brief	Test_Interrupt, test function for interrupts
 * @brief	Uses the project_function declared Interrupt handler which triggers by button input, interrupt triggers lights to turn on indicating that the test passed
 *
 * @param	Null
 *
 * @return	Void
 */
void Test_Interrupt(){
	test_var = 0;

	shift_reg_data[0] = 0b0;
	shift_reg_data[1] = 0b0;
	shift_reg_data[2] = 0b0;
	write_to_SPI();

	while(test_var == 0){}
	shift_reg_data[2] &= 0b11111000;
	shift_reg_data[2] |= 0b00000100;
	shift_reg_data[1] &= 0b11111000;
	shift_reg_data[1] |= 0b00000100;
	shift_reg_data[0] &= 0b11000000;
	shift_reg_data[0] |= 0b00100100;
	write_to_SPI();
	HAL_Delay(1000);
}
