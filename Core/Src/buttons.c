/*
 * butons.c
 *
 *  Created on: Jul 4, 2022
 *      Author: Gesislab
 */
#include "main.h"
#include "buttons.h"

uint8_t read_button(uint8_t ch){
	GPIO_PinState ret = GPIO_PIN_RESET;
	if(ch == 1){
		ret = HAL_GPIO_ReadPin(button1_GPIO_Port, GPIO_PIN_3);
	}
	else if (ch == 2){
		ret = HAL_GPIO_ReadPin(button2_GPIO_Port, GPIO_PIN_0);
	}

	return ret;
}

uint8_t write_button(uint8_t ch){

	if(ch == 1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(200);
		}
	else if (ch == 2){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_Delay(1000);
		}



}
