/*
 * parser.c
 *
 *  Created on: Sep 16, 2021
 *      Author: mg
 */

#include "main.h"
#include "uart_comm.h"
#include "dcmotor.h"
#include "parser.h"
#include "motion_controller.h"

extern UART_HandleTypeDef huart2;

#define UART_CMD_DC_MOTOR 			0
#define UART_CMD_LED_MATRIX 		1

#define UART_CMD_DC_STEP_FORWARD 	41
#define UART_CMD_DC_STEP_BACKWARD 	42
#define UART_CMD_DC_STEP_RIGHT 		43
#define UART_CMD_DC_STEP_LEFT 		44

#define UART_CMD_LED_MATRIX_ON 		50
#define UART_CMD_LED_MATRIX_OFF 	51
#define UART_CMD_LED_MATRIX_SAD 	52
#define UART_CMD_LED_MATRIX_HAPPY 	53
#define UART_CMD_LED_MATRIX_CROSS 	54
#define UART_CMD_LED_MATRIX_ANGRY 	55
#define UART_CMD_LED_MATRIX_HEART 	56

#define UART_CMD_READ_BTN1 57
#define UART_CMD_READ_BTN2 58

#define UART_CMD_WRITE_BTN1 59
#define UART_CMD_WRITE_BTN2 60

void process_uart_data(uint8_t* buf, uint8_t* resp_data){
	uint8_t len = buf[0];
	uint8_t cmd = buf[1];
	motion_cmd_t motion_cmd;
	switch (cmd){

	case UART_CMD_DC_MOTOR:
		motion_cmd.done = 0;
		motion_cmd.command = buf[2];
		motion_cmd.velocity = buf[3];
		motion_cmd.duration = buf[4];
		motion_cmd.angle = buf[5];
		push_cmd_to_motion_q(&motion_cmd);
		break;

	case UART_CMD_LED_MATRIX_HAPPY:
		break;

	case UART_CMD_LED_MATRIX_SAD:

		break;

	case UART_CMD_LED_MATRIX_ANGRY:

		break;

	case UART_CMD_LED_MATRIX:

		break;

	case UART_CMD_LED_MATRIX_ON:

		break;

	case UART_CMD_LED_MATRIX_OFF:

		break;

	case UART_CMD_LED_MATRIX_CROSS:
		break;

	case UART_CMD_LED_MATRIX_HEART:

		break;
	case UART_CMD_READ_BTN1:
		//buton1 değerini okuyup esp'ye gönderecem
		resp_data[0] = 2;
		resp_data[1] = read_button(1);
		break;
	case UART_CMD_READ_BTN2:
		//buton2 değerini okuyup esp'ye gönderecem
		resp_data[0] = 2;
		resp_data[1] = read_button(2);
		break;
	case UART_CMD_WRITE_BTN1:
		write_button(1);
		break;

	case UART_CMD_WRITE_BTN2:
		write_button(2);
		break;

	}
}

void parse_uart_data(uint8_t* resp_data){
	Uart_Ch* uch = get_uart_ch();

	if(uch->fdata_ready){
		//uart_send_data(&huart1, uch->rx_data_buf, uch->rx_data_cntr);
		process_uart_data(&(uch->rx_data_buf[2]), resp_data);
		set_f_send_uart_ack();
		uch->fdata_ready = 0;
	}
}

