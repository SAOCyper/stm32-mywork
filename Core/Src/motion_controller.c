/*
 * motor_controller.c
 *
 *  Created on: 20 Haz 2022
 *      Author: Gesislab
 */

#include "string.h"
#include "main.h"
#include "motion_controller.h"
#include "stdlib.h"
#include "Mpu6050.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim3;

uint8_t motion_q_elem_counter = 0;
motion_cmd_t motion_q[MAX_MOTION_QUEUE_ELEMENTS];
motion_cmd_t current_processing_cmd;



void push_cmd_to_motion_q(motion_cmd_t* elem){
	motion_cmd_t* cmd;
	if(motion_q_elem_counter<MAX_MOTION_QUEUE_ELEMENTS){
		//make velocity scaling
		if(elem->velocity > 100)
		{
			elem->velocity = 100;
		}

		cmd = &motion_q[motion_q_elem_counter];
		cmd->done = 0;
		cmd->command = elem->command;
		cmd->angle = elem->angle;

		cmd->velocity = elem->velocity *10; //velocity is mapped as 0-100 -> 0-1000
		cmd->duration = elem->duration *100; //each step is 100 ms

		motion_q_elem_counter++;
	}
}

motion_cmd_t* pop_cmd_from_motion_q(){
	motion_cmd_t* cmd;
	if(motion_q_elem_counter > 0){
		cmd = &motion_q[--motion_q_elem_counter];
		memcpy(&current_processing_cmd, cmd, sizeof(motion_cmd_t));
		return &current_processing_cmd;
	}

	return 0;
}

void motion_forward(motion_cmd_t* cmd){
	static uint8_t state = 0;
	static uint32_t velocity = 0;
	static uint32_t stop_tick;

	switch(state){
	case 0: //reset
		htim1.Instance->CCR1 = 0 ;
		htim15.Instance->CCR2 = 0;
		state = 1;
		break;
	case 1: //istedigim hıza kadar yumuşak kalkış
		for(velocity; velocity < cmd->velocity; velocity += 100){
			htim1.Instance->CCR1 = velocity;
			htim15.Instance->CCR2 = velocity;
		}
		state = 2;
		break;
	case 2://istenilen süreyi hesapla
		stop_tick = HAL_GetTick() + cmd->duration;
		state = 3;
		break;

	case 3: //istenilen süre kadar harekete devam et
		if(HAL_GetTick() >= stop_tick){
			htim1.Instance->CCR1 = 0 ;
			htim15.Instance->CCR2 = 0;
			cmd->done = 1;
			velocity = 0;
			state = 0;
		}
	}
}

void motion_backward(motion_cmd_t* cmd){
	static uint8_t state = 0;
	static uint32_t velocity = 0;
	static uint32_t stop_tick;

	switch(state){
	case 0: //reset
		htim1.Instance->CCR2 = 0 ;
		htim15.Instance->CCR1 = 0;
		state = 1;
		break;
	case 1: //istedigim hıza kadar yumuşak kalkış
		for(velocity; velocity < cmd->velocity; velocity += 100){
			htim1.Instance->CCR2 = velocity;
			htim15.Instance->CCR1 = velocity;
		}
		state = 2;
		break;
	case 2://istenilen süreyi hesapla
		stop_tick = HAL_GetTick() + cmd->duration;
		state = 3;
		break;

	case 3: //istenilen süre kadar harekete devam et
		if(HAL_GetTick() >= stop_tick){
			htim1.Instance->CCR2 = 0 ;
			htim15.Instance->CCR1 = 0;
			cmd->done = 1;
			velocity = 0;
			state = 0;
		}
	}
}

void motion_right(motion_cmd_t* cmd){
	static uint8_t state = 0;
	static uint32_t velocity = 0;
	static uint32_t stop_tick;

	switch(state){
	case 0: //reset
		htim1.Instance->CCR1 = 0 ;
		htim15.Instance->CCR1 = 0;
		state = 1;
		break;
	case 1: //istedigim hıza kadar yumuşak kalkış
		for(velocity; velocity < cmd->velocity; velocity += 100){
			htim1.Instance->CCR1 = velocity;
			htim15.Instance->CCR1 = velocity;
		}
		state = 2;
		break;
	case 2://istenilen süreyi hesapla
		stop_tick = HAL_GetTick() + cmd->duration;
		state = 3;
		break;
	case 3: //istenilen süre kadar harekete devam et
		if(HAL_GetTick() >= stop_tick){
			htim1.Instance->CCR1 = 0 ;
			htim15.Instance->CCR1 = 0;
			cmd->done = 1;
			velocity = 0;
			state = 0;
		}
	}
}

void motion_left(motion_cmd_t* cmd){
	static uint8_t state = 0;
	static uint32_t velocity = 0;
	static uint32_t stop_tick;

	switch(state){
	case 0: //reset
		htim1.Instance->CCR2 = 0 ;
		htim15.Instance->CCR2 = 0;
		state = 1;
		break;
	case 1: //istedigim hıza kadar yumuşak kalkış
		for(velocity; velocity < cmd->velocity; velocity += 100){
			htim1.Instance->CCR2 = velocity;
			htim15.Instance->CCR2 = velocity;
		}
		state = 2;
		break;
	case 2://istenilen süreyi hesapla
		stop_tick = HAL_GetTick() + cmd->duration;
		state = 3;
		break;
	case 3: //istenilen süre kadar harekete devam et
		if(HAL_GetTick() >= stop_tick){
			htim1.Instance->CCR2 = 0 ;
			htim15.Instance->CCR2 = 0;
			cmd->done = 1;
			velocity = 0;
			state = 0;
		}
	}
}
void motion_tracking(){

}
void do_motion_cmd(motion_cmd_t* cmd){
	if(cmd->command == MOTION_CMD_FORWARD){
		motion_forward(cmd);
	}

	else if(cmd->command == MOTION_CMD_BACKWARD){
		motion_backward(cmd);
	}
	else if(cmd->command == MOTION_CMD_TURNRIGHT){
		motion_right(cmd);
	}
	else if(cmd->command == MOTION_CMD_TURNLEFT){
		motion_left(cmd);
	}
}

int motion_control(){
	static uint8_t state;
	static motion_cmd_t* cmd;
	switch(state){
	case 100: //Init state
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
		  state = 0;
		break;
	case 0: //check for q
		if(motion_q_elem_counter > 0){
			state = 1;
		}
		break;

	case 1:
		cmd = pop_cmd_from_motion_q();
		state = 2;
		break;

	case 2:
		if(cmd != 0){
			do_motion_cmd(cmd);
			if(cmd->done == 1){
				state = 0;
			}
		}
		break;
	default:
		break;
	}
}


