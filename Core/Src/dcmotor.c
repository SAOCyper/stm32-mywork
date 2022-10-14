/*
 * motor.c
 *
 *  Created on: Sep 27, 2021
 *      Author: mg
 */
#include "main.h"
#include "dcmotor.h"
//#include "TIM.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;

MotorType wheelMotors[WHEEL_MOTOR_NUMBER];

uint8_t f_motor_new_action = 0;

uint8_t get_f_motor_new_action(){return f_motor_new_action;}
void set_f_motor_new_action(){f_motor_new_action = 1;}
void rst_f_motor_new_action(){f_motor_new_action = 0;}

void initWheelMotors(){
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
}

void motor_reset_action(uint32_t velocity)
{
	if((htim1.Instance->CCR1 == velocity) & (htim15.Instance->CCR2 == velocity))
	{
		htim1.Instance->CCR1 = 0 ;
		htim15.Instance->CCR2 = 0;
	}
	else if((htim1.Instance->CCR2 == velocity) & (htim15.Instance->CCR1 == velocity))
	{
		htim1.Instance->CCR2 = 0;
		htim15.Instance->CCR1 = 0;
	}
	else if((htim1.Instance->CCR1 == velocity) & (htim15.Instance->CCR1 == velocity))
	{
		htim1.Instance->CCR1 = 0;
		htim15.Instance->CCR1 =0;
	}
	else if((htim1.Instance->CCR2 ==velocity) & (htim15.Instance->CCR2 == velocity))
	{
		htim1.Instance->CCR2 = 0;
		htim15.Instance->CCR2 = 0;
	}

}

void motor_forward(uint32_t velocity)
{
	//int velocity2 = 0;
	//velocity2 = velocity - (velocity/3);
	htim1.Instance->CCR1 = velocity; //1->forward, 2->backward
	htim15.Instance->CCR2 = velocity;
}

void motor_backward(uint32_t velocity)
{
	//int velocity2 = 0;
	//velocity2 = velocity - (velocity/3);
	htim1.Instance->CCR2 = velocity;
	htim15.Instance->CCR1 = velocity;
}

void motor_right(uint32_t velocity)
{
	//int velocity2 = 0;
	//velocity2 = velocity - (velocity/3);
	htim1.Instance->CCR1 = velocity;
	htim15.Instance->CCR1 = velocity;
}

void motor_left(uint32_t velocity)
{
	//int velocity2 = 0;
	//velocity2 = velocity - (velocity/3);
	htim1.Instance->CCR2 = velocity;
	htim15.Instance->CCR2 = velocity;
}
//motor_number: 1,2
//action: ileri, geri, dur
//hiz: 0-100
void motor_action(uint8_t motor_number, uint8_t action, uint8_t velocity, uint8_t step_number){

	for(int i = 0; i< step_number;i++){
		if(action == ACTION_STEP_RIGHT){
			motor_right(velocity);
			step_number --;

		}
		else if(action == ACTION_STEP_LEFT){
			motor_left(velocity);
			step_number --;

		}
		else if(action == ACTION_STEP_FORWARD){
			motor_forward(velocity);
			step_number --;

		}
		else{
			motor_backward(velocity);
			step_number --;

		}

	}
}

void motor_all_action(uint8_t action, uint8_t velocity, uint8_t step){
	while(step>0){
		if(action == ACTION_STEP_RIGHT){
			motor_action(0,action,velocity,step);
			motor_action(1,action,0,step);
		}
		else if(action == ACTION_STEP_LEFT){
			motor_action(0,action,0,step);
			motor_action(1,action,velocity,step);
		}
		else{
			motor_action(0,action,velocity,step);
			motor_action(1,action,velocity,step);
		}
	}
}


void motor_pwm_timer_callback()
{

}

void test_motor()
{
	for(uint8_t i = 0;i<WHEEL_MOTOR_NUMBER;i++){
		motor_action(i, ACTION_FORWARD, 30,5);
		HAL_Delay(5000);
		motor_action(i, ACTION_FORWARD, 70,5);
		HAL_Delay(5000);
		motor_action(i, ACTION_BACKWARD, 30,5);
		HAL_Delay(5000);
		motor_action(i, ACTION_BACKWARD, 70,5);
		HAL_Delay(5000);
		motor_action(i, ACTION_STOP, 0,0);
		HAL_Delay(5000);

	}
}
