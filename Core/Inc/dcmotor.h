/*
 * motor.h
 *
 *  Created on: Sep 27, 2021
 *      Author: mg
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define ACTION_STOP 0
#define ACTION_FORWARD 1
#define ACTION_BACKWARD 2
#define ACTION_STEP_FORWARD 3
#define ACTION_STEP_BACKWARD 4
#define ACTION_STEP_RIGHT 5
#define ACTION_STEP_LEFT 6

#define WHEEL_MOTOR_NUMBER 4

typedef struct S_MotorType{
	uint32_t MOTOR_PORT;
	uint32_t MOTOR_F_PIN; //forward
	uint32_t MOTOR_B_PIN; //backward
	uint8_t action; //forward, backward, stop
	uint32_t velocity; //current velocity: 0-100
	uint32_t initial_velocity;
	uint32_t target_velocity;
	uint8_t step_number;
	uint8_t state_motion;
	uint8_t no_move;
}MotorType;

void initWheelMotors();
void motor_pwm_timer_callback();
void motor_forward(uint32_t velocity);
void motor_backward(uint32_t velocity);
void motor_right(uint32_t velocity);
void motor_left(uint32_t velocity);
void motor_action(uint8_t motor_number, uint8_t action, uint8_t velocity, uint8_t step_number);
void motor_all_action(uint8_t action, uint8_t velocity, uint8_t step);
uint8_t get_f_send_motor_ack();
void rst_f_send_motor_ack();
void motor_reset_action(uint32_t velocity);//Temporary
#endif /* INC_MOTOR_H_ */
