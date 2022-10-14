/*
 * motion_controller.h
 *
 *  Created on: 20 Haz 2022
 *      Author: Gesislab
 */

#ifndef INC_MOTION_CONTROLLER_H_
#define INC_MOTION_CONTROLLER_H_

#define MAX_MOTION_QUEUE_ELEMENTS 16

typedef struct{
	uint8_t done;
	uint16_t command;
	uint16_t velocity;
	uint16_t angle;
	uint16_t duration;
} motion_cmd_t;


enum {
	MOTION_CMD_FORWARD,
	MOTION_CMD_BACKWARD,
	MOTION_CMD_TURNRIGHT,
	MOTION_CMD_TURNLEFT,
	MOTION_CMD_SERVOTWO
};

int motion_control					();
void motion_tracking				();

#endif /* INC_MOTION_CONTROLLER_H_ */
