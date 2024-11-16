/*
 * servo.h
 *
 *  Created on: Nov 6, 2024
 *      Author: lilian.jaouanne
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

#define POS_INIT_PAN	85
#define POS_INIT_TILT	60

#define MOVING_TIME 1000 //ms

typedef enum{
  tilt,
  pan,
}servo_axis;

typedef struct servo{
	servo_axis name_axis;
	TIM_TypeDef* TIM_BASE;
	uint8_t TIM_CHANNEL;
}servo_t;

extern int pos_tilt[4];//rotation axe horizontal
extern int pos_pan[8];//roation axe vertical

void servo_go_to_angle(servo_t* myservo, int angle_deg);
void servo_init(servo_t* servo_pan, servo_t* servo_tilt, TIM_HandleTypeDef* htim_pan, TIM_HandleTypeDef* htim_tilt);

#endif /* INC_SERVO_H_ */
