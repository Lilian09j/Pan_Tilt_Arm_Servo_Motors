/*
 * servo.c
 *
 *  Created on: Nov 6, 2024
 *      Author: lilian.jaouanne
 */


/* Includes ------------------------------------------------------------------*/
#include "servo.h"

const float add_high_time_1deg = 0.460/45;//0.01022222 ms

//be aware, must be int and not uint8_t because there is negative values
//int pos_tilt[6] = {0, 10, 20, 30, 40, 50};//rotation axe horizontal
//int pos_pan[6] = {0, 45, 90, 0, -45, -90};//roation axe vertical
int pos_tilt[4] = {0, 15, 30, 45};//rotation axe horizontal
int pos_pan[8] = {0, 45, 90, 45, 0, -45, -90, -45};//roation axe vertical

//global variables to store the current position actualised after each write(angle)
int current_pos_pan_servo = 0;
int current_pos_tilt_servo = 0;

void servo_init(servo_t* servo_pan, servo_t* servo_tilt, TIM_HandleTypeDef* htim_pan, TIM_HandleTypeDef* htim_tilt){
	uint32_t ARR = 0;
	float high_time_ms = 0.0, CCR_float = 0.0, duty_cycle = 0.0;

	servo_pan->name_axis = pan;
	servo_pan->TIM_BASE = (TIM_TypeDef *)TIM1_BASE;
	servo_pan->TIM_CHANNEL = TIM_CHANNEL_1;
	//servo_pan.TIM_BASE->ARR;

	servo_tilt->name_axis = tilt;
	servo_tilt->TIM_BASE = (TIM_TypeDef *)TIM1_BASE;
	servo_tilt->TIM_CHANNEL = TIM_CHANNEL_2;

	//go to initial position
	ARR = servo_pan->TIM_BASE->ARR +1;
	high_time_ms = PWM_HIGH_TIME_0DEG + POS_INIT_PAN*add_high_time_1deg;
	duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	CCR_float = ARR*duty_cycle/100;
	servo_pan->TIM_BASE->CCR1 = (uint32_t)CCR_float;
	current_pos_pan_servo = POS_INIT_PAN;

	//go to initial position
	ARR = servo_tilt->TIM_BASE->ARR +1;
	high_time_ms = PWM_HIGH_TIME_0DEG + POS_INIT_TILT*add_high_time_1deg;
	duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	CCR_float = ARR*duty_cycle/100;
	servo_tilt->TIM_BASE->CCR2 = (uint32_t)CCR_float;
	current_pos_tilt_servo = POS_INIT_TILT;

	HAL_TIM_PWM_Start(htim_pan, TIM_CHANNEL_1);//PA5 servo PAN axes
	HAL_TIM_PWM_Start(htim_tilt, TIM_CHANNEL_2);//PA1 servo TILT axis

//	HAL_Delay(2000);
}

void servo_go_to_angle(servo_t* myservo, int angle_deg){
	int actual_pos = 0;
	uint32_t ARR = myservo->TIM_BASE->ARR +1;
	float high_time_ms = 0.0, CCR_float = 0.0, duty_cycle = 0.0;

	if(myservo->name_axis == tilt){
		angle_deg += POS_INIT_TILT;
		actual_pos = current_pos_tilt_servo;

	    if(actual_pos <= angle_deg){//incrementation to reduce the speed rotation
	      for(int i = actual_pos+1; i <= angle_deg; i++){
	          high_time_ms = PWM_HIGH_TIME_0DEG + i*add_high_time_1deg;
	          duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	          CCR_float = ARR*duty_cycle/100;
	          myservo->TIM_BASE->CCR2 = (uint32_t)CCR_float;
	          current_pos_tilt_servo = i;
	          HAL_Delay(4);
	      }
	    }
	    else if(actual_pos > angle_deg){
	      for(int i = actual_pos-1; i >= angle_deg; i--){
	    	  high_time_ms = PWM_HIGH_TIME_0DEG + i*add_high_time_1deg;
	    	  duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	    	  CCR_float = ARR*duty_cycle/100;
	    	  myservo->TIM_BASE->CCR2 = (uint32_t)CCR_float;
	    	  current_pos_tilt_servo = i;
	    	  HAL_Delay(4);
	      }
	    }
	  }

	  else{
	    angle_deg += POS_INIT_PAN;
	    actual_pos = current_pos_pan_servo;

	    if(actual_pos <= angle_deg){
	      for(int i = actual_pos+1; i <= angle_deg; i++){
	    	  high_time_ms = PWM_HIGH_TIME_0DEG + i*add_high_time_1deg;
	    	  duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	    	  CCR_float = ARR*duty_cycle/100;
	    	  myservo->TIM_BASE->CCR1 = (uint32_t)CCR_float;
	    	  current_pos_pan_servo = i;
	    	  HAL_Delay(4);
	      }
	    }
	    else if(actual_pos > angle_deg){
	      for(int i = actual_pos-1; i >= angle_deg; i--){
	    	  high_time_ms = PWM_HIGH_TIME_0DEG + i*add_high_time_1deg;
	    	  duty_cycle = 100*high_time_ms/PWM_PERIOD;//pwm freq 50Hz -> period 20ms
	    	  CCR_float = ARR*duty_cycle/100;
	    	  myservo->TIM_BASE->CCR1 = (uint32_t)CCR_float;
	    	  current_pos_pan_servo = i;
	    	  HAL_Delay(4);
	      }
	    }
	  }
}
