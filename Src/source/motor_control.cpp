/*
 * motor_control.cpp
 *
 *  Created on: 2019/01/14
 *      Author: shun2
 */

#include "motor_control.hpp"

#include "stm32f4xx_hal.h"
#include <math.h>

#include "peripheral.hpp"
/*
#define MOTOR_PGAIN 0.3
#define MOTOR_IGAIN 0.02
#define MOTOR_DGAIN 0.01*/
#define ANGLE_PGAIN 0.02
#define ANGLE_IGAIN 0.0
#define ANGLE_DGAIN 0

motor_target_t motor_target;

motor_var_t motor_var;

angle_var_t angle_var;

void motor::update_pwm(void){
	__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1, sqrtf(2)/2*(-motor_var.X+motor_var.Y)+110);//1
	__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2, sqrtf(2)/2*( motor_var.X+motor_var.Y)+110);//2
	__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3, sqrtf(2)/2*(-motor_var.X+motor_var.Y)+110);//M3
	__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4, sqrtf(2)/2*( motor_var.X+motor_var.Y)+110);//4
}

void motor::update_target(void){
	/*motor_var.xdiff[0]=motor_target.vx-motor_var.X;
	motor_var.ydiff[0]=motor_target.vy-motor_var.Y;
	motor_var.xsum+=motor_var.xdiff[0];
	motor_var.ysum+=motor_var.ydiff[0];
	motor_var.X+=MOTOR_PGAIN*motor_var.xdiff[0]+MOTOR_IGAIN*motor_var.xsum+MOTOR_DGAIN*(motor_var.xdiff[0]-motor_var.xdiff[1]);
	motor_var.Y+=MOTOR_PGAIN*motor_var.ydiff[0]+MOTOR_IGAIN*motor_var.ysum+MOTOR_DGAIN*(motor_var.ydiff[0]-motor_var.ydiff[1]);
	motor_var.xdiff[1]=motor_var.xdiff[0];
	motor_var.ydiff[1]=motor_var.ydiff[0];*/
	motor_var.X=motor_target.vx;
	motor_var.Y=motor_target.vy;
	return;
}

void motor::move_angle(float angle,int16_t sp){
	angle-=90;
	motor_target.vx=cosf(angle/180*M_PI)*sp;
	motor_target.vy=sinf(angle/180*M_PI)*sp;
	return;
}

void motor::brake(void){
	motor_target.vx=0;
	motor_target.vy=0;
	update_target();
	update_pwm();
	return;
}

void motor::init(void){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	motor::brake();
}
