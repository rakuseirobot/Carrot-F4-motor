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

int16_t bufferRx;
int16_t rxData_buffer[2];
uint8_t rx_flag=0;

float motor_receive_data[5];

bool MOTOR_BRAKE = false;

void raspi_uart_func(){
	if(bufferRx==255&&rx_flag==0){
		rx_flag=1;
	}
	else if(rx_flag==1){
		rxData_buffer[0]=bufferRx;
		rx_flag=2;
	}
	else if(rx_flag==2){
		rxData_buffer[1]=bufferRx;
		rx_flag=3;
	}
	else if(rx_flag==3){
		if(rxData_buffer[0]==1){
			motor_receive_data[1]=(float)((float)((int32_t)(rxData_buffer[1]<<8)|bufferRx)-10000)/10000;
		}
		else if(rxData_buffer[0]==2){
			motor_receive_data[2]=(float)((float)((int32_t)(rxData_buffer[1]<<8)|bufferRx)-10000)/10000;
		}
		else if(rxData_buffer[0]==3){
			motor_receive_data[3]=(float)((float)((int32_t)(rxData_buffer[1]<<8)|bufferRx)-10000)/10000;
		}
		else{
			motor_receive_data[rxData_buffer[0]]=(rxData_buffer[1]<<8)|bufferRx;
		}
		if(rxData_buffer[0]==5){
			if((((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01)==1){
				MOTOR_BRAKE=true;
				motor::brake();
			}
			if((((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01)==0){
				MOTOR_BRAKE=false;
			}

		}
		rx_flag=0;
		rxData_buffer[0]=0;
		rxData_buffer[1]=0;
	}
	HAL_UART_Receive_IT(&huart8, (uint8_t*) &bufferRx,1);
	//motor::update_target();
	//motor::update_pwm();
}

void motor::update_pwm(void){
	int16_t turn_fix = motor_receive_data[motor_turnspeed_data];
	if(MOTOR_BRAKE==false){
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1, sqrtf(2)/2*((motor_var.X-motor_var.Y)-turn_fix)+110);//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2, sqrtf(2)/2*((motor_var.X+motor_var.Y)-turn_fix)+110);//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3, sqrtf(2)/2*((motor_var.X-motor_var.Y)+turn_fix)+110);//M3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4, sqrtf(2)/2*((motor_var.X+motor_var.Y)+turn_fix)+110);//4
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1,110);//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2,+110);//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3,+110);//M3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4,+110);//4
	}
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
	//motor::move_angle(motor_receive_data[motor_angle_data],motor_receive_data[motor_speed_data]);
	/*motor_var.X=motor_target.vx;
	motor_var.Y=motor_target.vy;*/
	motor_var.X=motor_receive_data[motor_x_data];
	motor_var.Y=motor_receive_data[motor_y_data];
	return;
}

void motor::move_angle(float angle,int16_t sp){
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


void motor_task(void *argument){
	while(1){
		serial.string("[Motor_Status] X:");
		serial.putfloat(motor_receive_data[motor_x_data]);
		serial.string(",Y:");
		serial.putfloat(motor_receive_data[motor_y_data]);
		serial.string(",Turn:");
		serial.putfloat(motor_receive_data[motor_turnspeed_data]);
		serial.string(",Brake:");
		serial.putint(((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01);
		serial.string(",LiPo:");
		serial.putfloat(LiPo_boltage);
		serial.string("V\n\r");
		osDelay(100);
	}
}
