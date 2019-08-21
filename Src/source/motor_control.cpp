/*
 * motor_control.cpp
 *
 *  Created on: 2019/01/14
 *      Author: shun2
 */

#include "motor_control.hpp"

#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "uart_control.hpp"

#include "peripheral.hpp"
/*
#define MOTOR_PGAIN 0.3
#define MOTOR_IGAIN 0.02
#define MOTOR_DGAIN 0.01*/
#define ANGLE_PGAIN 0.02
#define ANGLE_IGAIN 0.0
#define ANGLE_DGAIN 0

extern uart serial;

motor_target_t motor_target;

motor_var_t motor_var;

angle_var_t angle_var;
uint16_t rxBuff[4];
int32_t X_count,Y_count;

float motor_receive_data[6];

bool MOTOR_BRAKE = false;
bool EMERGENCY = false;

void raspi_uart_func(){
	if(rxBuff[0]!=0xFF){
	    HAL_UART_Receive_DMA(&huart8, (uint8_t*)rxBuff, 4);
		return;
	}
	if(rxBuff[1]==1){
		motor_receive_data[1]=(float)((float)((int32_t)(rxBuff[2]<<8)|rxBuff[3])-10000)/10000;
	}
	else if(rxBuff[1]==2){
		motor_receive_data[2]=(float)((float)((int32_t)(rxBuff[2]<<8)|rxBuff[3])-10000)/10000;
	}
	else if(rxBuff[1]==3){
		motor_receive_data[3]=(float)((float)((int32_t)(rxBuff[2]<<8)|rxBuff[3])-10000)/10000;
	}
	else if(rxBuff[1]==5){
		motor_receive_data[5]=(float)((float)((int32_t)(rxBuff[2]<<8)|rxBuff[3])-10000)/10000;
	}
	else{
		motor_receive_data[rxBuff[1]]=(rxBuff[2]<<8)|rxBuff[3];
	}
	if(rxBuff[1]==motor_bb_data){
		if((((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01)==1){
			MOTOR_BRAKE=true;
			motor::brake();
		}
		if((((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01)==0){
			MOTOR_BRAKE=false;
		}

	}
	motor::update_target();
	motor::update_pwm();
    HAL_UART_Receive_DMA(&huart8, (uint8_t*)rxBuff, 4);
	return;
}

uint16_t pwm_calculater(uint16_t value){
	if(value<20){
		return 20;
	}
	else if(value>200){
		return 200;
	}
	else{
		return value;
	}
}

void motor::update_pwm(void){
	int16_t turn_fix = motor_receive_data[motor_turnspeed_data]*40;
	if(MOTOR_BRAKE==false&&EMERGENCY==false){
		uint16_t one= -1*sqrtf(2)/2*((motor_var.X-motor_var.Y)-turn_fix)+110;
		uint16_t two= -1*sqrtf(2)/2*((motor_var.X+motor_var.Y)-turn_fix)+110;
		uint16_t three= -1*sqrtf(2)/2*((motor_var.X+motor_var.Y)+turn_fix)+110;
		uint16_t four= -1*sqrtf(2)/2*((motor_var.X-motor_var.Y)+turn_fix)+110;

		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1,pwm_calculater(one));//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2,pwm_calculater(two));//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3,pwm_calculater(three));//M3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4,pwm_calculater(four));//4
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1,110);//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2,110);//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3,110);//M3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4,110);//4
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
	motor_var.X=motor_receive_data[motor_y_data]*100*motor_receive_data[5];
	motor_var.Y=motor_receive_data[motor_x_data]*100*motor_receive_data[5];
	if (motor_var.X>100){
		motor_var.X=100;
	}
	else if (motor_var.X<-100){
		motor_var.X=-100;
	}
	if (motor_var.Y>100){
		motor_var.Y=100;
	}
	else if (motor_var.Y<-100){
		motor_var.Y=-100;
	}
	return;
}

void motor::move_angle(float angle,int16_t sp){
	motor_target.vx=cosf(angle/180*M_PI)*sp;
	motor_target.vy=sinf(angle/180*M_PI)*sp;
	return;
}

void motor::brake(void){
	motor_var.X=0;
	motor_var.Y=0;
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
	motor_receive_data[5]=1;
	motor::brake();
}


void start_encoder(void){
		HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
		X_Encoder_COUNT=32767;
		Y_Encoder_COUNT=32767;
		return;
	}
void stop_encoder(void){
	HAL_TIM_Encoder_Stop_IT(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Stop_IT(&htim3,TIM_CHANNEL_ALL);
	return;
}

void update_encoder(void){
	int32_t XC=Get_Encoder(X_axis),YC=Get_Encoder(Y_axis);
	X_count-=XC;
	Y_count+=YC;
	X_Encoder_COUNT=32767;
	Y_Encoder_COUNT=32767;
}

int32_t Get_Encoder(ch_t x){
	if(x==X_axis){
		if(X_Encoder_COUNT<32767){
			return (32767-X_Encoder_COUNT)*-1;
		}
		else{
			return (X_Encoder_COUNT-32767);
		}
	}
	else if (x==Y_axis){
		if(Y_Encoder_COUNT<32767){
			return (32767-Y_Encoder_COUNT)*-1;
		}
		else{
			return (Y_Encoder_COUNT-32767);
		}

	}
	return 0;
}

void encoder_task(void *argument){
	while(1){
		update_encoder();
		osDelay(10);
	}
}


void motor_task(void *argument){
	while(1){
		if(LiPo_warning==true){//buzzer
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,5);
		}
		if(EMERGENCY==false){
			serial.string("[Motor_Status] X:");
			serial.putfloat(motor_receive_data[motor_x_data]);
			serial.string(",Y:");
			serial.putfloat(motor_receive_data[motor_y_data]);
			serial.string(",Turn:");
			serial.putfloat(motor_receive_data[motor_turnspeed_data]);
			serial.string(",Brake:");
			serial.putint(((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01);
			serial.string(",Motor:");
			serial.putfloat(LiPo_boltage);
			serial.string("V,Logic:");
			serial.putfloat(Logic_boltage);
			serial.string("V\n\r");
			/*serial.string("[Encoder_Status] X:");
			serial.putint(X_count);
			serial.string(",Y:");
			serial.putint(Y_count);
			serial.string("\n\r");*/
			osThreadSuspend(EMERGENCYHandle);
			HAL_GPIO_WritePin(FET_RED_GPIO_Port,FET_RED_Pin,GPIO_PIN_RESET);
		}
		else if(EMERGENCY==true){
			osThreadResume(EMERGENCYHandle);
			#ifdef LOW_POWER
			#else
			HAL_GPIO_TogglePin(FET_RED_GPIO_Port,FET_RED_Pin);
			HAL_GPIO_TogglePin(FET_RIGHT_GPIO_Port,FET_RIGHT_Pin);
			HAL_GPIO_TogglePin(FET_LEFT_GPIO_Port,FET_LEFT_Pin);
			#endif
			serial.string("\e[43m\e[31m [EMERGENCY STOP]");
			serial.string("LiPo:");
			serial.putfloat(LiPo_boltage);
			serial.string("V\e[0m\n\r");
		}
		if(EMERGENCY==false&&((motor_receive_data[motor_x_data]==0 && motor_receive_data[motor_y_data]==0)&&motor_receive_data[motor_turnspeed_data]==0)){
			HAL_GPIO_WritePin(FET_RED_GPIO_Port, FET_RED_Pin, GPIO_PIN_SET);
		}
		else if(EMERGENCY==false&&(motor_receive_data[motor_x_data]!=0 || motor_receive_data[motor_y_data]!=0)){
			HAL_GPIO_WritePin(FET_RED_GPIO_Port, FET_RED_Pin, GPIO_PIN_RESET);
		}
		osDelay(100);
	}
}
