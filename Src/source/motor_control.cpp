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

#define MOTOR_PGAIN 0.3
#define MOTOR_IGAIN 0.02
#define MOTOR_DGAIN 0.01
#define ANGLE_PGAIN 0.02
#define ANGLE_IGAIN 0.0
#define ANGLE_DGAIN 0

#define MAX_SPEED 2 //ROS側と合わせる

extern uart serial;

motor_speed_t target_speed;
motor_speed_t current_speed;
motor_speed_t control_speed;

motor_target_t motor_target;

uint16_t rxBuff[4];
int32_t X_count,Y_count;
uint32_t UART_LOSS=0;

float MOTOR_SPEED_GAIN=1.0;

float motor_receive_data[6];

bool MOTOR_BRAKE = false;
bool EMERGENCY = false;

void raspi_uart_func(){
	if(rxBuff[0]!=0xFF){
		HAL_UART_Receive_DMA(&huart8, (uint8_t*)rxBuff, 4);
		UART_LOSS++;
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

uint16_t pwm_calculater(uint16_t value){//本来pwmは10~210
	if(value == 110){
		return 110;
	}
	if(value<20){
		return 20;
	}
	else if(value>200){
		return 200;
	}
	if(120>value&&value>110){
		return 120;
	}
	else if(110>value&&value>100){
		return 100;
	}
	else{
		return value;
	}
}

void motor::update_pwm(void){
	int16_t turn_fix = target_speed.theta*40;
	if(MOTOR_BRAKE==false&&EMERGENCY==false){
		uint16_t one= -1*sqrtf(2)/2*((target_speed.vx-target_speed.vy)/MAX_SPEED-turn_fix)*MOTOR_SPEED_GAIN+110;
		uint16_t two= -1*sqrtf(2)/2*((target_speed.vx+target_speed.vy)/MAX_SPEED-turn_fix)*MOTOR_SPEED_GAIN+110;
		uint16_t three= -1*sqrtf(2)/2*((target_speed.vx+target_speed.vy)/MAX_SPEED+turn_fix)*MOTOR_SPEED_GAIN+110;
		uint16_t four= -1*sqrtf(2)/2*((target_speed.vx-target_speed.vy)/MAX_SPEED+turn_fix)*MOTOR_SPEED_GAIN+110;

		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1,pwm_calculater(one));//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2,pwm_calculater(two));//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3,pwm_calculater(three));//3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4,pwm_calculater(four));//4
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_1,110);//1
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_2,110);//2
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_3,110);//3
		__HAL_TIM_SET_COMPARE(&htim1, MOTORCH_4,110);//4
	}
}

void motor::update_target(void){
	#ifdef SPEED_CONTROL
	target_speed.vx=motor_receive_data[motor_y_data]*100*motor_receive_data[5];
	target_speed.vy=motor_receive_data[motor_x_data]*100*motor_receive_data[5];
	target_speed.theta=motor_receive_data[motor_turnspeed_data]*100*motor_receive_data[5];
	return;
	#else
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
	#endif
	
}

void motor::move_angle(float angle,float sp){
	target_speed.vx=cosf(angle/180*M_PI)*sp;
	target_speed.vy=sinf(angle/180*M_PI)*sp;
	return;
}

void motor::brake(void){
	target_speed.vx=0;
	target_speed.vy=0;
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

void update_encoder(void){//オムニ直径45mm エンコーダー一回転400
	int32_t XC=Get_Encoder(X_axis),YC=Get_Encoder(Y_axis);
	X_count-=XC;
	Y_count+=YC;
	X_Encoder_COUNT=32767;
	Y_Encoder_COUNT=32767;
	current_speed.vx=(45*M_PI*XC/400)*100/1000;
	current_speed.vy=(45*M_PI*YC/400)*100/1000;
}


void feedback_speed(void){//速度P制御
	
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
		#ifdef SPEED_CONTROL
		feedback_speed();
		#endif
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
			serial.putfloat(target_speed.vx);
			serial.string("m/s,Y:");
			serial.putfloat(target_speed.vy);
			serial.string("m/s,Turn:");
			serial.putfloat(target_speed.theta);
			serial.string("°/s,Brake:");
			serial.putint(((int16_t)motor_receive_data[motor_bb_data]>>9)&0x01);


			serial.string(",");
			serial.string(MOTOR_BATTERY_TYPE[MOTOR_BATTERY_TYPE_NUM]);
			serial.string(":");
			serial.putfloat(LiPo_boltage);
			serial.string("V,Logic:");
			serial.putfloat(Logic_boltage);
			serial.string("V\n\r");
			serial.string("[Encoder_Status] X:");
			serial.putint(X_count);
			serial.string(",Y:");
			serial.putint(Y_count);

			serial.string("\n\r[Speed_Status] X:");
			serial.putfloat(current_speed.vx);
			serial.string("m/s ,Y:");
			serial.putfloat(current_speed.vy);
			serial.string("m/s\n\r");
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
			serial.string(MOTOR_BATTERY_TYPE[MOTOR_BATTERY_TYPE_NUM]);
			serial.string(":");
			serial.putfloat(LiPo_boltage);
			serial.string("V\e[0m\n\r");
		}
		if(EMERGENCY==false&&((motor_receive_data[motor_x_data]==0 && motor_receive_data[motor_y_data]==0)&&motor_receive_data[motor_turnspeed_data]==0)){
			HAL_GPIO_WritePin(FET_RED_GPIO_Port, FET_RED_Pin, GPIO_PIN_SET);
			/*if(osThreadGetState(ledHandle)==osThreadRunning){
				osThreadSuspend(ledHandle);
			}
			HAL_GPIO_WritePin(FET_LEFT_GPIO_Port, FET_LEFT_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FET_RIGHT_GPIO_Port, FET_RIGHT_Pin, GPIO_PIN_RESET);*/
		}
		else if(EMERGENCY==false&&(motor_receive_data[motor_x_data]!=0 || motor_receive_data[motor_y_data]!=0)){
			HAL_GPIO_WritePin(FET_RED_GPIO_Port, FET_RED_Pin, GPIO_PIN_RESET);
			/*if(osThreadGetState(ledHandle)==osThreadBlocked){
				osThreadResume(ledHandle);
			}*/
		}
		osDelay(100);
	}
}
