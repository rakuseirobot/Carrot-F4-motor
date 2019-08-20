/*
 * carrot_wrapper.cpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */
#include "carrot_wrapper.hpp"
#include "carrot_adc_control.hpp"
#include "uart_control.hpp"
#include "peripheral.hpp"
#include "led_control.hpp"
#include "main.h"
#include "motor_control.hpp"

int16_t ff;


uart serial(&huart1);
neopixel front(FET4_GPIO_Port,FET4_Pin,50);

void cpploop(void){

}


void init_carrot(void){
	serial.string("wake up!\n\r");

	#ifdef LOW_POWER
	HAL_GPIO_WritePin(FET_BAR_GPIO_Port, FET_BAR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FET_RIGHT_GPIO_Port, FET_RIGHT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FET_LEFT_GPIO_Port, FET_LEFT_Pin, GPIO_PIN_RESET);
	#else
	HAL_GPIO_WritePin(FET_BAR_GPIO_Port,FET_BAR_Pin,GPIO_PIN_SET);
	#endif

	init_adc();

	HAL_GPIO_WritePin(BUZZER1_GPIO_Port,BUZZER1_Pin,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
	//front.set_all_color(0xFF, 0, 0);
	//front.update();
	motor::init();
	HAL_UART_Receive_IT(&huart8, (uint8_t*) &bufferRx,1);
	start_encoder();
	X_Encoder_COUNT=32767;
	Y_Encoder_COUNT=32767;
	if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_RESET){ //Push Button
			EMERGENCY=true;
			motor::brake();
	}
	return;
}
