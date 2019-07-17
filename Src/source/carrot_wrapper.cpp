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
	HAL_GPIO_WritePin(FET8_GPIO_Port,FET8_Pin,GPIO_PIN_SET);
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
	motor::move_angle(0, 30);
	motor::update_target();
	return;
}
