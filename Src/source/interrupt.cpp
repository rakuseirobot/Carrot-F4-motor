/*
 * interrupt.cpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */
#include <stm32f4xx_hal.h>
#include "cmsis_os.h"
#include "peripheral.hpp"
#include "led_control.hpp"
#include "interrupt.hpp"
#include "motor_control.hpp"
#include "main.h"


extern neopixel front;
extern int16_t ff;

void EMERGENCY_notification(void *argument){
	while(1){
		if(EMERGENCY==false){
			serial.string("false!!\n\r");
			osDelay(100);
			//osThreadSuspend(EMERGENCYHandle);
		}
		else{
			serial.string("notify!!\n\r");
			HAL_GPIO_WritePin(FET8_GPIO_Port, FET8_Pin, GPIO_PIN_SET);//LED
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,5);//buzzer
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,5);//buzzer
			osDelay(300);
			HAL_GPIO_WritePin(FET8_GPIO_Port, FET8_Pin, GPIO_PIN_RESET);//LED
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);//buzzer
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);//buzzer
			osDelay(300);
		}
	}
}

void _HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim10){
			front.int_act();
	}
}

void _EXTI0_IRQHandler(void){
	if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_RESET){ //Push Button
		osThreadResume(EMERGENCYHandle);
		EMERGENCY=true;
		motor::brake();
	}
	else if(HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_SET){ //Reset Button
		EMERGENCY=false;
		osThreadSuspend(EMERGENCYHandle);
	}
}
