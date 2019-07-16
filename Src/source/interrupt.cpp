/*
 * interrupt.cpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */
#include <stm32f4xx_hal.h>
#include "peripheral.hpp"
#include "led_control.hpp"
#include "interrupt.hpp"
#include "motor_control.hpp"
#include "main.h"


extern neopixel front;
extern int16_t ff;

void _HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim10){
			front.int_act();
	}
}

void _EXTI0_IRQHandler(void){
	if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_SET){ //Push Button
		EMERGENCY=true;
		motor::brake();
	}
	else if(HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_RESET){ //Reset Button
		EMERGENCY=false;
	}
}
