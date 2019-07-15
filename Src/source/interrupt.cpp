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


extern neopixel front;
extern int16_t ff;

void _HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim10){
			front.int_act();
	}
}


