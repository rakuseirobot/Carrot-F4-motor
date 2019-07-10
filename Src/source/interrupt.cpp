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

void _HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	led_control_int_func(htim);
}
