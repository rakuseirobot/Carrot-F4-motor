/*
 * peripheral.hpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */

#ifndef SOURCE_PERIPHERAL_HPP_
#define SOURCE_PERIPHERAL_HPP_

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>
#include "uart_control.hpp"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern CAN_HandleTypeDef hcan2;

extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_uart8_rx;

extern osThreadId_t defaultTaskHandle;

extern uart serial;


#endif /* SOURCE_PERIPHERAL_HPP_ */
