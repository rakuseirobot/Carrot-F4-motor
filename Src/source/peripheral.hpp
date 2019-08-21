/*
 * peripheral.hpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */

#ifndef SOURCE_PERIPHERAL_HPP_
#define SOURCE_PERIPHERAL_HPP_

#include <stm32f4xx_hal.h>
#include "stdbool.h"
#include "../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os.h"
//#include "uart_control.hpp"

#define LOW_POWER
#define LIPO_WARNING_VOLTAGE 11
#define LOGIC_WARNING_VOLTAGE 11

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
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_uart8_rx;


typedef StaticTask_t osStaticThreadDef_t;

extern osThreadId_t defaultTaskHandle;
extern uint32_t defaultTaskBuffer[ 128 ];
extern osStaticThreadDef_t defaultTaskControlBlock;
extern osThreadId_t adc1Handle;
extern uint32_t adc1Buffer[ 1024 ];
extern osStaticThreadDef_t adc1ControlBlock;
extern osThreadId_t motorHandle;
extern uint32_t motorBuffer[ 1024 ];
extern osStaticThreadDef_t motorControlBlock;
extern osThreadId_t EMERGENCYHandle;
extern uint32_t EMERGENCYBuffer[ 1024 ];
extern osStaticThreadDef_t EMERGENCYControlBlock;

extern uint16_t rxBuff[4];

extern bool LiPo_warning;
extern float LiPo_boltage;
extern bool logic_warning;
extern float Logic_boltage;

extern bool EMERGENCY;

extern int32_t X_count,Y_count;

#endif /* SOURCE_PERIPHERAL_HPP_ */
