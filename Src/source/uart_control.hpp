/*
 * uart_control.hpp
 *
 *  Created on: 2019/01/15
 *      Author: shun2
 */

#ifndef SOURCE_UART_CONTROL_HPP_
#define SOURCE_UART_CONTROL_HPP_

#include "stm32f4xx_hal.h"

struct uart{
	UART_HandleTypeDef *use_uart;
	uart(UART_HandleTypeDef *huart);
	uint8_t get(void);
	void send(uint8_t c);
	void string(const char *s);
	void putfloat(float data);
	void putint(int64_t data);
	void putnum(uint8_t n);
};


#endif /* SOURCE_UART_CONTROL_HPP_ */
