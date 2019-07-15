/*
 * led_control.hpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */

#ifndef SOURCE_LED_CONTROL_HPP_
#define SOURCE_LED_CONTROL_HPP_

#include <stm32f4xx_hal.h>
#include "peripheral.hpp"
#include <queue>

struct neopixel{
	GPIO_TypeDef* gpio;uint16_t pin;uint16_t led_num;
	std::queue<uint8_t> data;
	uint8_t flag_byte=0,flag_bits=0;
	uint8_t led_data[0xFF][3];
	neopixel(GPIO_TypeDef* mgpio, uint16_t mpin, uint16_t num);
	void int_act(void);
	void send_bit(uint8_t bit);
	void set_all_color(uint8_t red,uint8_t green,uint8_t blue);
	void update(void);
};



#endif /* SOURCE_LED_CONTROL_HPP_ */
