/*
 * led_control.cpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */
#include <queue>

#include "main.h"
#include "led_control.hpp"

neopixel::neopixel(GPIO_TypeDef* mgpio, uint16_t mpin, uint16_t num){
	gpio = mgpio;
	pin = mpin;
	led_num = num;
	for (uint8_t i=0; i<3; i++){
		data.push(0x00); //RESET
	}
	for (uint8_t i=0; i<num*3; i++){
		data.push(0x00); //RGB all Zero
	}
	return;
}
void neopixel::send_bit(uint8_t bit){
	if(flag_bits<=8){
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);//HIGH
	}
	else if(bit==1 && flag_bits<=16){
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);//HIGH
	}
	else{
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);//LOW
	}
	if(flag_bits>=25){
		flag_bits=1;
		if(flag_byte>=8){
			flag_byte=1;
			data.pop();//front data delete
		}
		else{
			flag_byte++;
		}
	}
	else{
		flag_bits++;
	}
	return;
}
void neopixel::int_act(void){
	if(data.empty()==true){
		HAL_TIM_Base_Start_IT(&htim13);//Finish Timer Stop
	}
	switch(flag_byte){
	case 1:
		send_bit((data.front()&0b10000000)>>7);
		break;
	case 2:
		send_bit((data.front()&0b01000000)>>6);
		break;
	case 3:
		send_bit((data.front()&0b00100000)>>5);
		break;
	case 4:
		send_bit((data.front()&0b00010000)>>4);
		break;
	case 5:
		send_bit((data.front()&0b00001000)>>3);
		break;
	case 6:
		send_bit((data.front()&0b00000100)>>2);
		break;
	case 7:
		send_bit((data.front()&0b00000010)>>1);
		break;
	case 8:
		send_bit(data.front()&0b00000001);
		break;
	default:
		break;
	}
	return;
}

void neopixel::set_all_color(uint8_t red,uint8_t green,uint8_t blue){
	while(!data.empty()){
		data.pop();
	}
	for (uint8_t i=0; i<3; i++){
		data.push(0x00); //RESET
	}
	for(uint16_t i=0;i<led_num;i++){
		data.push(green);
		data.push(red);
		data.push(blue);
	}
	return;
}

void neopixel::update(void){
	if(data.empty()==true){
		return;
	}
	while(!(data.size()==(led_num*3)+3)){
		data.push(0x00);
	}
	//FreeRTOS Task suspend
	HAL_TIM_Base_Start_IT(&htim13);//Timer Start
}

neopixel front(FET4_GPIO_Port,FET4_Pin,50);


void led_control_int_func(TIM_HandleTypeDef *htim){
	if (htim == &htim13){
		front.int_act();
	}
}


