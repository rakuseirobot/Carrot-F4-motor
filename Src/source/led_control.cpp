/*
 * led_control.cpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */
#include <queue>

#include "main.h"
#include "led_control.hpp"
#include "peripheral.hpp"
extern int16_t ff;

neopixel::neopixel(GPIO_TypeDef* mgpio, uint16_t mpin, uint16_t num){
	gpio = mgpio;
	pin = mpin;
	led_num = num;
	for (uint8_t i=0; i<num; i++){
		data.push(0x00); //RGB all Zero
		data.push(0x00); //RGB all Zero
		data.push(0x00); //RGB all Zero
		led_data[i][0]=0x00;
		led_data[i][1]=0x00;
		led_data[i][2]=0x00;
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
	ff=ff+1;
	if(data.empty()==true){
		flag_byte=0;flag_bits=0;
		HAL_TIM_Base_Stop_IT(&htim10);//Finish Timer Stop
		return;
	}
	switch(flag_byte){
	case 0:
		if(flag_bits>=10){
			flag_byte++;
			flag_bits=1;
		}
		else{
			flag_bits++;
		}
		HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
		break;
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
	HAL_TIM_Base_Start_IT(&htim10);
	return;
}

void neopixel::set_all_color(uint8_t red,uint8_t green,uint8_t blue){
	for(uint16_t i=0;i<led_num;i++){
		led_data[i][0]=green;
		led_data[i][1]=red;
		led_data[i][2]=blue;
	}
	return;
}

void neopixel::update(void){
	while(!data.empty()){
			data.pop();
	}
	for(uint16_t i=0;i<led_num;i++){
		data.push(led_data[i][0]);
		data.push(led_data[i][1]);
		data.push(led_data[i][2]);
	}
	//FreeRTOS Task suspend
	HAL_TIM_Base_Start_IT(&htim10);//Timer Start
}



