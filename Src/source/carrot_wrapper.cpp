/*
 * carrot_wrapper.cpp
 *
 *  Created on: Jul 6, 2019
 *      Author: shun2
 */
#include "carrot_wrapper.hpp"
#include "carrot_adc_control.hpp"
#include "uart_control.hpp"
#include "peripheral.hpp"

uart serial(&huart1);

void cpploop(void){

}
void init_carrot(void){
	serial.string("wake up!\n\r");
	init_adc();
	return;
}
