/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    main.c
 * @brief   Application entry point.
 * Daniel Barragán Alvarez and Ávila Chavira Jorge Alejandro
 */


#include <stdio.h>
#include "DataTypeDefinitions.h"
#include "GPIO.h"
#include "MK64F12.h"

void delay(uint16 delay);
void turnLEDsOff();
void blueLEDOn();
void redLEDOn();
void greenLEDOn();
void yellowColor();
void purpleColor();
void whiteColor();

int main(void) {

	/**Variable to capture the input value*/
		uint32 inputValue = 0;
		uint32 inputValue2 = 0;

		/**Activating the GPIOA, GPIOB, GPIOC and GPIOE clock gating*/
		SIM->SCGC5 = 0x2E00;// 0x2C00 SW2 || 0X2E00 SW2&SW3
		/**Pin control configuration of GPIOB pin22 and pin21 as GPIO*/
		PORTB->PCR[21] = 0x00000100;
		PORTB->PCR[22] = 0x00000100;
		/**Pin control configuration of GPIOA pin5 as GPIO with is pull-up resistor enabled*/
		PORTA->PCR[4] = 0x00000103;
		/**Pin control configuration of GPIOC pin6 as GPIO with is pull-up resistor enabled*/
		PORTC->PCR[6] = 0x00000103;
		/**Pin control configuration of GPIOE pin26 as GPIO*/
		PORTE->PCR[26] = 0x00000100;
		/**Assigns a safe value to the output pin21 of the GPIOB*/
		GPIOB->PDOR = 0x00200000;
		/**Assigns a safe value to the output pin22 of the GPIOB*/
		GPIOB->PDOR |= 0x00400000;
		/**Assigns a safe value to the output pin26 of the GPIOE*/
		GPIOE->PDOR |= 0x04000000;
		/**Configures GPIOA pin5 as input*/
		GPIOA->PDDR &=~(0x10);
		/**Configures GPIOC pin6 as input*/
		GPIOC->PDDR &=~(0x40);
		/**Configures GPIOB pin21 as output*/
		GPIOB->PDDR = 0x00200000;
		/**Configures GPIOB pin22 as output*/
		GPIOB->PDDR |= 0x00400000;
		/**Configures GPIOE pin26 as output*/
		GPIOE->PDDR |= 0x04000000;

		uint32 i = 0;

    while(1) {

    }
    return 0 ;
}

void blueLEDOn(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	delay(65000);
}
void redLEDOn(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void greenLEDOn(){
		turnLEDsOff();
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	delay(65000);
}
void yellowColor(){
		turnLEDsOff();
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void purpleColor(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	delay(65000);
}
void whiteColor(){
		turnLEDsOff();
	GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
	GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
	GPIOE->PDOR &= ~(0x4000000);/**Green led on*/
	delay(65000);
}
