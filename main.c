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
#include "NVIC.h"

void delay(uint16 delay);
void turnLEDsOff();
void blueLEDOn();
void redLEDOn();
void greenLEDOn();
void yellowColor();
void purpleColor();
void whiteColor();

typedef struct
{
	uint32 (*fptrBefore);
	uint16 delay;
	uint32 (*fptrAfter);
	uint32 (*fptrWhite);
}StateType;

const StateType FineStateMachineMoore[5]=
		{
			{&yellowColor,650000,&blueLEDOn,&whiteColor},
			{&greenLEDOn,650000,&purpleColor,&whiteColor},
			{&blueLEDOn,650000,&redLEDOn,&whiteColor},
			{&purpleColor,650000,&yellowColor,&whiteColor},
			{&redLEDOn,650000,&greenLEDOn,&whiteColor}
		};

typedef enum{
	BEFORE,
	AFTER,
	WHITES
}NextState;

int main(void) {

		/**Activating the clock gating of the GPIOs and the PIT*/
				GPIO_clockGating(GPIO_A);
				GPIO_clockGating(GPIO_B);
				GPIO_clockGating(GPIO_C);
				GPIO_clockGating(GPIO_E);

				/**Selected configurations*/
				GPIO_pinControlRegisterType pinControlRegisterMux1 = GPIO_MUX1;
				GPIO_pinControlRegisterType pinControlRegisterInputInterruptPSFE = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE;

				/**Configure the characteristics in the GPIOs*/
				GPIO_pinControlRegister(GPIO_B,BIT21,&pinControlRegisterMux1);
				GPIO_pinControlRegister(GPIO_B,BIT22,&pinControlRegisterMux1);
				GPIO_pinControlRegister(GPIO_E,BIT26,&pinControlRegisterMux1);
				GPIO_pinControlRegister(GPIO_A,BIT4,&pinControlRegisterInputInterruptPSFE);
				GPIO_pinControlRegister(GPIO_C,BIT6,&pinControlRegisterInputInterruptPSFE);

				/**Assigns a safe value to the output pin21 of the GPIOB*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
				GPIOB->PDOR |= 0x00400000;/**Read led off*/

				/**Configure Port Pins as input/output*/
				GPIO_dataDirectionPIN(GPIO_B,GPIO_OUTPUT,BIT21);
				GPIO_dataDirectionPIN(GPIO_B,GPIO_OUTPUT,BIT22);
				GPIO_dataDirectionPIN(GPIO_E,GPIO_OUTPUT,BIT26);
				GPIO_dataDirectionPIN(GPIO_C,GPIO_INPUT,BIT6);
				GPIO_dataDirectionPIN(GPIO_A,GPIO_INPUT,BIT4);

				/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
				NVIC_setBASEPRI_threshold(PRIORITY_5);
				/**Enables and sets a particular interrupt and its priority*/
				NVIC_enableInterruptAndPriotity(PORTA_IRQ,PRIORITY_4);
				/**Enables and sets a particular interrupt and its priority*/
				NVIC_enableInterruptAndPriotity(PORTC_IRQ,PRIORITY_4);

				EnableInterrupts;

				delay(3000);

		uint32 output=0,inputPortA=0, inputPortC=0, totalInput = 0;
		uint8 currentState = 0;

    while(1) {

    	if(TRUE == GPIO_getIRQStatus(GPIO_A) || TRUE == GPIO_getIRQStatus(GPIO_C)){

    		inputPortA = GPIOA->PDIR;
			inputPortA &=(0x10);
			inputPortC = GPIOC->PDIR;
			inputPortC &=(0x40);
			inputPortC = inputPortC >> 6;
			inputPortA = inputPortA >> 3;
			totalInput = inputPortA|inputPortC;

			if(BEFORE == totalInput){
				FineStateMachineMoore[currentState].fptrBefore;
				uint16 delayer = FineStateMachineMoore[currentState].delay;
				delay(delayer);
			}
			else if(AFTER == totalInput){
				FineStateMachineMoore[currentState].fptrAfter;
				uint16 delayer = FineStateMachineMoore[currentState].delay;
				delay(delayer);
			}
			else if(WHITES == totalInput){
				FineStateMachineMoore[currentState].fptrWhite;
				uint16 delayer = FineStateMachineMoore[currentState].delay;
				currentState = 0;//return to green state
				delay(delayer);
			}

			GPIO_clearIRQStatus(GPIO_A);
			GPIO_clearIRQStatus(GPIO_C);
    	}
    }
    return 0 ;
}

void delay(uint16 delay)
{
	volatile uint16 counter;

	for(counter=delay; counter > 0; counter--)
	{
	}
}

void turnLEDsOff(){
			GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			delay(1000);//65000
			GPIOB->PDOR |= 0x00400000;/**Read led off*/
			delay(1000);
			GPIOE->PDOR |= 0x4000000;/**Green led off*/
			delay(1000);
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
