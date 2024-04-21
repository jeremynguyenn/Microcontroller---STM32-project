#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "delay.h"

uint8_t buttonState = 0;
uint8_t running = 0;
uint8_t leds = 0;
uint8_t randomNumbe;
uint8_t onoff = 0;


void clock_Init(void){
	//Enable clock for TIM2, Port A, Port B, AFIO
	RCC -> APB2ENR	|= (1<<0) | (1<<2) | (1<<3);
	RCC -> APB1ENR |= (1<<0);
	
	//Enable clock for TIM3
	RCC -> APB1ENR |= (1<<1);
	
	
	//Set APB1 prescaler
	RCC -> CFGR  |= (4 << 8); //APB1 prescaler = 1/8
		
	//Set TIM2 prescaler
	TIM2 -> PSC = 17; //timer prescaler = 1/18
	
	//Set TIM3 prescaler
	TIM3 -> PSC = 17;
	
}

void On_Off_init(void){ 
	// your code
	if(buttonState==0)
	{
		GPIOA->ODR = 0x00fe;
		delay(1000);
		GPIOA->ODR = 0;
		EXTI->IMR |= (1<<3);
	}
if(buttonState !=0)
	{
		GPIOA->ODR = 0;
		EXTI->IMR &= ~(1<<3);
	}
		buttonState =~buttonState;
}

void EXTI0_IRQHandler(void)
{
	EXTI->PR |=(1<<0); //Clear interrupt flag EXTI0
	onoff = 1;
}

/* void EXTI1_IRQHandler(void){
	EXTI->PR |= (1<<3);  // clear interrupt flag
	
	while(1){
		if (buttonState != 0)
			{
				randomNumbe = rand() %6 ;
				for (int i = 0; i < 7; i++){
					if (randomNumbe & (1<< i)){
						GPIOA -> ODR &= ~(1<<i);
					}else{
						GPIOA -> ODR |= (1<<i);
						
					}
				}
			}
		}
} */


//for (int i=0; i <= 6; i++);
//GPIOA->ODR &= ~(1<<i); //turn led on 
//delay(250);
//GPIOA -> ODR |= (1<<i); //turn led off


int main(void){
	//Enable clock, Port A, Port B, AFIO
	clock_Init();
	
	//GPIO setup
	GPIOA -> CRL = 0x22222224; //PA1~7 output
	GPIOB -> CRH = 0x44442442;  //PB0-PB3 input button // turn off all led
	
	//setup TIM2
	//Timer2_Init();
	
	//SETUP EXTI for PB0 and PB3
	
	AFIO -> EXTICR[0] = 0x0001;   //Configure EXITCR1 (EXTICR[0]) for PB0
	AFIO -> EXTICR[1] = 0x0011;		// //Configure EXITCR1 (EXTICR[1]) for PB3
	
	EXTI -> IMR |= (1<<0);     // enable interrupt for PB0~3
	EXTI -> FTSR |= (1<<3)|(1<<0);     // detect falling edge
	
	//enable NVIC
	__enable_irq();
	
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	
	//Systick congifiguration
	
	SysTick_Config(9000);
	while (1){
			if(buttonState==1)
		{
			On_Off_init();
			leds=0;
		}
		/*
		if(rolling==1)
		{
			Roll();
			rolling = 0;
		} 
		*/
		}
}