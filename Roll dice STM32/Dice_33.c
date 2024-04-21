#include "stm32f10x.h"		//device
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "delay.h"

void Clock_init(void);
void EXTI_init(void);
void TIM2_3_init(void);
void EXTI0_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void On_Off_init(void);
void Roll(void);

uint8_t state = 0;
uint8_t onoff = 0;
uint8_t rolling = 0;
uint16_t Dice[6] = {0x10,0x44,0x38,0xAA,0xBA,0xEF};
uint16_t a;

int main(void)
{
	SysTick_Config(9000);// This one MUST be done first
	GPIOA->CRL = 0x22222224; //PA1 -> PA7 is output
	Clock_init();
	EXTI_init();
	while(1)
	{
		if(onoff==1)
		{
			On_Off_init();
			onoff=0;
		}
		if(rolling==1)
		{
			Roll();
			rolling = 0;
		}
	}
}

void Clock_init(void)
{
	//Enable Clock Port A, Port B and AFIO
	RCC->APB2ENR |= (1<<3)|(1<<2)|(1<<0);
}

void EXTI_init(void)
{
	//Configure EXTI0
	AFIO->EXTICR[0]|=0x0001; //Configure EXITCR1 (EXTICR[0]) for PB0
	AFIO->EXTICR[1]|=0x0010; //Configure EXITCR1 (EXTICR[1]) for PB5
	
	EXTI->IMR |= (1<<0); //Enable EXTI0 EXTI1 EXTI2 for PB0 PB5
	EXTI->FTSR |= (1<<5)|(1<<0); //Detect falling trigger 1->0 for PB0 PB5
	
	__enable_irq();
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI0_IRQHandler(void)
{
	EXTI->PR |=(1<<0); //Clear interrupt flag EXTI0
	onoff = 1;
}
void EXTI9_5_IRQHandler(void)
{
	EXTI->PR |=(1<<5); //Clear interrupt flag EXTI5
	rolling = 1;
}
void On_Off_init(void)
{
	if(state==0)
	{
		GPIOA->ODR = 0x00fe;
		delay(1000);
		GPIOA->ODR = 0;
		EXTI->IMR |= (1<<5);
	}
	if(state!=0)
	{
			GPIOA->ODR = 0;
			EXTI->IMR &= ~(1<<5);
	}
	state=~state;
}
void Roll(void)
{
	a = rand()%6 + 1;
	if(a==6)
	{
		for(int i =0;i<6;i++)
		{
		GPIOA->ODR = Dice[a-1];
		delay(333);
		GPIOA->ODR = 0;
		}
	}
	else
	{
		GPIOA->ODR = Dice[a-1];
		delay(1000);
		GPIOA->ODR = 0;
	}
}