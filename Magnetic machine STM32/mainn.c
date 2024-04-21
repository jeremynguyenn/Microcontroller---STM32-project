//Nguyen Trung Nhan - MSSV21146033
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "lcd.h"

// General
void SysTick_Handler(void);
void delay(uint32_t dlyTicks);
void Update_lcd(void);

// GPIO
void Init_GPIO(void);

// Variables
volatile uint16_t dutyCycle = 0;
volatile uint32_t encoderCount = 0;
volatile uint32_t prevEncoderCount = 0;
volatile float motorSpeed = 0.0;

unsigned long oldtime_1 = 0;
unsigned long oldtime_2 = 0;
unsigned long Delay_1 = 10;
unsigned long Delay_2 = 10;
// Interrupt
void Interrupt_Init(void);
void EXTI1_IRQHandler();
void EXTI2_IRQHandler();
void PWM_Init(void);
void Encoder_Init(void);
void TIM3_IRQHandler(void);

int main()
{
    SysTick_Config(9000);
		//Enable Clock
    Init_GPIO();
		//Set up interupt
    Interrupt_Init();
		//set up timer 2
		PWM_Init();
		//set up timer 3
		Encoder_Init();
		
		// Initialize lcd
		lcd_init();
		lcd_string("Off");
    while (1)
    {
			  Update_lcd();
    }
}


void Init_GPIO(void)
{
    //GPIOA, GPIOB va AFIO
    RCC->APB2ENR |= (1<<0)|(1<<2)|(1<<3);
		// Bat tin hieu clock TIME2 cho peripheral nam tren APB1
		RCC->APB1ENR |= (1 << 0);
		RCC->APB1ENR |= (3<<0);
		// Thiet lap bo chia tan so cho APB1
		RCC->CFGR |= (6 << 8); // Bo chia tan so APB1 = 1/8
		// Cau hinh chan dieu khien L298N
		GPIOA->CRL &= ~((0 << 4) | (0 << 8) | (0 << 12));  
		GPIOA->CRL |= (1 << 4) | (3 << 8) | (3 << 12);     
    // Cau hinh chan GPIOB tu 8 den 15
    GPIOB->CRH = 0x22222222;
		// Cau hinh chan PB1 va PB2 la input voi pull-up/pull-down
    GPIOB->CRL = 0x44444224;
    // Dat muc logic cao cho chan PB1 va PB2
    GPIOB->ODR |= (1 << 1) | (1 << 2);
    // Dat tat ca cac chan tu PB8 den PB13 ve muc logic 0
    GPIOB->ODR &= ~((0b111111 << 8));
		// Ðat PA2 la muc logic cao
		GPIOA->BSRR = (1 << 2);    // Ðat PA2 thanh muc cao (logic 1)
		// Ðat PA3 là muc logic thap
		GPIOA->BRR = (1 << 3);     // Ðat PA3 thanh muc thap (logic 0) 

}

void Interrupt_Init(void)
{
    // Cau hinh chan EXTI1 va EXTI2 de su dung chan PB1 va PB2
    AFIO->EXTICR[0] = 0x00001111;

    // Cau hinh ngat tren canh xuong cho EXTI1 va EXTI2
    EXTI->FTSR |= (1 << 1) | (1 << 2);
    // Cho phep ngat EXTI1 va EXTI2
    EXTI->IMR |= (15<<0);

    //ngat EXTI1 va EXTI2
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}


void EXTI1_IRQHandler()
{
    // Xoa co ngat cua EXTI1
    EXTI->PR = (1 << 1);
	
		if(dutyCycle > 2000) dutyCycle = 2000;
    else dutyCycle += 200;
	
		TIM2->CCR2 = dutyCycle - 1;
}

void EXTI2_IRQHandler()
{
    // Xoa co ngat cua EXTI2
    EXTI->PR = (1 << 2);
		
		if(dutyCycle < 0) dutyCycle = 0;
		else    dutyCycle -= 200;
		TIM2->CCR2 = dutyCycle -1;
}


/* PWM - USE FOR TIMER 2*/
void PWM_Init(void)
{ 
    TIM2->PSC = 35;//Prescaler is (1/(35+1))*2=0.5MHz
	
    TIM2->ARR = 1999; //CCR = adc_value*10000/5000
    TIM2->CNT = 0;
	//Set PWM mode for each channel 2 for PA1 and PA2
    TIM2->CCR2 = 0; //chanel 2 duty circle = 0
    TIM2->CCMR1 |= (6 << 12); //set PWM1 mode for chanel 2
	//Set CCER for enable channel 2
    TIM2->CCER |= (1 << 4);
	//Enable Counter
    TIM2->CR1 |= (1 << 0);
}

/* Encoder - USE FOR TIMER 3*/
void Encoder_Init(void)
{		
		
		GPIOA->CRL = 0x22222444;
		 //khai bao TIM3 theo encoder
		RCC->APB1ENR |= (1 << 1);
		TIM3->SMCR |= (3 << 4);
    //Configure TIM3
		TIM3->PSC = 17; //Prescaler is 9MHz*(1/(17+1))*2=1MHz
		TIM3->CCMR1 |= (1 << 0)|(1 << 8);
		// configure timer 3
		TIM3->CCER |= (1 << 0)|(1 << 4);
		TIM3->ARR = 19999; // (1/1Mhz)*(999+1) = 1ms 
		TIM3->CNT = 0; 
		// enable timer interrupt and counter
		TIM3->DIER |= (1 << 0)|(1<<5);   // enable timer interrupr

    __enable_irq();
    NVIC_EnableIRQ(TIM3_IRQn);
}
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & (1 << 1))
    {
        encoderCount++;
        TIM3->SR &= ~(1 << 1);
    } //Kiem tra co ngat cho co dc dat hay khong( bit 1 trong trang thai TIM3 -> SR), neu co ngat dat thi tang bien so ma hoa va xoa co UIF bang cach ghi 0 vao thanh ghi do.

    if (TIM3->SR & (1 << 5))
    {
        encoderCount--;
        TIM3->SR &= ~(1 << 5);
    } //neu co ngat TÌF duoc dat, no se giam bien so bo ma hoa va xoa co ngat TIF bang cach ghi 0.
}

void Update_lcd(void){
				if ((unsigned long)(msTicks - oldtime_1) > Delay_1)
        {
            uint16_t temp1 =  dutyCycle/30; //duty cirle 
						lcd_goto(0, 0);
						char buffer[16];
						lcd_string("PWM: ");
						sprintf(buffer, "%d", temp1);
						lcd_string(buffer);
						lcd_string(" % ");
        }
        if ((unsigned long)(msTicks - oldtime_2) > Delay_2)
        {
            motorSpeed = (float)(encoderCount - prevEncoderCount) * 3; // cong thuc cua encoder
            prevEncoderCount = encoderCount;
            lcd_goto(0, 1);
            char buffer[16];
            sprintf(buffer, "%d", motorSpeed);
						lcd_string(buffer);
            lcd_string(" rpm ");
        }    
    }

