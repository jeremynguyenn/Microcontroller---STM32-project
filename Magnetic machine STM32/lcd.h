#include "stm32f10x.h"
#include "stdio.h"

void lcd_init(void);
void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_goto(uint8_t x, uint8_t y);
void lcd_char(char c);
void lcd_string(char str[32]);




void lcd_init(void)
{
	delay(5);
	lcd_cmd(0x02);  		// 4-bit mode
	lcd_cmd(0x02);  		// 4-bit mode, 2 lines, 5x8 dots
	lcd_cmd(0x08);
	lcd_cmd(0x00);			// display on, cursor on, cursor blinking
	lcd_cmd(0x0F);
	lcd_cmd(0x00);			
	lcd_cmd(0x06);			// Entry mode, increment cursor, no shift
}

void lcd_cmd(uint8_t cmd)
{
	GPIOB->ODR &= ~(0x0f<<8); // reset PB8~PB11 
	GPIOB->ODR &= ~(1UL<<12);	// R/S = 0
	GPIOB->ODR |= (1<<13); 	// E = 1
	GPIOB->ODR |= (cmd << 8); 
	delay(1);
	GPIOB->ODR &= ~(1UL<<13); // E = 0
	GPIOB->ODR |= (1<<12);	// R/S = 1	
	delay(1);
}
void lcd_data(uint8_t data)
{
	GPIOB->ODR &= ~(0x0f<<8); // reset PB8~PB11 
	GPIOB->ODR |= (1UL<<12);	// R/S = 1
	GPIOB->ODR |= (1<<13); 	// E = 1
	GPIOB->ODR |= (data << 8); 
	delay(1);
	GPIOB->ODR &= ~(1UL<<13); // E = 0
	delay(1);
}

void lcd_clear(void){
	lcd_cmd(0);
	lcd_cmd(1);
	delay(1);
}

void lcd_goto(uint8_t x, uint8_t y){
	if(y==0){
		lcd_cmd(0x08);
		lcd_cmd(x);
	}
	else{
		lcd_cmd(0x0C);
		lcd_cmd(x);
	}
}

void lcd_char(char c){
	lcd_data(c >> 4);    // send 4 upper bit
	lcd_data(c & 0x0f);  // send 4 lower bit
}

void lcd_string(char str[32]){
	uint8_t k = 0;
	while(str[k]!= '\0'){
		if(k==16){
			lcd_cmd(0x0C);   // move cursor to 2nd row
			lcd_cmd(0x00);
		}
		lcd_char(str[k]);
		k++;
	}
}