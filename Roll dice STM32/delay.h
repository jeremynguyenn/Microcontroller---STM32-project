
void delay(uint32_t dlyTicks);
/* Counts 1ms timeTicks */
volatile uint32_t msTicks = 0;
 
void SysTick_Handler(void)
{
	 /* Increment counter necessary in Delay()*/
		msTicks++;
} 

void delay(uint32_t dlyTicks)   // delay function
{
      uint32_t curTicks;
      curTicks = msTicks;
      while ((msTicks - curTicks) < dlyTicks) ;
}