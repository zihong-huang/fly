#include "delay.h"

#if OS == 1
static uint8_t g_fac_us = 0;
/**
  * @brief  延时函数初始化
  * @param  无
  * @retval 无
  */
void delay_init(void)
{
	uint32_t reload;
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	
	reload = SystemCoreClock;
	
	
	reload = reload/configTICK_RATE_HZ;
	
	SysTick->LOAD = reload;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	g_fac_us = SystemCoreClock/1000000;		//72Mkz
	
}
#endif

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void delay_us(uint32_t xus)
{
#if OS == 0				//没有操作系统
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
#else							//有操作系统
	
	uint32_t ticks;
	uint32_t reload;
	uint32_t told, tnow, tcnt = 0;
	
	reload = SysTick->LOAD;
	ticks = xus * g_fac_us;
	told = SysTick->VAL;
	
	while(1)
	{
		
		tnow = SysTick->VAL;
		if(tnow != told)
		{
			if(tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if(tcnt >= ticks)
			{
				break;
			}
		}
	}
	
	
#endif
	
	
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_ms(uint32_t xms)
{
	while(xms--)
	{
		delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_s(uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
} 
