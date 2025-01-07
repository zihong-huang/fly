#include "LED.h"

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	//Init_gpioC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	

	
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

}

void LED_ON(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

void LED_OFF(void)
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LED_State1(void)
{
		LED_ON();
		delay_ms(2);
		LED_OFF();
		
}

void LED_State2(void)
{
		for(u8 i = 0; i<2; i++){
		LED_ON();
		delay_ms(100);
		LED_OFF();
		delay_ms(100);
		
		}
}

void LED_State3(void)
{
		for(u8 i = 0; i<3; i++){
		LED_ON();
		delay_ms(100);
		LED_OFF();
		delay_ms(100);
		
		}
}

void LED_State4(void)
{
		for(u8 i = 0; i<4; i++){
		LED_ON();
		delay_ms(100);
		LED_OFF();
		delay_ms(100);
		
		}
}


void LED_State_Choice(u8 input)
{
		switch(input)
		{
		case 1:
						LED_State1();
						break;
		case 2:
						LED_State2();
						break;
		case 3:
						LED_State3();
						break;
		case 4:
						LED_State4();
						break;
		default:
						break;
		}
			
}





