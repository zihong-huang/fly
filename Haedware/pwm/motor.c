#include "motor.h"

// 初始化PWM，使用TIM2的通道1 (PA15) 和 TIM3的通道1 (PB3)、通道2 (PB4)、通道3 (PB5)
void PWM_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
	

    // 使能GPIO和TIM2、TIM3时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
		
	
		GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
		GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
		
    // 关闭JTAG功能以释放PA15（PB3为SWO功能，不受影响）
		//GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		
		//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
		
    // 使能TIM2重映射：将TIM2_CH1和CH2重映射到PA15和PB3

	
    // 配置PA15为复用推挽输出 (TIM2_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置PB3为复用推挽输出 (TIM2_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置PB4和PB5为复用推挽输出 (TIM3_CH1和CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		

    // 设置TIM2的定时器频率
    TIM_TimeBaseStructure.TIM_Period = arr;  // PWM周期为1000
    TIM_TimeBaseStructure.TIM_Prescaler = psc;  // 时钟分频为72MHz / (71+1) = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 设置TIM3的定时器频率
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // 配置TIM2通道1 (PA15)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 配置TIM2通道2 (PB3)
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 配置TIM3通道1 (PB4)
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 配置TIM3通道2 (PB5)
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // 启动TIM2和TIM3定时器
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

}

// 设置TIM2 PA15和TIM3 PB3、PB4、PB5的PWM占空比
void PWM_SetDutyCycle(uint16_t pulseA15, uint16_t pulseB3, uint16_t pulseB4, uint16_t pulseB5)
{
    TIM_SetCompare1(TIM2, pulseA15);  // 更新 TIM2 通道1 (PA15) 的占空比
    TIM_SetCompare2(TIM2, pulseB3);   // 更新 TIM2 通道2 (PB3) 的占空比
    TIM_SetCompare1(TIM3, pulseB4);   // 更新 TIM3 通道1 (PB4) 的占空比
    TIM_SetCompare2(TIM3, pulseB5);   // 更新 TIM3 通道2 (PB5) 的占空比
}


void Motor_Init(void)
{
		PWM_SetDutyCycle(100, 0, 0, 0);
		delay_ms(100);
		PWM_SetDutyCycle(0, 100, 0, 0);
		delay_ms(100);
		PWM_SetDutyCycle(0, 0, 100, 0);
		delay_ms(100);
		PWM_SetDutyCycle(0, 0, 0, 100);
		delay_ms(100);
		PWM_SetDutyCycle(0, 0, 0, 0);
}

