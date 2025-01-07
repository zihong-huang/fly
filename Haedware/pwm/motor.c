#include "motor.h"

// ��ʼ��PWM��ʹ��TIM2��ͨ��1 (PA15) �� TIM3��ͨ��1 (PB3)��ͨ��2 (PB4)��ͨ��3 (PB5)
void PWM_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
	

    // ʹ��GPIO��TIM2��TIM3ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
		
	
		GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
		GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
		
    // �ر�JTAG�������ͷ�PA15��PB3ΪSWO���ܣ�����Ӱ�죩
		//GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		
		//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
		
    // ʹ��TIM2��ӳ�䣺��TIM2_CH1��CH2��ӳ�䵽PA15��PB3

	
    // ����PA15Ϊ����������� (TIM2_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ����PB3Ϊ����������� (TIM2_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ����PB4��PB5Ϊ����������� (TIM3_CH1��CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		

    // ����TIM2�Ķ�ʱ��Ƶ��
    TIM_TimeBaseStructure.TIM_Period = arr;  // PWM����Ϊ1000
    TIM_TimeBaseStructure.TIM_Prescaler = psc;  // ʱ�ӷ�ƵΪ72MHz / (71+1) = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // ����TIM3�Ķ�ʱ��Ƶ��
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // ����TIM2ͨ��1 (PA15)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // ����TIM2ͨ��2 (PB3)
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // ����TIM3ͨ��1 (PB4)
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ����TIM3ͨ��2 (PB5)
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ����TIM2��TIM3��ʱ��
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

}

// ����TIM2 PA15��TIM3 PB3��PB4��PB5��PWMռ�ձ�
void PWM_SetDutyCycle(uint16_t pulseA15, uint16_t pulseB3, uint16_t pulseB4, uint16_t pulseB5)
{
    TIM_SetCompare1(TIM2, pulseA15);  // ���� TIM2 ͨ��1 (PA15) ��ռ�ձ�
    TIM_SetCompare2(TIM2, pulseB3);   // ���� TIM2 ͨ��2 (PB3) ��ռ�ձ�
    TIM_SetCompare1(TIM3, pulseB4);   // ���� TIM3 ͨ��1 (PB4) ��ռ�ձ�
    TIM_SetCompare2(TIM3, pulseB5);   // ���� TIM3 ͨ��2 (PB5) ��ռ�ձ�
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

