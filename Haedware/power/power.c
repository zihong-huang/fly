#include "power.h"

#define BATTERY_MAX_VOLTAGE 4.2  // 电池满电电压
#define BATTERY_MIN_VOLTAGE 3.2  // 电池最低电压

void ADC_Init_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    // 使能ADC1和GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);

    // 配置PB1为模拟输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置ADC1
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  // 单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // 连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;  // 转换通道数为1
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置ADC1的通道
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);  // PB1 对应 ADC_Channel_9

    ADC_Cmd(ADC1, ENABLE);  // 使能ADC1

    // 校准ADC1
    ADC_ResetCalibration(ADC1);  // 重置校准寄存器
    while(ADC_GetResetCalibrationStatus(ADC1));  // 等待校准寄存器重置完成
    ADC_StartCalibration(ADC1);  // 开始校准
    while(ADC_GetCalibrationStatus(ADC1));  // 等待校准完成

    // 启动ADC转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t ADC_Get_Value(void) {
    return ADC_GetConversionValue(ADC1);  // 读取ADC的转换值
}

float ADC_Get_Voltage(void) {
    uint16_t adc_value = ADC_Get_Value();
    float voltage = (adc_value / 4096.0) * 3.3;  // 3.3V为参考电压，12位ADC最大值为4096
    return voltage;
}

uint8_t Get_Battery_Percentage(void) {
    float voltage = ADC_Get_Voltage();

    // 将电压限制在有效范围内
    if (voltage > BATTERY_MAX_VOLTAGE) {
        voltage = BATTERY_MAX_VOLTAGE;
    } else if (voltage < BATTERY_MIN_VOLTAGE) {
        voltage = BATTERY_MIN_VOLTAGE;
    }

    // 计算电量百分比
    float percentage = (voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100;

    return (uint8_t)percentage;  // 返回0到100之间的百分比值
}
