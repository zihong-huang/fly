#include "power.h"

#define BATTERY_MAX_VOLTAGE 4.2  // ��������ѹ
#define BATTERY_MIN_VOLTAGE 3.2  // �����͵�ѹ

void ADC_Init_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    // ʹ��ADC1��GPIOBʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);

    // ����PB1Ϊģ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // ����ADC1
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;  // ��ͨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // ����ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  // �����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;  // ת��ͨ����Ϊ1
    ADC_Init(ADC1, &ADC_InitStructure);

    // ����ADC1��ͨ��
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);  // PB1 ��Ӧ ADC_Channel_9

    ADC_Cmd(ADC1, ENABLE);  // ʹ��ADC1

    // У׼ADC1
    ADC_ResetCalibration(ADC1);  // ����У׼�Ĵ���
    while(ADC_GetResetCalibrationStatus(ADC1));  // �ȴ�У׼�Ĵ����������
    ADC_StartCalibration(ADC1);  // ��ʼУ׼
    while(ADC_GetCalibrationStatus(ADC1));  // �ȴ�У׼���

    // ����ADCת��
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t ADC_Get_Value(void) {
    return ADC_GetConversionValue(ADC1);  // ��ȡADC��ת��ֵ
}

float ADC_Get_Voltage(void) {
    uint16_t adc_value = ADC_Get_Value();
    float voltage = (adc_value / 4096.0) * 3.3;  // 3.3VΪ�ο���ѹ��12λADC���ֵΪ4096
    return voltage;
}

uint8_t Get_Battery_Percentage(void) {
    float voltage = ADC_Get_Voltage();

    // ����ѹ��������Ч��Χ��
    if (voltage > BATTERY_MAX_VOLTAGE) {
        voltage = BATTERY_MAX_VOLTAGE;
    } else if (voltage < BATTERY_MIN_VOLTAGE) {
        voltage = BATTERY_MIN_VOLTAGE;
    }

    // ��������ٷֱ�
    float percentage = (voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100;

    return (uint8_t)percentage;  // ����0��100֮��İٷֱ�ֵ
}
