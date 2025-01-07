#include "i2c.h"

#define I2C_SCL_PIN  GPIO_Pin_6
#define I2C_SDA_PIN  GPIO_Pin_7
#define I2C_GPIO_PORT GPIOB

#define I2C_SCL_HIGH()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW()   GPIO_ResetBits(I2C_GPIO_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH()  GPIO_SetBits(I2C_GPIO_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW()   GPIO_ResetBits(I2C_GPIO_PORT, I2C_SDA_PIN)

#define I2C_DELAY() for (volatile int i = 0; i < 30; i++)

/**
 * @brief  ��ʼ�����I2Cʹ�õ�GPIO���š�
 * @param  ��
 * @retval ��
 */
void SoftI2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

/**
 * @brief  ����I2C��ʼ������
 * @param  ��
 * @retval ��
 */
void I2C_Start(void) {
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SDA_LOW();
    I2C_DELAY();
    I2C_SCL_LOW();
}

/**
 * @brief  ����I2Cֹͣ������
 * @param  ��
 * @retval ��
 */
void I2C_Stop(void) {
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SDA_HIGH();
}

/**
 * @brief  ��I2C���߷���һ���ֽڵ����ݡ�
 * @param  data: Ҫ���͵������ֽ�
 * @retval ��
 */
void I2C_SendByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            I2C_SDA_HIGH();
        } else {
            I2C_SDA_LOW();
        }
        I2C_DELAY();
        I2C_SCL_HIGH();
        I2C_DELAY();
        I2C_SCL_LOW();
        data <<= 1;
    }
}

/**
 * @brief  ��I2C���߽���һ���ֽڵ����ݡ�
 * @param  ��
 * @retval ��ȡ���������ֽ�
 */
uint8_t I2C_ReadByte(void) {
    uint8_t data = 0;

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_HIGH();
        I2C_DELAY();
        if (GPIO_ReadInputDataBit(I2C_GPIO_PORT, I2C_SDA_PIN)) {
            data |= 0x01;
        }
        I2C_SCL_LOW();
        I2C_DELAY();
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

    return data;
}

/**
 * @brief  ����ACK�źš�
 * @param  ��
 * @retval ��
 */
void I2C_Ack(void) {
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SCL_LOW();
}

/**
 * @brief  ����NACK�źš�
 * @param  ��
 * @retval ��
 */
void I2C_NAck(void) {
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SCL_LOW();
}

/**
 * @brief  ��ָ��I2C�豸�ļĴ���д��һ���ֽڵ����ݡ�
 * @param  deviceAddr: I2C�豸�ĵ�ַ
 * @param  registerAddr: Ҫд��ļĴ�����ַ
 * @param  data: Ҫд�������
 * @retval ��
 */
void I2C_WriteRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data) {
    I2C_Start();
    I2C_SendByte(deviceAddr << 1); // �����豸��ַ��дģʽ
    I2C_Ack();
    I2C_SendByte(registerAddr);    // ���ͼĴ�����ַ
    I2C_Ack();
    I2C_SendByte(data);            // ��������
    I2C_Ack();
    I2C_Stop();
}

/**
 * @brief  ��ָ��I2C�豸�ļĴ�����ȡһ���ֽڵ����ݡ�
 * @param  deviceAddr: I2C�豸�ĵ�ַ
 * @param  registerAddr: Ҫ��ȡ�ļĴ�����ַ
 * @retval ��ȡ��������
 */
uint8_t I2C_Readiic(uint8_t deviceAddr, uint8_t registerAddr) {
    uint8_t data;

    I2C_Start();
    I2C_SendByte(deviceAddr << 1); // �����豸��ַ��дģʽ
    I2C_Ack();
    I2C_SendByte(registerAddr);    // ���ͼĴ�����ַ
    I2C_Ack();
    I2C_Start();
    I2C_SendByte((deviceAddr << 1) | 1); // �����豸��ַ����ģʽ
    I2C_Ack();
    data = I2C_ReadByte();         // ��ȡ����
    I2C_NAck();
    I2C_Stop();

    return data;
}
