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
 * @brief  初始化软件I2C使用的GPIO引脚。
 * @param  无
 * @retval 无
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
 * @brief  产生I2C起始条件。
 * @param  无
 * @retval 无
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
 * @brief  产生I2C停止条件。
 * @param  无
 * @retval 无
 */
void I2C_Stop(void) {
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SDA_HIGH();
}

/**
 * @brief  向I2C总线发送一个字节的数据。
 * @param  data: 要发送的数据字节
 * @retval 无
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
 * @brief  从I2C总线接收一个字节的数据。
 * @param  无
 * @retval 读取到的数据字节
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
 * @brief  发送ACK信号。
 * @param  无
 * @retval 无
 */
void I2C_Ack(void) {
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SCL_LOW();
}

/**
 * @brief  发送NACK信号。
 * @param  无
 * @retval 无
 */
void I2C_NAck(void) {
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_DELAY();
    I2C_SCL_LOW();
}

/**
 * @brief  向指定I2C设备的寄存器写入一个字节的数据。
 * @param  deviceAddr: I2C设备的地址
 * @param  registerAddr: 要写入的寄存器地址
 * @param  data: 要写入的数据
 * @retval 无
 */
void I2C_WriteRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data) {
    I2C_Start();
    I2C_SendByte(deviceAddr << 1); // 发送设备地址，写模式
    I2C_Ack();
    I2C_SendByte(registerAddr);    // 发送寄存器地址
    I2C_Ack();
    I2C_SendByte(data);            // 发送数据
    I2C_Ack();
    I2C_Stop();
}

/**
 * @brief  从指定I2C设备的寄存器读取一个字节的数据。
 * @param  deviceAddr: I2C设备的地址
 * @param  registerAddr: 要读取的寄存器地址
 * @retval 读取到的数据
 */
uint8_t I2C_Readiic(uint8_t deviceAddr, uint8_t registerAddr) {
    uint8_t data;

    I2C_Start();
    I2C_SendByte(deviceAddr << 1); // 发送设备地址，写模式
    I2C_Ack();
    I2C_SendByte(registerAddr);    // 发送寄存器地址
    I2C_Ack();
    I2C_Start();
    I2C_SendByte((deviceAddr << 1) | 1); // 发送设备地址，读模式
    I2C_Ack();
    data = I2C_ReadByte();         // 读取数据
    I2C_NAck();
    I2C_Stop();

    return data;
}
