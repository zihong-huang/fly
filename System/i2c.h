#ifndef __I2C_H
#define __I2C_H

#include "sys.h"


/**
 * @brief  初始化软件I2C使用的GPIO引脚。
 * @param  无
 * @retval 无
 */
void SoftI2C_Init(void);

/**
 * @brief  向指定I2C设备的寄存器写入一个字节的数据。
 * @param  deviceAddr: I2C设备的地址
 * @param  registerAddr: 要写入的寄存器地址
 * @param  data: 要写入的数据
 * @retval 无
 */
void I2C_WriteRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data);

/**
 * @brief  从指定I2C设备的寄存器读取一个字节的数据。
 * @param  deviceAddr: I2C设备的地址
 * @param  registerAddr: 要读取的寄存器地址
 * @retval 读取到的数据
 */
uint8_t I2C_Readiic(uint8_t deviceAddr, uint8_t registerAddr);


#endif
