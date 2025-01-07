#ifndef __I2C_H
#define __I2C_H

#include "sys.h"


/**
 * @brief  ��ʼ�����I2Cʹ�õ�GPIO���š�
 * @param  ��
 * @retval ��
 */
void SoftI2C_Init(void);

/**
 * @brief  ��ָ��I2C�豸�ļĴ���д��һ���ֽڵ����ݡ�
 * @param  deviceAddr: I2C�豸�ĵ�ַ
 * @param  registerAddr: Ҫд��ļĴ�����ַ
 * @param  data: Ҫд�������
 * @retval ��
 */
void I2C_WriteRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data);

/**
 * @brief  ��ָ��I2C�豸�ļĴ�����ȡһ���ֽڵ����ݡ�
 * @param  deviceAddr: I2C�豸�ĵ�ַ
 * @param  registerAddr: Ҫ��ȡ�ļĴ�����ַ
 * @retval ��ȡ��������
 */
uint8_t I2C_Readiic(uint8_t deviceAddr, uint8_t registerAddr);


#endif
