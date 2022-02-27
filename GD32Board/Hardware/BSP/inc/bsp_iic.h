#ifndef __BSP_IIC_H
#define __BSP_IIC_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include <gd32e230.h>
#include <inttypes.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
#define I2C_WR	        0		/* д����bit */
#define I2C_RD	        1		/* ������bit */

/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����4�д��뼴������ı�SCL��SDA������ */
#define PORT_I2C_SCL	GPIOB			/* GPIO�˿� */
#define PIN_I2C_SCL		GPIO_PIN_6		/* GPIO���� */

#define PORT_I2C_SDA	GPIOB			/* GPIO�˿� */
#define PIN_I2C_SDA		GPIO_PIN_7		/* GPIO���� */


/* �����дSCL��SDA�ĺ꣬�����Ӵ���Ŀ���ֲ�ԺͿ��Ķ��� */
#if 0	/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */

#define I2C_SCL_1()   GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		  /* SCL = 1 */
#define I2C_SCL_0()   GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
#define I2C_SDA_1()   GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		  /* SDA = 1 */
#define I2C_SDA_0()   GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* ��SDA����״̬ */

#else	/* �����֧ѡ��ֱ�ӼĴ�������ʵ��IO��д */
    /*��ע�⣺����д������IAR��߼����Ż�ʱ���ᱻ�����������Ż� */
#define I2C_SCL_1  gpio_bit_set(PORT_I2C_SCL,PIN_I2C_SCL)				/* SCL = 1 */
#define I2C_SCL_0  gpio_bit_reset(PORT_I2C_SCL,PIN_I2C_SCL)		/* SCL = 0 */

#define I2C_SDA_1  gpio_bit_set(PORT_I2C_SDA,PIN_I2C_SDA)				/* SDA = 1 */
#define I2C_SDA_0  gpio_bit_reset(PORT_I2C_SDA,PIN_I2C_SDA)				/* SDA = 0 */
	
#define I2C_SDA_READ()  gpio_input_bit_get(PORT_I2C_SDA,PIN_I2C_SDA)	 /* ��SDA����״̬ */

#endif

void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(uint8_t ack);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
void i2c_CfgGpio(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
#endif /* __BSP_I2C_GPIO_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/

