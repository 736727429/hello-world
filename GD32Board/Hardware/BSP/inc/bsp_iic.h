#ifndef __BSP_IIC_H
#define __BSP_IIC_H

/* 包含头文件 ----------------------------------------------------------------*/
#include <gd32e230.h>
#include <inttypes.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
#define I2C_WR	        0		/* 写控制bit */
#define I2C_RD	        1		/* 读控制bit */

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define PORT_I2C_SCL	GPIOB			/* GPIO端口 */
#define PIN_I2C_SCL		GPIO_PIN_6		/* GPIO引脚 */

#define PORT_I2C_SDA	GPIOB			/* GPIO端口 */
#define PIN_I2C_SDA		GPIO_PIN_7		/* GPIO引脚 */


/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
#if 0	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */

#define I2C_SCL_1()   GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		  /* SCL = 1 */
#define I2C_SCL_0()   GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)		/* SCL = 0 */
	
#define I2C_SDA_1()   GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		  /* SDA = 1 */
#define I2C_SDA_0()   GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)		/* SDA = 0 */
	
#define I2C_SDA_READ()  GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)	/* 读SDA口线状态 */

#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
#define I2C_SCL_1  gpio_bit_set(PORT_I2C_SCL,PIN_I2C_SCL)				/* SCL = 1 */
#define I2C_SCL_0  gpio_bit_reset(PORT_I2C_SCL,PIN_I2C_SCL)		/* SCL = 0 */

#define I2C_SDA_1  gpio_bit_set(PORT_I2C_SDA,PIN_I2C_SDA)				/* SDA = 1 */
#define I2C_SDA_0  gpio_bit_reset(PORT_I2C_SDA,PIN_I2C_SDA)				/* SDA = 0 */
	
#define I2C_SDA_READ()  gpio_input_bit_get(PORT_I2C_SDA,PIN_I2C_SDA)	 /* 读SDA口线状态 */

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

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

