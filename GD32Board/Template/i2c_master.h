#ifndef _I2C_MASTER_H_
#define _I2C_MASTER_H_

#include "global.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define I2C_MASTER0 I2C0
#define I2C_MASTER1 I2C1
#define I2C_BAUDRATE 100000U

#define MAX_I2C_SLAVE			2

#define software_i2c 1

typedef struct {
	uint32_t base;
	uint32_t baudrate;
	uint8_t slave_addr;
}I2C_MASTER_CONFIG;

extern I2C_MASTER_CONFIG i2c_master_config[2];



typedef struct {
	//i2c_rtos_handle_t master_rtos_handle;
	I2C_MASTER_CONFIG config;
	uint8_t error_num;
	
} I2C_CLIENT;


#ifdef software_i2c
void I2C_GPIO_Configuration(void);
void I2C_SDA_IN();
void I2C_SDA_OUT();
void I2C_Initializes(void);
int I2C_Start(void);
void I2C_Stop(void);
static void I2C_Ack();
static void I2C_NoAck();
int8_t I2C_GetAck(void);
void I2C_SendByte(uint8_t Data);
int8_t I2C_ReadByte(void);
int8_t i2c_write_byte(uint8_t slave_addr, uint8_t reg, uint8_t data);
int8_t i2c_read_byte(uint8_t slave_addr, uint8_t reg);
#else
void i2c_master_init(void);

int8_t i2c_write_byte(uint8_t slave_addr, uint8_t reg, uint8_t data);

int8_t i2c_read_byte(uint8_t slave_addr, uint8_t reg);

int8_t i2c_master_write(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len);

int8_t i2c_master_read(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len);

void i2c_master_reset(uint8_t slave_addr);

void i2c3_master_unlock();
#endif

#endif
