#include "i2c_master.h"
#include "uart.h"
#include "stdint.h"

#define I2C1_SLAVE_ADDRESS7    	0x32
#define I2C1_WR                 0x00                     //?
#define I2C1_RD                 0x01                     //?

#ifdef software_i2c
#define SCL_H 		gpio_bit_set(GPIOA,GPIO_PIN_11);
#define SDA_H 		gpio_bit_set(GPIOA,GPIO_PIN_12);
#define SCL_L 		gpio_bit_reset(GPIOA,GPIO_PIN_11);
#define SDA_L 		gpio_bit_reset(GPIOA,GPIO_PIN_12);
#define SDA_read 	gpio_input_bit_get(GPIOA,GPIO_PIN_12)

static void I2C_Delay(uint16_t n)
{
    uint8_t i;

    while(n--)
    {
        /* 1us */
        for(i = 0; i < 15; i++)
        {
            __asm("NOP");
        }
    }
}

//GPIO配置函数
void I2C_GPIO_Configuration(void)
{
		/* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* configure GPIO pins of I2C1 */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_11);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
}


//SDA上拉输入
void I2C_SDA_IN()
{
	  /* configure charge GPIO port */ 
	  gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12);
}


//重置SDA输上拉出
void I2C_SDA_OUT()
{
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
}


//I2C初始化
void I2C_Initializes(void)
{
  I2C_GPIO_Configuration();
  SCL_H;                                  //????
  SDA_H;
}

int I2C_Start(void)
{
	I2C_SDA_OUT();
	
	SDA_H;
	SCL_H;
	I2C_Delay(2);
	SDA_L;
	I2C_Delay(2);
	SCL_L;
	I2C_Delay(2);
	return OK;
}


void I2C_Stop(void)
{
	I2C_SDA_OUT();
	SDA_L;	
	SCL_H;
	I2C_Delay(2);
	SDA_H;
	I2C_Delay(2);
}


//???????,????????,??????????
static void I2C_Ack()
{
	I2C_SDA_OUT();	//??SDA?????
	SDA_L;//??
	I2C_Delay(2);
	SCL_H;
	I2C_Delay(2);
	SCL_L;
	I2C_Delay(2);
	SDA_H;
}


//????????,????????,???????????
static void I2C_NoAck()
{
	I2C_SDA_OUT();	//??SDA?????
	SDA_H;//??
	I2C_Delay(2);
	SCL_H;
	I2C_Delay(2);
	SCL_L;
	I2C_Delay(2);
}


int8_t I2C_GetAck(void)
{
  uint8_t time = 0;
	I2C_SDA_OUT();	//??SDA?????
	SDA_H;
	I2C_Delay(2);
	SCL_H;
	I2C_Delay(2);
	I2C_SDA_IN();
	while(SDA_read)//?????,???,??????
	{
		time++;
		if(time > 250)
		{
			//?????????????,??,????????????????
			//SCCB_Stop();
			SCL_L;
			return ERROR;
		}
	}
	SCL_L;
	I2C_Delay(2);
	return OK;
}


//I2C?????
void I2C_SendByte(uint8_t Data)
{
  uint8_t cnt;
  I2C_SDA_OUT();	//??SDA?????
 
  for(cnt=0; cnt<8; cnt++)
  {
    if(Data & 0x80)
    {
      SDA_H;                              //SDA?,????????
    }
    else
    {
      SDA_L;                               //SDA?
    }
		I2C_Delay(2);
		SCL_H;  
		I2C_Delay(2);
		SCL_L;
		if(cnt == 7)
			SDA_H;
    Data <<= 1;                               //SCL?(????)
		I2C_Delay(2);
  }
}
 
//I2C??????
int8_t I2C_ReadByte(void)
{
  uint8_t cnt;
  uint8_t data = 0;
  I2C_SDA_IN();	//??SDA?????
	
  for(cnt=0; cnt<8; cnt++)
  {
		data <<= 1;
    SCL_H;                                 //SCL?
		I2C_Delay(2);
    if(SDA_read)
    {
      data++;                              //SDA?(????)
    }
		SCL_L;
		I2C_Delay(2);
  }

  return data;                                   //????
}

int8_t i2c_write_byte(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
  I2C_Start();
	I2C_SendByte(slave_addr);
	I2C_NoAck();
	I2C_SendByte(reg);
	I2C_NoAck();
	I2C_SendByte(data);
	I2C_NoAck();
	I2C_Stop();
	return OK;
}

int8_t i2c_read_byte(uint8_t slave_addr, uint8_t reg)
{
	int8_t data = 0;
	
	I2C_Start();
	I2C_SendByte(slave_addr);
	I2C_NoAck();
	I2C_SendByte(reg);
	I2C_NoAck();
	I2C_Start();
	I2C_SendByte(slave_addr+1);
	I2C_NoAck();
	data = I2C_ReadByte();
	I2C_NoAck();
  I2C_Stop(); 
	return data;
}
#else
void i2c_gpio_config(void)
{
//	    /* enable GPIOB clock */
//    rcu_periph_clock_enable(RCU_GPIOB);
//    /* enable I2C0 clock */
//    rcu_periph_clock_enable(RCU_I2C0);
//    /* connect PB6 to I2C0_SCL */
//    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_6);
//    /* connect PB7 to I2C0_SDA */
//    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_7);
//    /* configure GPIO pins of I2C0 */
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_6);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_6);
//    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_7);
//    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
	
		/* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	  /* enable I2C1 clock */
    rcu_periph_clock_enable(RCU_I2C1);
	  /* connect PB10 to I2C1_SCL */
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_11);
    /* connect PB11 to I2C1_SDA */
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_12);
    /* configure GPIO pins of I2C1 */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_11);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP,GPIO_PIN_12);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_12);
}

/*!
    \brief      cofigure the I2C0 and I2C1 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)
{
//    /* I2C clock configure */
//    i2c_clock_config(I2C0, 100000, I2C_DTCY_16_9);
//    /* I2C address configure */
//    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
//    /* enable I2C0 */
//    i2c_enable(I2C0);
//    /* enable acknowledge */
//    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	
    i2c_clock_config(I2C1, 100000, I2C_DTCY_16_9);
    /* I2C address configure */
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C1_SLAVE_ADDRESS7);
    /* enable I2C1 */
    i2c_enable(I2C1);
    /* enable acknowledge */
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);
}

void i2c_master_init(void)
{
	printf("\nI2C Init\n");
	i2c_gpio_config();
	i2c_config();
	i2c_software_reset_config(I2C1, I2C_SRESET_SET);
	i2c_software_reset_config(I2C1, I2C_SRESET_RESET);
}

int8_t i2c_write_byte(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
		/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY));

	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C1);

	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));

	/* send slave address to I2C bus*/
	i2c_master_addressing(I2C1, I2C1_SLAVE_ADDRESS7, I2C_TRANSMITTER);

	/* wait until ADDSEND bit is set*/
	while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
	/* clear ADDSEND bit */
	i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
	/* wait until the transmit data buffer is empty */
	while(SET != i2c_flag_get(I2C1, I2C_FLAG_TBE));
	/* send a data byte */
	i2c_data_transmit(I2C1,reg);
	/* wait until BTC bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_BTC));
	/* send the byte to be written */
	i2c_data_transmit(I2C1, data);
	/* wait until BTC bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_BTC));
	/* wait until the transmission data register is empty*/
//    while(!i2c_flag_get(I2C1, I2C_FLAG_TBE));
	/* send a stop condition to I2C bus*/
	i2c_stop_on_bus(I2C1);
	while(I2C_CTL0(I2C1)&0x0200);
	while(!i2c_flag_get(I2C1, I2C_FLAG_STPDET));
	/* clear the STPDET bit */
	i2c_enable(I2C1);
		
	return 0;
}

int8_t i2c_read_byte(uint8_t slave_addr, uint8_t reg)
{
	uint8_t i = 0;
	uint8_t data;
	
	printf("\n%d\n",i++);

	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY));
//	printf("\n%d\n",i++);
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C1);
		printf("\n%d\n",i++);
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));
//		printf("\n%d\n",i++);
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C1, slave_addr, I2C_TRANSMITTER);
//		printf("\n%d\n",i++);
	/* wait until ADDSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
//		printf("\n%d\n",i++);
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C1,I2C_FLAG_ADDSEND);
//		printf("\n%d\n",i++);
	/* wait until the transmit data buffer is empty */
	while(SET != i2c_flag_get( I2C1 , I2C_FLAG_TBE));
//		printf("\n%d\n",i++);
	/* enable I2C0*/
	i2c_enable(I2C1);
		printf("\n%d\n",i++);
	/* send the EEPROM's internal address to write to */
	i2c_data_transmit(I2C1, reg);   //???????
		printf("\n%d\n",i++);
	/* wait until BTC bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_BTC));
		printf("\n%d\n",i++);
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C1);
		printf("\n%d\n",i++);
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_SBSEND));
		printf("\n%d\n",i++);
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C1, slave_addr, I2C_RECEIVER); //????????
		printf("\n%d\n",i++);
	/* disable acknowledge */
	i2c_ack_config(I2C1,I2C_ACK_DISABLE);
		printf("\n%d\n",i++);
	/* wait until ADDSEND bit is set */
	while(!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND));
		printf("\n%d\n",i++);
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C1,I2C_FLAG_ADDSEND);
		printf("\n%d\n",i++);
	/* send a stop condition to I2C bus */
	i2c_stop_on_bus(I2C1);
		printf("\n%d\n",i++);
	if(i2c_flag_get(I2C1, I2C_FLAG_RBNE)){
			printf("\n%d\n",i++);
	/* read a byte from the EEPROM */
	data = i2c_data_receive(I2C1); //???????
		}	
		printf("\n%d\n",i++);
//    /* N=1,reset ACKEN bit before clearing ADDRSEND bit */
//    i2c_ack_config(I2C0, I2C_ACK_DISABLE);
//    /* clear ADDSEND bit */
//    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
//    /* N=1,send stop condition after clearing ADDRSEND bit */
//    i2c_stop_on_bus(I2C0);
//    /* wait until the RBNE bit is set */
//    while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));
//    /* read a data from I2C_DATA */
//    data = i2c_data_receive(I2C0);
	while(I2C_CTL0(I2C1)&0x0200);
	/* enable acknowledge */
	i2c_ack_config(I2C1, I2C_ACK_ENABLE);
	i2c_ackpos_config(I2C1,I2C_ACKPOS_CURRENT);

	return data;
}


int8_t i2c_master_write(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	
		return OK;
}

int8_t i2c_master_read(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
				
		return OK;
}


void i2c_master_reset(uint8_t slave_addr)
{

}

void i2c3_master_unlock()
{

}
#endif
