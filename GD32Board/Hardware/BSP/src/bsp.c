#include "bsp.h"

void USART_Config(void);

void BSP_init(void)
{	
	i2c_CheckDevice(MAX30100_SLAVE_ADDRESS);	//检查从机地址是否正确
	MAX30100_Init();
}


