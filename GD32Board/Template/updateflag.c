#include "updateflag.h"
#include "rtc_task.h"

//擦除flash标志位页
void Flash_Erase_page(uint32_t page_Address)
{
	fmc_page_erase(page_Address);
}

//flash 写入
void Flash_ProgramBytes(uint32_t Address,uint8_t *Buffer,uint16_t Bytecount)
{
	uint16_t i = 0;
	
	fmc_unlock();
	Flash_Erase_page(Address); //擦除再写入
	while(i<Bytecount)
	{
		fmc_word_program(Address,*(uint32_t*)Buffer);
		i = i + 4;
		Address = Address + 4;
		Buffer = Buffer + 4;
	}
	fmc_lock();
}

//写入
void write_updateflag(uint8_t *data)
{
	uint8_t write_data[256] = {0};
	
	memset(write_data,0xFF,sizeof(write_data));
	write_data[0] = write_data[1] = write_data[8] = write_data[9] = 0x5A;
	write_data[16] = write_data[17] = 0xA5;
	
	write_data[4] = data[0];
	write_data[5] = data[1];
	write_data[6] = data[2];
	write_data[12] = data[3];
	write_data[13] = data[4];
	write_data[14] = data[5];
	
	Flash_ProgramBytes(FLASH_APP_ADDR,write_data,sizeof(write_data));
}


//读取
uint8_t read_updateflag()
{
	uint32_t test_buffer[5] = {0};
	uint8_t data[16] = {0};
	uint8_t i = 0;
	
	for(i = 0;i < 5;i++)
		test_buffer[i] = *(volatile uint32_t*)(FLASH_APP_ADDR + i * 4)&0x00ffffff;
	
	data[0] = (uint8_t)(test_buffer[0] & 0xff);			//0x5a
	data[1] = (uint8_t)((test_buffer[0] >> 8) & 0xff);	//0x5a
	data[2] = (uint8_t)(test_buffer[1] & 0xff);			//year
	data[3] = (uint8_t)((test_buffer[1] >> 8) & 0xff);	//month
	data[4] = (uint8_t)((test_buffer[1] >> 16) & 0xff);	//day
	data[5] = (uint8_t)(test_buffer[2] & 0xff);			//0x5a
	data[6] = (uint8_t)((test_buffer[2] >> 8) & 0xff);	//0x5a
	data[7] = (uint8_t)(test_buffer[3] & 0xff);			//hour
	data[8] = (uint8_t)((test_buffer[3] >> 8) & 0xff);	//min
	data[9] = (uint8_t)((test_buffer[3] >> 16) & 0xff);	//sec
	data[10] = (uint8_t)(test_buffer[4] & 0xff);		//0xa5	
	data[11] = (uint8_t)((test_buffer[4] >> 8) & 0xff);	//0xa5
	
	if((0x5A == data[0])&&(0x5A == data[5])&&(0xA5 == data[10]))
	{
		if((0x5A == data[1])&&(0x5A == data[6])&&(0xFF == (data[1] + data[11])))
		{
			alarm_set_time_t2[0] = data[2];
			alarm_set_time_t2[1] = data[3];
			alarm_set_time_t2[2] = data[4];
			alarm_set_time_t2[3] = data[7];
			alarm_set_time_t2[4] = data[8];
			alarm_set_time_t2[5] = data[9];
			
			rtc_alarm_set_flag = true;
			
			for(uint8_t i = 0;i < 5;i++)
				alarm_set_time_t[i] = alarm_set_time_t2[i];
			printf("alarm_set_time_flash:%d-%d-%d %d:%d:0\n",alarm_set_time_t2[0],alarm_set_time_t2[1],alarm_set_time_t2[2],alarm_set_time_t2[3],alarm_set_time_t2[4]);
		
		}
	}
	else
	{
		rtc_alarm_set_flag = false;
		printf("alarm_set_time_flash read error.\n");
	}
}


void Flash_Erase_page_test(uint32_t page_Address)
{
	fmc_unlock();
	fmc_page_erase(page_Address);
	fmc_lock();
}


