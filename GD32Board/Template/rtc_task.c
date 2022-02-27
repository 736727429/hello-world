/*
 * rtc_task.c
 *
 *  Created on: Jun 13, 2018
 *      Author: yang
 */

#include "rtc_task.h"
#include "reset.h"
#include "i2c_master.h"
#include "bcd.h"
#include <stdio.h>
#include "systick.h"
#include "uart.h"
#include "timer.h"
#include "gd32e230c_eval.h"
#include "hardware_switch.h"
#include "Iot.h"
#include "nbiot_task.h"
#include "updateflag.h"

rtc_time_t rtc_time;
rx8010_module_t rtc_rx;
uint32_t timestamp;
uint8_t sync_flag = 0;
uint8_t get_time_cnt = 0;

RTC_DEV_T g_rtc_dev;
uint8_t alarm_poweron_time[4] = {0};
uint8_t alarm_poweroff_time[4] = {0};
uint8_t alarm_poweroff_flag = 0;
uint8_t rtc_print_count = 0;

volatile bool rtc_time_ask_flag = false;
volatile bool rtc_alarm_timeon_ask_flag = false;
volatile bool rtc_alarm_timeoff_ask_flag = false;
volatile bool rtc_alarm_set_flag = false;

uint8_t days_of_mon[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
uint8_t week_analy[7] = {0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x01};
/******************* week初始化***********************
 * 星期    日    一    二    三    四    五    六
 *  %x   0x01  0x02  0x04  0x08 0x10  0x20  0x40
 */
uint8_t time_set[7] = {21, 04, 22, 17, 27, 0, 10};
uint8_t alarm_time_set[4] = {19, 17, 1, 0};

uint8_t alarm_set_time_count = 0;
uint8_t alarm_set_time[40][6] = {0};
uint8_t alarm_set_time_t[6] = {0};
uint8_t alarm_set_time_t2[6] = {0};
uint8_t current_time_t[6] = {0};
uint16_t alarm_flash_count = 0;

void rtc_rx8010_dev_init(void)
{
	g_rtc_dev.timestamp = 0;
	g_rtc_dev.rtc = &rtc_rx;
	g_rtc_dev.rtc_time = &rtc_time;
}

void rtc_rx8010_init()
{
	static int error_count = 0;
	rtc_rx8010_dev_init();
	rx8010_module_init(g_rtc_dev.rtc);
	while(rx8010_check_vlf(g_rtc_dev.rtc) < 0) {
		printf("rtc check error\n");
		if(error_count++ > 50)
			break;
	}
	rx8010_init(g_rtc_dev.rtc);
	printf("RTC_Task\n");
}

void rtc_rx8010_get_time(void)
{
	rx8010_get_time(g_rtc_dev.rtc, g_rtc_dev.rtc_time);
}

int8_t rtc_rx8010_set_alarm(uint8_t *data)
{
	int ret = 0;
	rtc_alarm_t rtc_alarm;
	rtc_alarm.date.day = data[0];
	rtc_alarm.time.hour = data[1];
	rtc_alarm.time.min = data[2];
	rtc_alarm.time.sec = data[3];
	ret = rx8010_set_alarm(g_rtc_dev.rtc, &rtc_alarm);
	if(ret < 0)
	{
		printf("Set rtc_alarm error!\n");
		return ret;
	}
	ret = rx8010_alarm_irq_enable(g_rtc_dev.rtc,1);
	if(ret < 0)
	{
		printf("Set rtc_alarm error!\n");
		return ret;
	}
	return OK;
}

int8_t rtc_rx8010_set_time(uint8_t *data)
{
	int8_t ret = 0;
	uint8_t i;
	rtc_time_t rtc;
	rtc.year = data[0];
	rtc.date.month = data[1];
	rtc.date.day = data[2];
	rtc.time.hour = data[3];
	rtc.time.min = data[4];
	rtc.time.sec = data[5];
	rtc.week = data[6];
	printf("week:%x\n",rtc.week);
	for(i = 0;i < 7;i++)
	{
		if(data[6] == week_analy[i])
			break;
	}
	printf("set time : 20%d-%d-%d  %d:%d:%d   %d\n", rtc.year, rtc.date.month, rtc.date.day, rtc.time.hour, rtc.time.min, rtc.time.sec,i+1);
	ret = rx8010_set_time(g_rtc_dev.rtc, &rtc);
	if (ret < 0) {
		printf("Set Time Error\n");
		return ERROR;
	}
	printf("Set Time OK\n");
	return OK;
}

int8_t rtc_rx8010_sync_time(rtc_time_t *rtc)
{
	int8_t ret = 0;

	printf("set time : %d-%d-%d  %d:%d:%d   %d\n", rtc->year, rtc->date.month, rtc->date.day, rtc->time.hour, rtc->time.min, rtc->time.sec, rtc->week);
	ret = rx8010_set_time(g_rtc_dev.rtc, rtc);
	if (ret < 0) {
		printf("Set Time Error\n");
		return ERROR;
	}
	printf("Set Time OK\n");
	return OK;
}


uint8_t is_leap_year(rtc_time_t *rtc)
{
	if((rtc->year % 400) == 0)
		return 1;
	else if((rtc->year % 100) == 0)
		return 0;
	else if(((rtc->year % 4) == 0))
		return 1;
	
	return 0;
}

uint8_t days_of_month(rtc_time_t *rtc)
{
	if(rtc->date.month != 2)
		return days_of_mon[rtc->date.month -1];
	else 
		return days_of_mon[1] + is_leap_year(rtc);
}

void rtc_rc8010_set_time()
{
	int8_t ret = 0;
		ret = rtc_rx8010_set_time(time_set);
		if(ret < 0)
			printf("Set RTC Time ERROR\n");
}
extern uint8_t test_tmp1;
extern uint8_t test_tmp2;
void rtc_rx8010_task()
{
		int8_t ret = 0;
		uint8_t i = 0;
		uint8_t j = 0;
		uint8_t p = 0,q = 0;
		uint8_t data[8] = {0};
		int flagreg;
		
		static uint16_t time_report_count = 0;
		
//		ret = rtc_rx8010_set_time(time_set);
//		if(ret < 0)
//			printf("Set RTC Time ERROR\n");
//		ret = rtc_rx8010_set_alarm(alarm_time_set);
//		if(ret < 0)
//			printf("Set RTC_ALARM Time ERROR\n");

		if (timer13_flag) {
			timer13_flag = false;

			ret = rx8010_get_time(g_rtc_dev.rtc, g_rtc_dev.rtc_time);
			if (ret < 0)
				printf("RTC Read ERROR!\n");

			for(i = 0;i < 7;i++)
			{
				if(g_rtc_dev.rtc_time->week == week_analy[i])
				{
					if(week_analy[i] == 0x01)
						g_rtc_dev.rtc_time->week = 7;
					else
						g_rtc_dev.rtc_time->week = i+1;
					break;
				}
			}
			data[0] = g_st_reset_dev.mainboard_status;
			msg_robot_status_report(data,1);  //gd32 status
			Iot_sw(g_st_reset_dev.iot_switch_status);
//			printf("-------[%d][%d]\n",g_st_reset_dev.mainboard_status,g_st_reset_dev.iot_switch_status);
			if(g_rtc_dev.rtc_time->date.day > days_of_month(g_rtc_dev.rtc_time))
			{
				g_rtc_dev.rtc_time->date.day = 1;
				g_rtc_dev.rtc_time->date.month += 1;
				if(g_rtc_dev.rtc_time->date.month > 12)
				{
					g_rtc_dev.rtc_time->date.month = 1;
					g_rtc_dev.rtc_time->year += 1;
				}
				time_set[0] = g_rtc_dev.rtc_time->year - 2000;
				time_set[1] = g_rtc_dev.rtc_time->date.month;
				time_set[2] = g_rtc_dev.rtc_time->date.day;
				time_set[3] = g_rtc_dev.rtc_time->time.hour;
				time_set[4] = g_rtc_dev.rtc_time->time.min;
				time_set[5] = g_rtc_dev.rtc_time->time.sec;
				time_set[6] = g_rtc_dev.rtc_time->week;
				rtc_rx8010_set_time(time_set);
			}

//			if(rtc_alarm_set_flag)
//			{
//				for(p = 0;p < alarm_set_time_count;p++)
//				{
//					if((alarm_set_time[p][4] == g_rtc_dev.rtc_time->time.min)&&\
//						(alarm_set_time[p][3] == g_rtc_dev.rtc_time->time.hour)&&\
//						(alarm_set_time[p][2] == g_rtc_dev.rtc_time->date.day)&&\
//						(alarm_set_time[p][1] == g_rtc_dev.rtc_time->date.month)&&\
//						(alarm_set_time[p][0] == g_rtc_dev.rtc_time->year - 2000))
//					{
//						for(q = p;q < alarm_set_time_count-1;q++)
//							for(j = 0;j < 5;j++){
//								alarm_set_time[q][j] = alarm_set_time[q+1][j];
//								alarm_set_time[q+1][j] = 0;
//							}
//						alarm_set_time_count -= 1;
//						printf("rx8010_alarm_time coming.num = %d\n",alarm_set_time_count);
//						if((g_st_reset_dev.mainboard_status == MAINBOARD_OFF)&&(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0))
//						{
//							xHWSwitchDev.xHWSMode = eHWSPwrOn;
//							//prvNbiotDeviceInfoReport();
//							printf("POWER ON_2\n");
//							gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN); 	//main_board_pwr_en
//							delay_ms(1000);
//							gpio_bit_reset(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_en
//							delay_ms(1000); 	
//							gpio_bit_reset(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_en
//							delay_ms(1000);
//							msg_x86_poweron_report(data,1) ;
//							g_st_reset_dev.mainboard_status = MAINBOARD_ON; 								
//						}
//						break;
//					}
//				}
//			}

			if(rtc_alarm_set_flag)
			{
				if((alarm_set_time_t[4] == g_rtc_dev.rtc_time->time.min)&&\
					(alarm_set_time_t[3] == g_rtc_dev.rtc_time->time.hour)&&\
					(alarm_set_time_t[2] == g_rtc_dev.rtc_time->date.day)&&\
					(alarm_set_time_t[1] == g_rtc_dev.rtc_time->date.month)&&\
					(alarm_set_time_t[0] == g_rtc_dev.rtc_time->year - 2000))
				{
					if((g_st_reset_dev.mainboard_status == MAINBOARD_OFF)&&(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0))
					{
						xHWSwitchDev.xHWSMode = eHWSPwrOn;
						//prvNbiotDeviceInfoReport();
						printf("POWER ON_2\n");
						gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN); 	//main_board_pwr_en
						delay_ms(1000);
						gpio_bit_reset(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_en
						delay_ms(1000); 	
						gpio_bit_reset(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_en
						delay_ms(1000);
						msg_x86_poweron_report(data,1) ;
						g_st_reset_dev.mainboard_status = MAINBOARD_ON; 
						g_st_reset_dev.iot_switch_status = MAINBOARD_ON;				
					}
				}
			}
			
			if(rtc_print_count++ > 5)
			{
				rtc_print_count = 0;
				printf("cutime:%d-%d-%d  %d:%d:%d  %d   %d\n",\
				g_rtc_dev.rtc_time->year, g_rtc_dev.rtc_time->date.month, g_rtc_dev.rtc_time->date.day,g_rtc_dev.rtc_time->time.hour,\
				g_rtc_dev.rtc_time->time.min, g_rtc_dev.rtc_time->time.sec, g_rtc_dev.rtc_time->week,alarm_set_time_count);
				printf("xHWSwitchDev.ulGD32ShutdownDelayTime:%d\n",xHWSwitchDev.ulGD32ShutdownDelayTime);
				printf("alarm_set_time_min:20%d-%d-%d %d:%d:0\n",alarm_set_time_t[0],alarm_set_time_t[1],alarm_set_time_t[2],alarm_set_time_t[3],alarm_set_time_t[4]);
			}
			printf("test_tmp1:%d,test_tmp2:%d\n",test_tmp1,test_tmp2);
//			static uint8_t i = 0;
//			if(rtc_alarm_set_flag)
//			{
//				printf("alarm_time:%d-%d-%d  %d:%d:0\n",\
//				alarm_set_time[i][0],alarm_set_time[i][1],alarm_set_time[i][2],alarm_set_time[i][3],alarm_set_time[i][4]);
//				i++;
//				if(i == alarm_set_time_count)
//					i = 0;
//			}
			
			if(time_report_count++ >= 300)
			{
				time_report_count = 0;
				current_time_t[0] = g_rtc_dev.rtc_time->year - 2000;
				current_time_t[1] = g_rtc_dev.rtc_time->date.month;
				current_time_t[2] = g_rtc_dev.rtc_time->date.day;
				current_time_t[3] = g_rtc_dev.rtc_time->time.hour;
				current_time_t[4] = g_rtc_dev.rtc_time->time.min;
				current_time_t[5] = g_rtc_dev.rtc_time->time.sec;
				msg_gd32_current_time_report(current_time_t,6);
				msg_gd32_alarm_time_report(alarm_set_time_t,6);
			}
		}
		
		if(250 <= alarm_flash_count)
			alarm_flash_count = 300;
		else if(200 <= alarm_flash_count)
		{
			alarm_flash_count = 250;
//			write_updateflag(alarm_set_time_t);
		}
		
		if(rtc_time_ask_flag)
		{
			rtc_time_ask_flag = false;
			data[0] = 0;
			data[1] = BIN2BCD(g_rtc_dev.rtc_time->year - 2000);
			data[2] = BIN2BCD(g_rtc_dev.rtc_time->date.month);
			data[3] = BIN2BCD(g_rtc_dev.rtc_time->date.day);
			data[4] = BIN2BCD(g_rtc_dev.rtc_time->time.hour);
			data[5] = BIN2BCD(g_rtc_dev.rtc_time->time.min);
			data[6] = BIN2BCD(g_rtc_dev.rtc_time->time.sec);
			data[7] = BIN2BCD(g_rtc_dev.rtc_time->week);
			msg_rtc_time_report(data,8) ;
		}
		if(rtc_alarm_timeon_ask_flag)
		{
			rtc_alarm_timeon_ask_flag = false;
			rx8010_read_alarm(g_rtc_dev.rtc, g_rtc_dev.rtc_alarm_time);
			data[0] = 0;
			data[1] = BIN2BCD(g_rtc_dev.rtc_alarm_time->date.day);
			data[2] = BIN2BCD(g_rtc_dev.rtc_time->time.hour);
			data[3] = BIN2BCD(g_rtc_dev.rtc_time->time.min);
			data[4] = BIN2BCD(g_rtc_dev.rtc_time->time.sec);
			msg_rtc_alarm_report(data,5,MSG_TYPE_ALARM_POWERON_TIME_GET);
		}
		if(rtc_alarm_timeoff_ask_flag)
		{
			rtc_alarm_timeoff_ask_flag = false;
			rx8010_read_alarm(g_rtc_dev.rtc, g_rtc_dev.rtc_alarm_time);
			data[0] = 0;
			data[1] = BIN2BCD(g_rtc_dev.rtc_alarm_time->date.day);
			data[2] = BIN2BCD(g_rtc_dev.rtc_time->time.hour);
			data[3] = BIN2BCD(g_rtc_dev.rtc_time->time.min);
			data[4] = BIN2BCD(g_rtc_dev.rtc_time->time.sec);
			msg_rtc_alarm_report(data,5,MSG_TYPE_ALARM_POWEROFF_TIME_GET);
		}
}
