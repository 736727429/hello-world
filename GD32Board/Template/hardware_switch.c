/*
 * hardware_switch.c
 *
 *  Created on: Sep 10, 2019
 *      Author: yxq
 */
#include "hardware_switch.h"
#include "reset.h"
#include "gd32e230c_eval.h"
#include "global.h"
#include "systick.h"
#include "timer.h"
#include "uart.h"
#include "nbiot_task.h"
#include "at_hand.h"

uint8_t power_on_flag = 0;
uint8_t power_off_num = 0;

#define TASK_DELAY_MS			100		// ms
#define GPIO_READ_FILTER_TIMES	3
#define HW_SWITCH_PRESS_TIME	4		//seconds

HWSwitchDev_t xHWSwitchDev;

uint8_t pressOnCnt = 0;
uint8_t pressOffCnt = 0;

volatile bool gd32_poweroff_flag = false;
volatile bool gd32_lowpower_flag = false;
volatile bool gd32_reset_flag = false;
volatile bool gd32_reset_cmp_flag = false;
volatile bool gd32_charge_protect_flag = false;

void HWS_Init(void)
{
	xHWSwitchDev.ucGpioState = eHWSPwrOff;
	xHWSwitchDev.xHWSMode = eHWSPwrOff;
	xHWSwitchDev.ulSwitchHoldTime = 0;
	xHWSwitchDev.ulSwitchPressTime = 0;
	xHWSwitchDev.ucSwitchPressFlag = 0;
	xHWSwitchDev.ucGD32Shutdown_Charge_Flag = 0;
	xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag = 0;
	xHWSwitchDev.ucShutdown_charge_Flag = 0;
	xHWSwitchDev.ulGD32ShutdownDelayTime = 0;
	xHWSwitchDev.ulDebugReminderTime = 1;
	xHWSwitchDev.ucSwitchPressOffFlag = 1;
}

//????
void HWS_CheckHWSitch()
{
	if(!gpio_input_bit_get(ON_OFF_SW_GPIO,ON_OFF_SW_PIN)) {
		if (pressOnCnt++ >= GPIO_READ_FILTER_TIMES) {
			pressOnCnt = GPIO_READ_FILTER_TIMES;
			pressOffCnt = 0;
			if (xHWSwitchDev.ucSwitchPressOffFlag) {
#if DEBUG_ENABLE
				printf("HWS_CheckHWSitch: switch:%d\n",xHWSwitchDev.ulSwitchHoldTime);
#endif
				if (xHWSwitchDev.ulSwitchHoldTime++ >= ((HW_SWITCH_PRESS_TIME * 1000) / TASK_DELAY_MS)) { 	//press 3s, only check one time until press off
					xHWSwitchDev.ulSwitchHoldTime = (HW_SWITCH_PRESS_TIME / TASK_DELAY_MS);
					xHWSwitchDev.ucSwitchPressOffFlag = 0;
#if DEBUG_ENABLE
					printf("HWS_CheckHWSitch: gpio:%d, mode:%d\n",xHWSwitchDev.ucGpioState,xHWSwitchDev.xHWSMode);
#endif
					if(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0){
						if(xHWSwitchDev.ucGpioState == xHWSwitchDev.xHWSMode) {	//????
							if (xHWSwitchDev.xHWSMode == eHWSPwrOff) {
								xHWSwitchDev.xHWSMode = eHWSPwrOn;
#if DEBUG_ENABLE							
								printf("HWS_CheckHWSitch: POWER ON\n");
#endif							
							}else if (xHWSwitchDev.xHWSMode == eHWSPwrOn){
								xHWSwitchDev.xHWSMode = eHWSPwrOff;
#if DEBUG_ENABLE							
								printf("HWS_CheckHWSitch: POWER OFF\n");
#endif						
							}
						}
					}
				}
			}
		}
	} else {//???????
		if (pressOffCnt++ >= GPIO_READ_FILTER_TIMES) {
			pressOffCnt = GPIO_READ_FILTER_TIMES;
			pressOnCnt = 0;
			xHWSwitchDev.ulSwitchHoldTime = 0;
			xHWSwitchDev.ucSwitchPressOffFlag = 1;
			xHWSwitchDev.ucGpioState = xHWSwitchDev.xHWSMode;
		}
	}

	if ((pressOffCnt >= GPIO_READ_FILTER_TIMES) || (pressOnCnt >= GPIO_READ_FILTER_TIMES)) {
		PM_GetPowerState(xHWSwitchDev.xHWSMode, xHWSwitchDev.charge_status);
	}
}

void hardware_switch_task()
{
	uint8_t data[1] = {0};
	data[0] = 30;
	if(xHWSwitchDev.ucGpioState != xHWSwitchDev.xHWSMode) {	//????????
		xHWSwitchDev.ucGpioState = xHWSwitchDev.xHWSMode;	//update Gpio state
		PM_GetPowerState(xHWSwitchDev.xHWSMode, xHWSwitchDev.charge_status); //update power mode state
			switch(xHWSwitchDev.PowerState) {
				case PWR_OFF_NOCHARGE:
					printf("POWER OFF NO CHARGE1\n");
					g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
					msg_robot_pwroff_report(data,1) ;
					xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag = 1;
					xHWSwitchDev.ulGD32ShutdownDelayTime = 35;
					break;
				case PWR_OFF_CHARGE:
					printf("POWER OFF IN CHARGE\n");
					g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
					msg_robot_pwroff_report(data,1) ;
					xHWSwitchDev.ucGD32Shutdown_Charge_Flag = 1;
					xHWSwitchDev.ucShutdown_charge_Flag = 1;
					xHWSwitchDev.ulGD32ShutdownDelayTime = 35;
					break;
				case PWR_ON_NOCHARGE:
					printf("POWER ON NO CHARGE\n");
					//prvNbiotDeviceInfoReport();
					gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_en
					delay_ms(1000);
					gpio_bit_reset(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_en
					delay_ms(1000);
					gpio_bit_reset(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_en
					delay_ms(1000);
					msg_x86_poweron_report(data,1) ;
					g_st_reset_dev.mainboard_status = MAINBOARD_ON;
					g_st_reset_dev.iot_switch_status = MAINBOARD_ON;
//				  rt1020_uart_init();
					break;
				case PWR_ON_CHARGE:
					printf("POWER ON IN CHARGE\n");
					//prvNbiotDeviceInfoReport();
					xHWSwitchDev.ucShutdown_charge_Flag = 0;
					gpio_bit_reset(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_en
					delay_ms(1000);
					msg_x86_poweron_report(data,1) ;
					g_st_reset_dev.mainboard_status = MAINBOARD_ON;
					g_st_reset_dev.iot_switch_status = MAINBOARD_ON;
					break;
				default:
					printf("UNDEFINE POWER MODE\n");
					break;				
			}
	}
	//gd32 receive poweroff cmd
	if(gd32_poweroff_flag)
	{
		gd32_poweroff_flag = false;
		g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
		xHWSwitchDev.xHWSMode = eHWSPwrOff;
		PM_GetPowerState(xHWSwitchDev.xHWSMode, xHWSwitchDev.charge_status); //update power mode state
		switch(xHWSwitchDev.PowerState) {
			case PWR_OFF_NOCHARGE:
				printf("POWER OFF NO CHARGE2\n");
				g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
				msg_robot_pwroff_report(data,1) ;
				xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag = 1;
				xHWSwitchDev.ulGD32ShutdownDelayTime = 35;
//				gd_eval_com_to_gpio(RT1020_COM_ID);
				break;
			case PWR_OFF_CHARGE:
				printf("POWER OFF IN CHARGE\n");
				g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
				msg_robot_pwroff_report(data,1) ;
				xHWSwitchDev.ucGD32Shutdown_Charge_Flag = 1;
				xHWSwitchDev.ucShutdown_charge_Flag = 1;
				xHWSwitchDev.ulGD32ShutdownDelayTime = 35;
				break;
			default:
				printf("UNDEFINE POWER MODE\n");
				break;				
		}
	}
	//gd32 receive reset cmd
	if(gd32_reset_flag)
	{
		gd32_reset_flag = false;
		g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
		msg_robot_pwroff_report(data,1) ;
		xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag = 1;
		xHWSwitchDev.ulGD32ShutdownDelayTime = 35;
		gd32_reset_cmp_flag = true;
	}
	//GD32ÑÓÊ±¹Ø»ú
	if (xHWSwitchDev.ucGD32Shutdown_Charge_Flag) {
		init_state = 0;
		if (xHWSwitchDev.ulGD32ShutdownDelayTime < 5) {	//30S????
			printf("GD32 Shutdown when charge!!\n");
			g_st_reset_dev.iot_switch_status = MAINBOARD_OFF;			
			xHWSwitchDev.ucGD32Shutdown_Charge_Flag = 0;
			xHWSwitchDev.ulGD32ShutdownDelayTime = 0;
			gpio_bit_set(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_dis
			delay_ms(1000);
		}
	}
	if (xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag) {
		init_state = 0;
		if (xHWSwitchDev.ulGD32ShutdownDelayTime < 5) {	//30S????
			printf("GD32 Shutdown when nocharge!!\n");			
			if(X86ECDET == 0)
			{
				xHWSwitchDev.ulGD32ShutdownDelayTime = 5;
				printf("GD32 Shutdown when nocharge failed!!!\n");
				return;
			}
			g_st_reset_dev.iot_switch_status = MAINBOARD_OFF;
			xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag = 0;
			xHWSwitchDev.ulGD32ShutdownDelayTime = 0;
			gpio_bit_set(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_dis
			delay_ms(1000);
			gpio_bit_set(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_dis
			delay_ms(1000);
			gpio_bit_reset(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_dis
			delay_ms(1000);
//			pmu_to_sleepmode(WFI_CMD);   //µÍ¹ŠºÄÄ£Êœ
			if(gd32_reset_cmp_flag)
			{
				gd32_reset_cmp_flag = false;
				//prvNbiotDeviceInfoReport();
				gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_en
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
}
uint8_t test_tmp1 = 0;
uint8_t test_tmp2 = 0;
void RT1020_charge()
{
	static uint8_t count1 = 0,count2 = 0,count3 = 0;
	uint8_t data[8] = {0};
	if((g_st_reset_dev.mainboard_status == MAINBOARD_OFF)&&(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0))
	{
		if(xHWSwitchDev.charge_status == CHARGE)//charge
		{
			count1++;
			if(count1 == 1)
			{
				test_tmp1 = 1;
				gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_en
			}
			else if(count1 == 2)
			{
				test_tmp2 = 1;
				gpio_bit_reset(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_en
			}
			else
			{
				count1 = 2;					
				xHWSwitchDev.ucShutdown_charge_Flag = 1;
			}
		}		
		if(xHWSwitchDev.ucShutdown_charge_Flag)
		{
			if((xHWSwitchDev.charge_status == NOCHARGE)&&(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0))
			{
				count1 = 0;
				count2++;
				if(count2 == 1)
				{
					if(X86ECDET == 0)
					{
						count2 = 0;
						return;
					}
					gpio_bit_set(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_dis
				}				
				else if(count2 == 2)
				{
					count2 = 0;
					xHWSwitchDev.ucShutdown_charge_Flag = 0;
					gpio_bit_reset(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_dis
//					pmu_to_sleepmode(WFI_CMD);   //µÍ¹ŠºÄÄ£Êœ			
					test_tmp1 = test_tmp2 = 0;
				}						
			}
		}	
	}
	else
	{
		count1 = 0;count2 = 0;count3 = 0;
	}
}

void RT1020_charge_status()
{
	static uint8_t count = 0;
	if(!gpio_input_bit_get(GD_ADP_DET_GPIO,GD_ADP_DET_PIN))//charge
	{
		if(count++ > 3){//charge
			count = 0;
			xHWSwitchDev.charge_status = CHARGE;
		}
	}
	else
		xHWSwitchDev.charge_status = NOCHARGE;
}

void PM_GetPowerState(uint8_t ucHWSwitch, uint8_t ucCharging)
{
    uint8_t state;
    state = ucHWSwitch + (ucCharging * 2);
    switch(state) {
        case 0: xHWSwitchDev.PowerState = PWR_ON_NOCHARGE; break;
        case 1: xHWSwitchDev.PowerState = PWR_OFF_NOCHARGE; break;
        case 2: xHWSwitchDev.PowerState = PWR_ON_CHARGE; break;
        case 3: xHWSwitchDev.PowerState = PWR_OFF_CHARGE; break;
        default:break;
    }
}
