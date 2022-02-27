/*!    \file    main.c    \brief   led spark with systick, USART print and key example        \version 2018-06-19, V1.0.0, firmware for GD32E230*//*    Copyright (c) 2018, GigaDevice Semiconductor Inc.    All rights reserved.    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:    1. Redistributions of source code must retain the above copyright notice, this        list of conditions and the following disclaimer.    2. Redistributions in binary form must reproduce the above copyright notice,        this list of conditions and the following disclaimer in the documentation        and/or other materials provided with the distribution.    3. Neither the name of the copyright holder nor the names of its contributors        may be used to endorse or promote products derived from this software without        specific prior written permission.    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/#include "gd32e230.h"#include "systick.h"#include <stdio.h>#include <stdbool.h>#include "main.h"#include "gd32e230c_eval.h"#include "gd32e230_fwdgt.h"#include "gd32e230_rcu.h"#include "global.h"#include "timer.h"#include "reset.h"#include "hardware_switch.h"#include "rtc_task.h"#include "uart.h"#include "Iot.h"#include "nbiot_task.h"#include "at_hand.h"#include "updateflag.h"#include "bsp.h"#define BOARD_NAME "GD32-PowerCtrlBoard"volatile uint8_t gHWVer = 5;char gd32_version[30] = "V104R103C000M01B009";/*!    \brief      toggle the led every 500ms    \param[in]  none    \param[out] none    \retval     none*/void led_spark(void){    static __IO uint32_t timingdelaylocal = 0U;    if(timingdelaylocal){        if(timingdelaylocal < 1000U){            gpio_bit_reset(GD_USER_LED_GPIO,GD_USER_LED_PIN);        }else{            gpio_bit_set(GD_USER_LED_GPIO,GD_USER_LED_PIN);        }        timingdelaylocal--;    }else{        timingdelaylocal = 2000U;		iot_setstep_flag = true;    }}void power_init(){	  /* enable the GPIO clock */    rcu_periph_clock_enable(RCU_GPIOA);	rcu_periph_clock_enable(RCU_GPIOB);    /* configure power GPIO port */ 	gpio_mode_set(MAIN_PWR_CTRL_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MAIN_PWR_CTRL_PIN);	gpio_mode_set(X86_12V_EN_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, X86_12V_EN_PIN);	gpio_mode_set(EXT_12V_EN_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, EXT_12V_EN_PIN);	gpio_mode_set(GD_IOT_SW_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GD_IOT_SW_PIN);	gpio_mode_set(GD_WAKEUP_NB_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GD_WAKEUP_NB_PIN);	gpio_output_options_set(MAIN_PWR_CTRL_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, MAIN_PWR_CTRL_PIN);	gpio_output_options_set(X86_12V_EN_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, X86_12V_EN_PIN);	gpio_output_options_set(EXT_12V_EN_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, EXT_12V_EN_PIN);	gpio_output_options_set(GD_IOT_SW_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GD_IOT_SW_PIN);	gpio_output_options_set(GD_WAKEUP_NB_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GD_WAKEUP_NB_PIN);	gpio_bit_reset(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);	gpio_bit_reset(GD_IOT_SW_GPIO,GD_IOT_SW_PIN);  //reset-MAINBOARD_OFF   set-MAINBOARD_ON	gpio_bit_set(X86_12V_EN_GPIO,X86_12V_EN_PIN);	gpio_bit_set(EXT_12V_EN_GPIO,EXT_12V_EN_PIN);	gpio_bit_set(GD_WAKEUP_NB_GPIO,GD_WAKEUP_NB_PIN);		    /* configure ON/OFF sw GPIO port */     gpio_mode_set(ON_OFF_SW_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ON_OFF_SW_PIN);	/* configure charge GPIO port */ 	gpio_mode_set(GD_ADP_DET_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GD_ADP_DET_PIN);	/* configure GD_RTC_IRQ GPIO port */ 	gpio_mode_set(GD_RTC_IRQ_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GD_RTC_IRQ_PIN);	/* configure user_led GPIO port */ 	gpio_mode_set(GD_USER_LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GD_USER_LED_PIN);	gpio_output_options_set(GD_USER_LED_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GD_USER_LED_PIN);	gpio_bit_reset(GD_USER_LED_GPIO,GD_USER_LED_PIN);}/*!    \brief      main function    \param[in]  none    \param[out] none    \retval     none*/int main(void){	uint8_t data[8] = {0};	uint8_t temp_num=0;    /* configure systick */    systick_config();			    /* initilize the LEDs, USART and key */    gd_eval_com_init(EVAL_COM,EVAL_COM_ID);		  /* print out the clock frequency of system, AHB, APB1 and APB2 */    printf("\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));    printf("\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));    printf("\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));    printf("\r\nCK_APB2 is %d\r\n", rcu_clock_freq_get(CK_APB2));		BSP_init();	ReadID();	while(1){		test();		//twinkle();	}}/* retarget the C library printf function to the USART */int fputc(int ch, FILE *f){    usart_data_transmit(EVAL_COM, (uint8_t)ch);    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));    return ch;}