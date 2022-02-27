/*!
    \file    gd32e230_it.c
    \brief   interrupt service routines
    
    \version 2018-06-19, V1.0.0, firmware for GD32E230
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e230_it.h"
#include "main.h"
#include "systick.h"
#include "timer.h"
#include "reset.h"
#include "hardware_switch.h"
#include "uart.h"
#include "gd32e230c_eval.h"
#include "Iot.h"
#include "at_wrapper.h"
#include "at_hand.h"
#include "rtc_task.h"


uint8_t timer2_count = 0;
uint8_t timer13_count = 0;
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    led_spark();
    delay_decrement();
}
/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART0_IRQHandler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        /* receive data */
//        receiver_buffer[rxcount++] = usart_data_receive(EVAL_COM);
//        if(rxcount == receivesize){
//            usart_interrupt_disable(EVAL_COM, USART_INT_RBNE);
//        }
			iot_recv_count++;
			vATUartInterruptHandle((uint8_t)usart_data_receive(EVAL_COM));
    }

    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){
        /* transmit data */
//        usart_data_transmit(EVAL_COM, transmitter_buffer[txcount++]);
//        if(txcount == transfersize){
//            usart_interrupt_disable(EVAL_COM, USART_INT_TBE);
//        }
    }
}
/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler(void)
{
	  uint16_t    tmp;
	
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)){
        g_st_uart[1].read_buf[g_st_uart[1].write_index] = (uint8_t)usart_data_receive(RT1020_COM);
        g_st_uart[1].write_index++;

        if(256 <= g_st_uart[1].write_index) {
            g_st_uart[1].write_index = 0;
        }

//        if(0 == g_st_uart[1].write_index) {
//            tmp = 127;
//        } else {
//            tmp = g_st_uart[1].write_index - 1;
//        }
//        if(tmp == g_st_uart[1].read_index) {

//        }
//					printf("--%x",usart_data_receive(RT1020_COM));
    }

    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_TBE)){
        /* transmit data */
//        usart_data_transmit(EVAL_COM, transmitter_buffer[txcount++]);
//        if(txcount == transfersize){
//            usart_interrupt_disable(EVAL_COM, USART_INT_TBE);
//        }
    }
}

/**
  * @brief  This function handles TIMER2 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER2_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_CH0);
        /* toggle selected led */
				if(timer2_count++ == 10)
				{
						timer2_count = 0;
					  timer2_flag = true;
				}
    }
}

/**
  * @brief  This function handles TIMER13 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER13_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER13, TIMER_INT_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER13, TIMER_INT_CH0);
        /* toggle selected led */
		if(timer13_count++ == 10)//1000ms
		{
//					check_hard_switch();
			timer13_count = 0;
			timer13_flag = true;
			RT1020_charge();
			if(xHWSwitchDev.ucGD32Shutdown_Charge_Flag || xHWSwitchDev.ucGD32Shutdown_Nocharge_Flag) {//??????
				xHWSwitchDev.ulGD32ShutdownDelayTime--;//???30s
			}
		}
		if(gd32_reset_rcu_count++ == 100)
		{
			gd32_reset_rcu_count = 0;
			gd32_reset_rcu_flag = true;
		}
		if(rtc_alarm_set_flag)
			alarm_flash_count++;
		else
			alarm_flash_count = 0;
		RT1020_charge_status();
		HWS_CheckHWSitch();//100ms
    }
}

/*!
    \brief      this function handles external lines 2 to 3 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI2_3_IRQHandler(void)
{
//    if (RESET != exti_interrupt_flag_get(EXTI_2)) {
//		iot_wakeup_enable = true;
//        exti_interrupt_flag_clear(EXTI_2);
//    }
}