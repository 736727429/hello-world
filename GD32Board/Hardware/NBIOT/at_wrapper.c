/*
 * at_wrapper.c
 *
 *  Created on: Aug 9, 2019
 *      Author: yxq
 */

#include "at_wrapper.h"
#include "uart.h"
#include "systick.h"
#include "at_hand.h"

char cNbiotTxData[NBIOTAT_UART_BUF_MAX];
char cNbiotRxData[NBIOTAT_UART_BUF_MAX];
char cTempBuffer[128];
uint8_t ucTempPos = 0;
extern NbiotDevice_t xNbiotDev;

volatile bool ATUartInterrupt_recv1_flag = false;
volatile bool ATUartInterrupt_recv2_flag = false;
volatile bool ATUartInterrupt_recv3_flag = false;

void vATUartInterruptHandle(uint8_t ucData)
{
	cTempBuffer[ucTempPos++] = ucData;
	iot_recv_count++;
	if (ucTempPos >= 2 && cTempBuffer[ucTempPos-1] == '\n' && cTempBuffer[ucTempPos-2] == '\r') { //接收成功
		//printf("\ncTempBuffer:%s\n", cTempBuffer);
		if (strstr((const char*)cTempBuffer, "OK") != NULL) {
			if (strstr((const char*)cTempBuffer, "+QLWEVTIND:0") != NULL || strstr((const char*)cTempBuffer, "+QLWEVTIND:3") != NULL) {
				xNbiotDev.ucConnectServerFlag = 1;
			}
			memcpy(cNbiotRxData, cTempBuffer, ucTempPos);
			memset(cTempBuffer, 0, ucTempPos);
			ucTempPos = 0;
			ATUartInterrupt_recv1_flag = true;
		} else if (strstr((const char*)cTempBuffer, "ERROR") != NULL) {
			memcpy(cNbiotRxData, cTempBuffer, ucTempPos);
			memset(cTempBuffer, 0, ucTempPos);
			ucTempPos = 0;
			ATUartInterrupt_recv2_flag = true;
		} else if (strstr((const char*)cTempBuffer, "+NNMI:") != NULL) {
			memcpy(cNbiotRxData, cTempBuffer, ucTempPos);
			memset(cTempBuffer, 0, ucTempPos);
			ucTempPos = 0;
			ATUartInterrupt_recv3_flag = true;
		} else if (strstr((const char*)cTempBuffer, "+QLWEVTIND:0") != NULL || strstr((const char*)cTempBuffer, "+QLWEVTIND:3") != NULL) {
			xNbiotDev.ucConnectServerFlag = 1;
			memset(cTempBuffer, 0, ucTempPos);
			ucTempPos = 0;
		}
	} else {
		if (ucTempPos > 50) {
			memset(cTempBuffer, 0, ucTempPos);
			ucTempPos = 0;
		}
	}
}


int32_t lATUartInit()
{
    int retval;
#ifndef DMA
    retval = gd_eval_com_init(EVAL_COM,EVAL_COM_ID);
    if(OK != retval) {
        return ERROR;
    }
    return OK;
#else

#endif
}


int32_t lATUartSend(const void *pvData, uint32_t ulSize, uint32_t ulTimeout)
{
	int32_t ret = 0;
#ifndef DMA
	ret = uart_primary_write_noblock(EVAL_COM, pvData, ulSize);

	return ret;
#else

#endif
}


int32_t lATUartRecv(void *pvData, uint32_t ulExceptSize, uint32_t *pulRecvSize, uint32_t ulTimeout)
{
	int32_t ret = 0;
#ifndef DMA
	ret = uart_primary_read_block(EVAL_COM, pvData, ulExceptSize, ulTimeout);
	*pulRecvSize = ret;
	return ret;
#else

#endif
}


void vATUartBufClear(void)
{
	memset(g_st_uart[0].read_buf, 0, 128);
	g_st_uart[0].read_index = g_st_uart[0].write_index;
}


void vATDelay(uint32_t ulMs)
{
	delay_ms(ulMs);
}

