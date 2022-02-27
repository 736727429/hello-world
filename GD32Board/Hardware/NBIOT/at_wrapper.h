/*
 * at_wrapper.h
 *
 *  Created on: Aug 9, 2019
 *      Author: yxq
 */

#ifndef NBIOT_AT_WRAPPER_H_
#define NBIOT_AT_WRAPPER_H_

#include "global.h"

#define  NBIOTAT_UART_BUF_MAX   200

typedef struct xNbiotDevice
{
	uint8_t ucInitStatus ;				//初始化NB模块的状态
	uint8_t ucSignalCsq ;				//NB模块的信号强度
	char cImei[16];						//IMEI卡号
	char cSerialNumber[16];				//序列号
	char cImsi[16];						//IMSI卡号
	char cVersion[16];					//版本号
	uint8_t ucRegisterNetWorkFlag ;		//注网标志位
	uint8_t ucConnectServerFlag ;		//服务器连接标志位
} NbiotDevice_t;

extern char cNbiotRxData[NBIOTAT_UART_BUF_MAX];

extern volatile bool ATUartInterrupt_recv1_flag;
extern volatile bool ATUartInterrupt_recv2_flag;
extern volatile bool ATUartInterrupt_recv3_flag;

void vATUartInterruptHandle(uint8_t ucData);
int32_t lATUartInit();
int32_t lATUartSend(const void *pvData, uint32_t ulSize, uint32_t ulTimeout);
int32_t lATUartRecv(void *pvData, uint32_t ulExceptSize, uint32_t *pulRecvSize, uint32_t ulTimeout);
void vATUartBufClear(void);
void vATDelay(uint32_t ulMs);

#endif /* NBIOT_AT_WRAPPER_H_ */
