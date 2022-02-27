/*
 * nbiot_task.c
 *
 *  Created on: Aug 6, 2019
 *      Author: yxq
 */
#include "nbiot_task.h"
#include "reset.h"
#include "uart.h"
#include "Iot.h"
#include "hardware_switch.h"
#include "systick.h"

NbiotDevice_t xNbiotDev;
IotMsg_t xIotStatusMsg[4];
IotDeviceStatus_t xIotDeviceStatusTemp;
IotSensorStatus_t xIotSensorStatusTemp;
IotFunctionStatus_t xIotFunctionStatusTemp;
IotSystemStatus_t xIotSystemStatusTemp;

int8_t prvNBIotDevInit(NbiotDevice_t *xDev )
{
    xIotStatusMsg[0].ucMessageId = eDeviceStatus;
    xIotStatusMsg[0].ucLen = 7;
    xIotStatusMsg[0].ucStatusChange = 0;
    xIotStatusMsg[1].ucMessageId = eSensorStatus;
    xIotStatusMsg[1].ucLen = 12;
    xIotStatusMsg[1].ucStatusChange = 0;
    xIotStatusMsg[2].ucMessageId = eFunctionStatus;
    xIotStatusMsg[2].ucLen = 5;
    xIotStatusMsg[2].ucStatusChange = 0;
    xIotStatusMsg[3].ucMessageId = eSystemStatus;
    xIotStatusMsg[3].ucLen = 3;
    xIotStatusMsg[3].ucStatusChange = 0;
    memset(xIotStatusMsg[0].u.cData, 0, xIotStatusMsg[0].ucLen);
    memset(xIotStatusMsg[1].u.cData, 0, xIotStatusMsg[1].ucLen);
    memset(xIotStatusMsg[2].u.cData, 0, xIotStatusMsg[2].ucLen);
    memset(xIotStatusMsg[3].u.cData, 0, xIotStatusMsg[3].ucLen);

    memset(&xIotDeviceStatusTemp, 0xFF, 7);
    memset(&xIotSensorStatusTemp, 0xFF, 12);
    memset(&xIotFunctionStatusTemp, 0xFF, 5);
    memset(&xIotSystemStatusTemp, 0xFF, 3);
	
	xDev->ucInitStatus = 0;
	xDev->ucConnectServerFlag = 0;

	return 0;
}


void prvNbiotMsgHandle(char *pcMsg)
{
	uint16_t ucId = 0;
	uint16_t usLen = strlen(pcMsg)/2;
	uint8_t ucTemp = 0;
	uint8_t data[8];
	uint8_t cmdType = 0x06;
	memset(data,0,sizeof(data));
	char *pcNewMsg = malloc(usLen);
	StringToHex(pcMsg, pcNewMsg, &ucTemp);
	if(pcNewMsg[0] != cmdType)
		return;
	ucId = pcNewMsg[1] << 8 | pcNewMsg[2];
	printf("id:%d len:%d\n", ucId, usLen);
	switch(ucId) {
		case ePowerOn:
//			vNbiotDevicePowerStatus(0);
//			delay_ms(500);
			iot_wakeup_enable = true;
			printf("POWERON\n");
			break;
//		case ePowerOff:
//			vNbiotDevicePowerStatus(0);
//			delay_ms(1000);
//			gd32_poweroff_flag = true;
//			printf("POWEROFF\n");
//			break;
//		case eReboot:
//			vNbiotDevicePowerStatus(0);
//			delay_ms(1000);
//			gd32_reset_flag = true;
//			printf("REBOOT\n");
//			break;
		default:
			break;
	}
	free(pcNewMsg);
}


void vNbiotRecvTask(void)
{
	char buffer[128] = {0};
	if (xNbiotDev.ucInitStatus && xNbiotDev.ucConnectServerFlag) {
		if(ATUartInterrupt_recv3_flag)
		{
			ATUartInterrupt_recv3_flag = false;
			printf("\r\n||------New Msg------||\r\n");
			cATCoapRecvData(buffer);
			printf("Buffer:%s\n", buffer);
			prvNbiotMsgHandle(buffer);
			memset(buffer, 0, 128);
		}
	}
	delay_ms(10);
}

int8_t prvNbiotMsgProcess(char *pcData, IotMsg_t *pxMsg)
{
	int i = 0;
	sprintf(pcData,"%02x", pxMsg->ucMessageId);
	for (i = 0; i < pxMsg->ucLen; i++)
		sprintf(&pcData[i*2+2],"%02x", pxMsg->u.cData[i]);
	return 0;
}

void prvNbiotVersionReport(char *pcData)
{
//	int i = 0;
//	char *hv;
//	char *hv_num;
//	hv = NULL;
//	hv_num = NULL;
//	sprintf(hv, "%d" , gHWVer);
//	if(gHWVer < 10)
//		hv_num = "Nimbot - 00";
//	else
//		hv_num = "Nimbot - 0";
//	strcat(hv_num,hv);
//	static char version[128];
//	IotMsg_t xIotMsg;
//	xIotMsg.ucMessageId = eVersion;
//	memset(version, 0x20, 128);
//	memcpy(version, __REVERSION__, strlen(__REVERSION__));
//	memcpy(version+64, hv_num, strlen(hv_num));
//	xIotMsg.ucLen = 128;
////	prvNbiotMsgProcess(pcData, &xIotMsg);
//	sprintf(pcData,"%02x", xIotMsg.ucMessageId);
//	for (i = 0; i < xIotMsg.ucLen; i++)
//		sprintf(&pcData[i*2+2],"%02x", version[i]);
//	cATCoapSendData(pcData, (xIotMsg.ucLen + 1));
}

void prvNbiotDeviceInfoReport()
{
	if(xNbiotDev.ucInitStatus)
	{
		char device_info[30];
		IotMsg_t xIotMsg;
		char data[100];
		int i = 0;
		xIotMsg.ucMessageId = eDeviceInfo;
		memcpy(device_info, xNbiotDev.cSerialNumber, 15);
		memcpy(device_info+15, xNbiotDev.cImsi, 15);
		xIotMsg.ucLen = 30;
		sprintf(data,"%02x", xIotMsg.ucMessageId);
		for (i = 0; i < xIotMsg.ucLen; i++)
			sprintf(&data[i*2+2],"%02x", device_info[i]);
		delay_ms(500);
		cATCoapSendData(data, (xIotMsg.ucLen + 1));
	}
}

void vNbiotDevicePowerStatus(char cStatus)
{
	if(xNbiotDev.ucInitStatus)
	{
		IotMsg_t xIotMsg;
		char data[10];
		int i = 0;
		xIotMsg.ucMessageId = ePowerStatus;
		xIotMsg.ucLen = 1;
		sprintf(data,"%02x", xIotMsg.ucMessageId);
		for (i = 0; i < xIotMsg.ucLen; i++)
			sprintf(&data[i*2+2],"%02x", cStatus);
		cATCoapSendData(data, (xIotMsg.ucLen + 1));
	}
}

void vNbiotSendTask(IotMsg_t xIotMsg)
{
	uint8_t result = ERROR;
	uint8_t print_cnt = 0;
	char data[300];
	delay_ms(200);
	if (xNbiotDev.ucInitStatus && xNbiotDev.ucConnectServerFlag) {
		if (print_cnt++ >= 10) {
			print_cnt = 0;
//				printf("NB Ver:%s\n",xNbiotDev.cVersion);
//				printf("IMEI:%s\n",xNbiotDev.cImei);
//				printf("SN:%s\n",xNbiotDev.cSerialNumber);
//				printf("IMSI:%s\n",xNbiotDev.cImsi);
//				printf("Net:%d\n",xNbiotDev.ucRegisterNetWorkFlag);
//				printf("CSQ:%d\n",xNbiotDev.ucSignalCsq);
		}			
		printf("\r\n--------Report--------\r\n");
		memset(data, 0, 300);
		prvNbiotMsgProcess(data, &xIotMsg);
		cATCoapSendData(data, (xIotMsg.ucLen + 1));
		delay_ms(200);
	}
}

void vNbiotStatusReport(char cId, char *pcData)
{
	int j;
	switch(cId){
		case eDeviceStatus:
		{
			for (j = 0; j < xIotStatusMsg[0].ucLen; j++) {
				if (pcData[j] == 0xFF)
					continue;
				if (xIotStatusMsg[0].u.cData[j] != pcData[j]) { //检测状态是否变化
					xIotStatusMsg[0].u.cData[j] = pcData[j];
					xIotStatusMsg[0].ucStatusChange = 1;
				}
			}
			if (xIotStatusMsg[0].ucStatusChange) { //状态改变后发送
				xIotStatusMsg[0].ucStatusChange = 0;
				vNbiotSendTask(xIotStatusMsg[0]);
			}
			break;
		}
		case eSensorStatus:
		{
			for (j = 0; j < xIotStatusMsg[1].ucLen; j++) {
				if (pcData[j] == 0xFF)
					continue;
				if (xIotStatusMsg[1].u.cData[j] != pcData[j]) { //检测状态是否变化
					xIotStatusMsg[1].u.cData[j] = pcData[j];
					xIotStatusMsg[1].ucStatusChange = 1;
				}
			}
			if (xIotStatusMsg[1].ucStatusChange) { //状态改变后发送
				xIotStatusMsg[1].ucStatusChange = 0;
				vNbiotSendTask(xIotStatusMsg[1]);
			}
			break;
		}
		case eFunctionStatus:
		{
			for (j = 0; j < xIotStatusMsg[2].ucLen; j++) {
				if (pcData[j] == 0xFF)
					continue;
				if (xIotStatusMsg[2].u.cData[j] != pcData[j]) { //检测状态是否变化
					xIotStatusMsg[2].u.cData[j] = pcData[j];
					xIotStatusMsg[2].ucStatusChange = 1;
				}
			}
			if (xIotStatusMsg[2].ucStatusChange) { //状态改变后发送
				xIotStatusMsg[2].ucStatusChange = 0;
				vNbiotSendTask(xIotStatusMsg[2]);
			}
			break;
		}
		case eSystemStatus:
		{
			for (j = 0; j < xIotStatusMsg[3].ucLen; j++) {
				if (pcData[j] == 0xFF)
					continue;
				if (xIotStatusMsg[3].u.cData[j] != pcData[j]) { //检测状态是否变化
					xIotStatusMsg[3].u.cData[j] = pcData[j];
					xIotStatusMsg[3].ucStatusChange = 1;
				}
			}
			if (xIotStatusMsg[3].ucStatusChange) { //状态改变后发送
				xIotStatusMsg[3].ucStatusChange = 0;
				vNbiotSendTask(xIotStatusMsg[3]);
			}
			break;
		}
	}
}

void vNbiotInit()
{
	uint8_t result = ERROR;
	IotMsg_t xIotMsg;
	uint8_t print_cnt = 0;
	char data[300];
	delay_ms(500);
	prvNBIotDevInit(&xNbiotDev);
//	while(cATModuleInit(&xNbiotDev,"180.101.147.115",5683)) {

//	}
//	printf("init:%d net:%d\n",xNbiotDev.ucInitStatus, xNbiotDev.ucConnectServerFlag);
//	delay_ms(500);
//	prvNbiotDeviceInfoReport();
//	delay_ms(500);
}

//int StringToHex(char *str, char *out, uint8_t *outlen)
//{
//	char *p = str;
//	char high = 0, low = 0;
//	int tmplen = strlen(p), cnt = 0;
//	if (str == NULL)
//		return -1;
//	tmplen = strlen(p);
//	while(cnt < (tmplen / 2))
//	{
//		high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
//		low = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
//		out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
//		p ++;
//		cnt ++;
//	}
//	if(tmplen % 2 != 0) out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

//	if(outlen != NULL) *outlen = tmplen / 2 + tmplen % 2;
//	return tmplen / 2 + tmplen % 2;
//}