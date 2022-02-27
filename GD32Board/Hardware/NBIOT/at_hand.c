/*
 * AT.c
 *
 *  Created on: Aug 6, 2019
 *      Author: yxq
 */
#include "at_hand.h"
#include "systick.h"
#include "uart.h"

/*****************************      RAM      *****************************/
extern NbiotDevice_t xNbiotDev;

uint16_t iot_recv_count = 0;
volatile bool iot_setok_from_rt1020 = false;
volatile bool iot_setstep_flag = false;

/*****************************      function      *****************************/
int8_t prvATSendCmdBlock(char *cmd, const char *reply1, const char *reply2, int timeout)
{
    bool status = false ;
	timeout = timeout * 1500;
//	printf("timeout:%d\r\n",timeout);
#ifdef DEBUG
	printf("------------%s", cmd);
#endif
	memset(cNbiotRxData, 0, sizeof((const char*)cNbiotRxData));
	//uart send data
	//lATUartSend(cmd, strlen(cmd), 5);
	printf(cmd);
	//wait
	if (reply1 == 0)
		return -1;
	else {
		while(!ATUartInterrupt_recv1_flag)
		{
			//printf("iot_recv_count:%d\r\n",iot_recv_count);
			timeout--;
			if(timeout < 0)
			{
				printf("error.\r\n");
				return -1;
			}
		}
		ATUartInterrupt_recv1_flag = false;
	}
	//recv
#ifdef NBIOT_DEBUG
	printf("RECV:%s\n", (const char*)cNbiotRxData);
#endif
	//check input reply
	if(strstr((const char*)(const char*)cNbiotRxData, reply2) != NULL)
	{
#ifdef DEBUG
		printf("ackerror:%s\n", reply2);
#endif
		return -1 ;
	}
	if(strstr((const char*)(const char*)cNbiotRxData, reply1) != NULL)
	{
//		printf("acksuccess:%s\n", (const char*)(const char*)cNbiotRxData);
		//如果匹配到断连标志，就重新建立连接
		if(strstr((const char*)cNbiotRxData, "+NSOCLI: 1") != NULL)
		{
			//status = NB_Create_TCP("120.78.136.134", 9002);
			if(false == status)
			{
				//printf("重新创建TCP连接失败\n");
				return -1 ;
			}
			else
			{
				//printf("重新创建TCP连接成功\n");
			}
		}
		return 0;
	} 
	else {
			//printf("error.\r\n");
		return -1;
	}
}

void prvATSendCmdNonblock(char *cmd)
{
#ifndef DEBUG
	printf("%s", cmd);
#endif
	lATUartSend(cmd, strlen(cmd), 5);
}


int8_t prvATCheckHand(void)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_CHECK_STATE, "OK", NULL, 10000);
	return ret;
}

int8_t prvATSetATE(void)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_SET_ATE, "OK", NULL, 10000);
	return ret;
}

int8_t prvATGetVer(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	char *p = NULL;
	ret = prvATSendCmdBlock(NBIOT_CHECK_VER, "OK", NULL, 10000);
	if (ret == 0) {
		if ((p = strstr((const char*)cNbiotRxData, "Revision")) != NULL) {
			memcpy(pxDev->cVersion, p + 9, 16);
			//printf("NBIOT Version:%s\n", pxDev->cVersion);
		}
	}
	return ret;
}

int8_t prvATOpenErrorCheck(void)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_OPENUE, "OK", NULL, 10000);
	return ret;
}   

int8_t prvATSetBand(uint8_t ucBand)
{
	int8_t ret = 0;
	char cmd[15];
	sprintf(cmd, "AT+NBAND=%d\r\n", ucBand);
	ret = prvATSendCmdBlock(cmd, "OK", NULL, 10000);
	return ret;
}

//获取IMEI
int8_t prvATGetIMEI(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	char *p = NULL;
	ret = prvATSendCmdBlock(NBIOT_CHECK_IMEI, "OK", NULL, 10000);
	if (ret < 0) {
		return ret;
	} else {
		if ((p = strstr((const char*)cNbiotRxData, "+CGSN")) != NULL) {
			memcpy(pxDev->cImei, p + 6, 15);
			//printf("IMEI:%s\n",pxDev->cImei);
		}
		return ret;
	}
}

//获取IMEISV
int8_t prvATGetIMEISV(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+CGSN=0\r\n", "OK", NULL, 10000);
	if (ret < 0) {
		return ret;
	} else {
		memcpy(pxDev->cSerialNumber, (const char*)cNbiotRxData+2, 15);
		//printf("SerialNumber:%s\n",pxDev->cSerialNumber);
		return ret;
	}
}

int8_t cATDisregister(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+QLWSREGIND=1\r\n", "OK", NULL, 10000);
	if (ret < 0) {
		return ret;
	}
	pxDev->ucConnectServerFlag = 0;
	return 0;
}

int8_t prvATRegister()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+QLWSREGIND=0\r\n", "OK", NULL, 10000);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

int8_t prvATOpenLed()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+QLEDMODE=1\r\n", "OK", NULL, 10000);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

//获取SIM卡IMSI
int8_t prvATGetIMSI(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	char *p = NULL;
	ret = prvATSendCmdBlock(NBIOT_CHECK_IMSI, "OK", NULL, 1000);
	if (ret < 0) {
		return ret;
	} else {
		if ((p = strstr((const char*)cNbiotRxData, "46")) != NULL) {
			memcpy(pxDev->cImsi, p, 15);
			//printf("IMSI:%s\n", pxDev->cImsi);
		}
		return ret;
	}
}

//检查网络注册状态
int8_t cATGetNetStatus(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	char buffer[30] = {0};
	ret = prvATSendCmdBlock(NBIOT_CHECK_NET_STATUS,"OK", "ERROR", 10000);
	if (ret < 0) {
		//printf("1111111111111111111111\r\n");
		return ret;
	} else {
		char *p = strstr((const char*)cNbiotRxData, "+CGATT");
		memcpy(buffer, p + 7, 1);
		//printf("buffer:%s %s", buffer, p);
		pxDev->ucRegisterNetWorkFlag = atoi(buffer);
		//printf("Net Status:%d\n", pxDev->ucRegisterNetWorkFlag);
		if (pxDev->ucRegisterNetWorkFlag == 1) {
			//printf("2222222222222222222222\r\n");
			return 0;
		} else {
			//printf("333333333333333333333333\r\n");
			return -1;
		}
	}
}

//注册网络
int8_t prvATRegister_Net(void)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_REGISTER_NET,"OK", "ERROR", 10000);
	return ret;
}

//检查信号强度
int8_t prvATGetCSQ(NbiotDevice_t *pxDev)
{
	int8_t ret = 0;
	char buffer[50] = {0};
	ret = prvATSendCmdBlock(NBIOT_CHECK_CSQ,"+CSQ", NULL, 1000);
	if (ret < 0) {
		return ret;
	} else {
		char *p = strstr((const char*)cNbiotRxData, "+CSQ");
		memcpy(buffer, p + 5, 2);
		pxDev->ucSignalCsq = atoi(buffer);
		if (pxDev->ucSignalCsq != 99) {
			return 0;
		} else {
			return -1;
		}
	}
}

//创建TCP Socket
int8_t prvATCreateTCP(const char *ip, uint32_t port)
{
	int8_t ret = 0;
	char buffer[50] = {0};
	//创建TCP Socket
	ret = prvATSendCmdBlock(NBIOT_CREATE_SOCKET,"OK", NULL, 10000);
	if (ret == 0) {
		sprintf(buffer, "AT+NSOCO=1,%s,%d\r\n", ip, port);
		ret = prvATSendCmdBlock(buffer,"OK", NULL, 10000);
	}
	return ret;
}

//关闭TCP Socket
int8_t prvATCloseTCP()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_CLOSE_SOCKET,"OK", NULL, 10000);
	return ret;
}

//发送数据到服务器
int8_t prvATSendToServer(char *data, int32_t len)
{
	int8_t ret = 0;
	char buffer[512] = {0};
	sprintf(buffer, "AT+NSOSD=1,%d,%s\r\n", len, data);
	//printf("NBIOT Send To Server: %s", buffer);
	ret = prvATSendCmdBlock(buffer,"OK", "ERROR", 10000);
	return ret;
}

//模块复位
int8_t prvATReboot(void)
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_REBOOT, "OK", NULL, 10000);
	return ret;
}

int8_t prvATCloseCFUN()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_CLOSE_CFUN, "OK", NULL, 10000);
	return ret;
}

int8_t prvATOpenCFUN()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock(NBIOT_OPEN_CFUN, "OK", NULL, 10000);
	return ret;
}

int8_t prvATSetCoap(char *ip, int32_t port)
{
	int8_t ret = 0;
	char buffer[50] = {0};
	sprintf(buffer,"AT+NCDP=%s,%d\r\n", ip, port);
	//printf("Set Coap: %s", buffer);
	ret = prvATSendCmdBlock(buffer, "OK", NULL, 10000);
	return ret;
}

int8_t prvATOpenNNMI()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+NNMI=1\r\n", "OK", NULL, 10000);
	return ret;
}

int8_t prvATDisablePSM()
{
	int8_t ret = 0;
	ret = prvATSendCmdBlock("AT+CPSMS=0,,,11100011,11100011\r\n", "OK",NULL, 10000);
	return ret;
}

int8_t cATCoapSendData(char *data, uint16_t len)
{
	int8_t ret = 0;
	char buffer[300] = {0};
	sprintf(buffer, "AT+NMGS=%d,%s\r\n", len, data);
	//printf("Send Data:%s", buffer);
	ret = prvATSendCmdBlock(buffer, "OK", NULL, 10000);
	return ret;
}

int8_t cATCoapRecvData(char *data)
{
	int8_t ret = 0;
	char *pcIndex = NULL;
	char len[3] = {0};
	uint8_t ucTmp = 0;
	uint16_t length = 0;
	if ((pcIndex = strstr((const char*)cNbiotRxData, "+NNMI")) != NULL) {//+NNMI:3,01001E\r\n
		pcIndex = strstr((const char*)cNbiotRxData, ",");//,01001E\r\n
		ucTmp = pcIndex - (const char*)cNbiotRxData - 8;
		//printf("ucTmp:%d\r\n",ucTmp);
		memcpy(len, (const char*)cNbiotRxData+8, ucTmp);
		//printf("p:%s, buffer:%s, len:%s\n", pcIndex, (const char*)cNbiotRxData, len);
//		length = len[ucTmp - 1] - '0';
//		memcpy(data, (const char*)cNbiotRxData+10, length*2);
		length = atoi(len);
		memcpy(data, (const char*)pcIndex+1, length*2);
		memset(cNbiotRxData, 0, NBIOTAT_UART_BUF_MAX);
	}
	//printf("recv:%s %d %d\n", data, cNbiotRxData[8], cNbiotRxData[8]);
	return ret;
}


int8_t init_state = 0;
int8_t error_cnt = 0;
int8_t check_cnt = 0;
int8_t init_ret = 0;
//模块初始化
int8_t cATModuleInit(NbiotDevice_t *pxDev, char *ip, uint32_t port)
{
	uint8_t data[8] = {0};

	if(!pxDev->ucInitStatus) {
		switch (init_state) {
		case Check_Hand:
			if(prvATCheckHand() == 0) {
#ifdef NBIOT_DEBUG
				printf("step1.\r\n");
#endif
				error_cnt = 0;
				init_state = Set_CDP_Service;
			} else {
				error_cnt += 1;
			}
			break;
		case Check_Ver:
			if(prvATGetVer(pxDev) == 0) {
				error_cnt = 0;
				init_state = Check_CSQ;
			} else {
				error_cnt += 1;
			}
			break;
		case Check_CSQ:
			if(prvATGetCSQ(pxDev) == 0) {
				error_cnt = 0;
				init_state = Open_CFUN;
			} else {
				error_cnt += 1;
			}
			break;
		case Check_IMEI:
			if(prvATGetIMEI(pxDev) == 0) {
#ifdef NBIOT_DEBUG
				printf("step5.\r\n");
#endif
				error_cnt = 0;
			} else {
				error_cnt += 1;
			}
			if(prvATGetIMEISV(pxDev) == 0) {
#ifdef NBIOT_DEBUG
				printf("step6.\r\n");
#endif
				error_cnt = 0;
				init_state = Check_SIM;
			} else {
				error_cnt += 1;
			}
			break;
		case Check_SIM:
			if (prvATGetIMSI(pxDev) == 0) {
#ifdef NBIOT_DEBUG
				printf("step7.\r\n");
#endif
				error_cnt = 0;
				init_state = Check_Ver;
			} else {
				error_cnt += 1;
			}
			break;
		case Set_CDP_Service:
			if(prvATSetCoap(ip, port) == 0) {
#ifdef NBIOT_DEBUG
				printf("step2.\r\n");
#endif
				error_cnt = 0;
				init_state = Reboot;
			} else {
				error_cnt += 1;
			}
			break;
		case Reboot:
#ifdef NBIOT_DEBUG
			printf("step3.\r\n");
#endif
			prvATReboot();
			init_state = Set_Band;
			break;
		case Set_Band:
			if (prvATSetBand(5) == 0) {
#ifdef NBIOT_DEBUG
				printf("step4.\r\n");
#endif
				error_cnt = 0;
				init_state = Check_IMEI;
			} else {
				error_cnt += 1;
			}
			break;
		case Open_CFUN:
			//printf("Open CFUN\n");
			if (prvATOpenCFUN() == 0) {
				error_cnt = 0;
				init_state = Register_Net;
			} else {
				error_cnt += 1;
			}
			break;
		case Register_Net:
			if (prvATRegister_Net() == 0) {
				error_cnt = 0;
				init_state = Check_Net;
			} else {
				error_cnt += 1;
			}
			break;
		case Check_Net:
			if (cATGetNetStatus(pxDev) == 0) {
#ifdef NBIOT_DEBUG
				printf("step8.\r\n");
#endif

				error_cnt = 0;
				init_ret = 0;
				pxDev->ucInitStatus = 1;
			} else {
				if (check_cnt++ < 30) {
					init_state = Check_Net;
				} else {
					check_cnt = 0;
					error_cnt += 1;
					init_state = Close_CFUN;
				}
			}
			break;
		case Close_CFUN:
		//	printf("Close CFUN\n");
			if (prvATCloseCFUN() == 0) {
				error_cnt = 0;
				init_state = Open_CFUN;
			} else {
				error_cnt += 1;
			}
			break;
		default:
			break;
		}
//		if (error_cnt > 0) {
//#ifdef NBIOT_DEBUG
//			printf("error.\r\n");
//#endif
//			init_ret = -1;
//		}
	}
	if(error_cnt >= 10)
	{
		error_cnt  = 0;
		pxDev->ucInitStatus = 2;
		init_ret = -1;
	}
	if(pxDev->ucInitStatus == 1)
	{
		prvATDisablePSM();
		prvATOpenLed();
		prvATRegister();
		prvATOpenNNMI();
		data[0]  = 1;
//		msg_gd32_iotset_report(data,1);
		pxDev->ucInitStatus = 3;
		printf("OK....\r\n");
	}
	else if(pxDev->ucInitStatus == 2)
	{
		data[0]  = 0;
//		msg_gd32_iotset_report(data,1);
		pxDev->ucInitStatus = 3;
		printf("ERROR....\r\n");
	}
}

//coap 设备接入
int8_t cATCoapDeviceConnect(char *ip, uint32_t port)
{
	int8_t ret = 0;
	//关闭CFUN
	ret = prvATCloseCFUN();
	if (ret < 0)
		return ret;
	//设置对接平台IP、端口
	ret = prvATSetCoap(ip, port);
	if (ret < 0)
		return ret;
	//打开CFUN
	ret = prvATOpenCFUN();
	if (ret < 0)
		return ret;
	//设置频段
	//注册网络
	ret = prvATRegister_Net();
	if (ret < 0)
		return ret;
	//打开接收
	ret = prvATOpenNNMI();
	if (ret < 0)
		return ret;
	return ret;
}


