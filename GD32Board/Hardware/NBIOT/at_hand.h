/*
 * AT.h
 *
 *  Created on: Aug 6, 2019
 *      Author: yxq
 */

#ifndef NBIOT_AT_HAND_H_
#define NBIOT_AT_HAND_H_

#include "global.h"
#include "string.h"
#include "stdlib.h"
#include "at_wrapper.h"


/****************************   define   *************************/
#define  NBIOTAT_UART_BAUDRATE   9600

#define  NBIOTAT_UARTBUF_MAX      50
#define  NBIOTAT_UARTRX_MAX      5
#define  NBIOTAT_UARTTX_MAX     2

#define NBIOT_BAND			5

//#define NBIOT_DEBUG         0

extern uint16_t iot_recv_count;
extern volatile bool iot_setok_from_rt1020;
extern int8_t init_state;
extern volatile bool iot_setstep_flag;

typedef enum
{
	NBIOT_WORK_NULL           = 0 ,
	NBIOT_WORK_SET             = 10 ,
	NBIOT_WORK_TCP_NULL        = 80 ,
	NBIOT_WORK_TCP_CREAT,
	NBIOT_WORK_TCP_CONNECT,
	NBIOT_WORK_TCP_SEND,
	NBIOT_WORK_TCP_REC,
	NBIOT_WORK_TCP_CLOSE,
	NBIOT_WORK_RESERVED       = 100
} NBIOT_WORK_STATE;

typedef enum
{
	Check_Hand = 0,
	Check_Ver,
	Check_IMEI,
	Check_SIM,
	Set_CDP_Service,
	Reboot,
	Set_Band,
	Open_CFUN,
	Register_Net,
	Check_Net,
	Close_CFUN,
	Close_AutoConnect,
	Check_CSQ,
	Error,
} NBIOT_INIT_STATE_E;






/****************************   bc28  mind  define *************************/
//常用AT指令
#define		NBIOT_OPENUE								"AT+CMEE=1\r\n"			//return OK\r\n
#define		NBIOT_CHECK_VER 							"ATI\r\n"				//return  Quectel\r\n  BC28\r\n Revision:BC28JAR01A03\r\n
#define		NBIOT_CHECK_IMEI 							"AT+CGSN=1\r\n"			//return  +CGSN:867726030047601\r\n  OK\r\n
#define		NBIOT_SET_ATE								"ATE1\r\n"
#define 	NBIOT_REBOOT								"AT+NRB\r"
//网络注册
#define 	NBIOT_CHECK_STATE   						"AT\r\n"				// 查看模块是否正常工作 return  OK\r\n
#define 	NBIOT_CHECK_IMSI   							"AT+CIMI\r\n"			//检查SIM卡是否读卡成功 ,出现ERROR请检查SIM是否插好    return  460001357924680\r\n OK\r\n
#define  	NBIOT_CHECK_CSQ   							"AT+CSQ\r\n"			// 查看信号强度   return  +CSQ:99,99 \r\nOK \r\n
#define  	NBIOT_CHECK_SUPPORT_BAND					"AT+NBAND=?\r\n"		//查看BC28支持的频段   return  +NBAND:(1,3,5,8,20,28)\r\nOK \r\n
#define  	NBIOT_SET_NBAND    							"AT+NBAND=5\r\n"		//切换频段，BAND5为电信频段，BAND8为移动联通频段  return OK \r\n
#define  	NBIOT_CHECK_NBAND							"AT+NBAND=5\r\n"		//查看当前所在的频段  return  +NBAND:5\r\n OK \r\n
#define		NBIOT_CHECK_NET_STATUS    					"AT+CGATT?\r\n"         // 查看入网状态，返回0为入网失败，返回1为入网成功     return  +CGATT:1\r\nOK \r\n
#define		NBIOT_REGISTER_NET    						"AT+CGATT=1\r\n"
#define		NBIOT_CHECK_CFUN							"AT+CFUN?\r\n"
#define  	NBIOT_CLOSE_CFUN	   						"AT+CFUN=0\r\n"         //  如果入网成功，但是信号强度依然是9，先执行AT+CFUN=0再执行AT+CFUN=1,再次查询信号强度
#define 	NBIOT_OPEN_CFUN 	   						"AT+CFUN=1\r\n"         //


//发送和接收TCP数据
#define 	NBIOT_CREATE_SOCKET   						"AT+NSOCR=STREAM,6,56000,1\r\n"   //Create a socket，56000是设备端口，设置范围1-66535     return  1\r\nOK \r\n
#define  	NBIOT_CONNECT_SERVER  						"AT+NSOCO=1,123.206.108.227,9099\r\n"   //Connect to server，这个是大白自己搭建的服务器地址，用于测试
                                                                                                                                                                                  //return   OK \r\n
#define  	NBIOT_SEND_TCP1    							"AT+NSOSD=1,3,010203\r\n"    	//Send the messages: 0x01 0x02 0x03   return  1,3\r\nOK \r\n +NSONMI:1,3 \r\n
#define  	NBIOT_SEND_TCP2  							"AT+NSOSD=1,3,616263\r\n" 		//Send the messages:0x61 0x62 0x63
#define  	NBIOT_READ_TCP   							"AT+NSORF=1,3\r\n"				//Read the messages,读取到的值是之前发送的数据0x01 0x02 0x03
                                                                                                                             //return 1,220.180.239.212,8009,3,010203,0 \r\nOK
#define  	NBIOT_CLOSE_SOCKET  						"AT+NSOCL=0\r\n"   //Closethe socket      return   OK \r\n


//接入电信的IOT流程
#define  	NBIOT_SET_LOT  								"AT+NCDP=180.101.147.115,5683\r\n" 		//设置IOT平台IP和端口号     return   OK \r\n
#define  	NBIOT_CHECK_MODE 							"AT+QREGSWT?\r\n"                       //查询模块注册模式          return   +QREGSWT:1\r\nOK \r\n
#define  	NBIOT_OPEN_MODE 							"AT+QLWSREGIND=0\r\n"                	//模块启动寄存器注册到IOT平台      return    OK \r\n
#define  	NBIOT_SEND_NON 								"AT+QLWULDATA=3,313233\r\n"             //模块发送NON数据到IOT平台      return    OK \r\n
#define  	NBIOT_SEND_CON   							"AT+QLWULDATAEX=3,313233,0X0100\r\n"    //模块发送CON数据到IOT平台    return    OK \r\n+QLWULDATASTATUS:4\r\n
#define  	NBIOT_CHECK_CON   							"AT+QLWULDATASTATUS?\r\n"               //查询发送CON数据到IOT平台的状态   return    +QLWULDATASTATUS:4\r\mOK \r\n


/****************************   function   *************************/
int8_t cATModuleInit(NbiotDevice_t *pxDev, char *ip, uint32_t port);
int8_t cATGetNetStatus(NbiotDevice_t *pxDev);
int8_t cATCoapDeviceConnect(char *ip, uint32_t port);
int8_t cATCoapSendData(char *data, uint16_t len);
int8_t cATCoapRecvData(char *data);
int8_t cATDisregister(NbiotDevice_t *pxDev);
int8_t prvATCloseCFUN();
int8_t prvATOpenCFUN();
int8_t prvATRegister_Net(void);

#endif /* NBIOT_AT_HAND_H_ */
