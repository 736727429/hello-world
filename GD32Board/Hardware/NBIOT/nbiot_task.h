/*
 * nbiot_task.h
 *
 *  Created on: Aug 6, 2019
 *      Author: yxq
 */

#ifndef NBIOT_NBIOT_TASK_H_
#define NBIOT_NBIOT_TASK_H_

#include "global.h"
#include "stdbool.h"
#include "stdlib.h"
#include "at_hand.h"


typedef enum
{
	eVersion = 0x05,
	eDeviceInfo = 0x0b,
	ePowerStatus,
	eDeviceStatus = 0x10,
	eSensorStatus = 0x11,
	eFunctionStatus = 0x12,
	eSystemStatus = 0x13,
	ePowerOn = 8001,
	ePowerOff = 8002,
	eReboot = 8003,
} eIotMsgId;



typedef struct xIotDeviceStatus
{
	char cConnection;
	char cHardwareStop;
	char cCharge;
	char cBattery;
	char cLeftMotor;
	char cRightMotor;
	char cX86Communication;
} __attribute__((packed))IotDeviceStatus_t;

typedef struct xIotSensorStatus
{
	char cLaser;
	char cFaceCamera;
	char cMilWave;
	char cImu;
	char cUltrasonic;
	char cEcompass;
	char cBump;
	char cDepthCamera;
	char cOdom[4];
} __attribute__((packed))IotSensorStatus_t;

typedef struct xIotFunctionStatus
{
	char cLocation;
	char cNavigation;
	char cActiveWelcomeGuests;
	char cFocusFollowing;
	char cRecharge;
} IotFunctionStatus_t;

typedef struct xIotSystemStatus
{
	char cX86;
	char cRosSystem;
	char cMCU;
} IotSystemStatus_t;


typedef struct xIotMsg
{
	uint8_t ucMessageId;
	union{
		IotDeviceStatus_t xIotDeviceStatus;
		IotSensorStatus_t xIotSensorStatus;
		IotFunctionStatus_t xIotFunctionStatus;
		IotSystemStatus_t xIotSystemStatus;
		char cData[12];
	} u;
	uint8_t ucLen;
	uint16_t usError;
	uint8_t ucStatusChange;
} IotMsg_t;

extern NbiotDevice_t xNbiotDev;
extern IotMsg_t xIotStatusMsg[4];
extern IotDeviceStatus_t xIotDeviceStatusTemp;
extern IotSensorStatus_t xIotSensorStatusTemp;
extern IotFunctionStatus_t xIotFunctionStatusTemp;
extern IotSystemStatus_t xIotSystemStatusTemp;

void vNbiotRecvTask(void);

void prvNbiotDeviceInfoReport();
void vNbiotSendTask(IotMsg_t xIotMsg);
void vNbiotDevicePowerStatus(char cStatus);
void vNbiotStatusReport(char id, char *pcData);
void vNbiotInit();
int StringToHex(char *str, char *out, uint8_t *outlen);

#endif /* NBIOT_NBIOT_TASK_H_ */
