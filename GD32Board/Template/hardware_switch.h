#ifndef HARDWARE_SWITCH_H_
#define HARDWARE_SWITCH_H_
#include "stdint.h"
#include "stdbool.h"

typedef enum{
	eHWSPwrOn,
	eHWSPwrOff,
	eHWSDebug = 9
} HWSMode_t;

typedef enum{
	NOCHARGE = 0,
	CHARGE
}Charge_Status_t;

typedef enum{
  PWR_OFF_NOCHARGE = 0,
  PWR_OFF_CHARGE,
  PWR_ON_NOCHARGE,
  PWR_ON_CHARGE,
  PWR_RESERVED =  '.'
}Pwr_Status;
/*!
 * @brief hardware switch params struct
 */
typedef struct _xHWSwitchDev
{
	HWSMode_t xHWSMode;
	uint8_t charge_status;
	uint8_t PowerState;
	uint8_t ucGpioState;
	uint8_t ucSwitchPressFlag;
	uint16_t ulSwitchPressTime;
	uint16_t ulSwitchHoldTime;
	uint16_t ulDebugReminderTime;
	uint8_t ucGD32Shutdown_Charge_Flag;
	uint8_t ucGD32Shutdown_Nocharge_Flag;
	uint8_t ucShutdown_charge_Flag;
	int16_t ulGD32ShutdownDelayTime;
	uint8_t ucSwitchPressOffFlag;
} HWSwitchDev_t;

extern HWSwitchDev_t xHWSwitchDev;

extern uint8_t power_on_flag;
extern volatile bool gd32_poweroff_flag;
extern volatile bool gd32_lowpower_flag;
extern volatile bool gd32_reset_flag;
extern volatile bool gd32_reset_cmp_flag;
extern volatile bool gd32_charge_protect_flag;

void HWS_Init(void);
void HWS_CheckHWSitch();
void hardware_switch_task();
void RT1020_charge();
void RT1020_charge_status();
void PM_GetPowerState(uint8_t ucHWSwitch, uint8_t ucCharging);


#endif /* HARDWARE_SWITCH_H_ */
