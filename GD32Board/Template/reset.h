#ifndef RESET_H_
#define RESET_H_

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

#define PWR_12V_ON					1
#define PWR_12V_OFF					0
#define PWR_X86_ON					1
#define PWR_X86_OFF					0
#define PWR_MOTO_ON					1
#define PWR_MOTO_OFF				0
#define PWR_MAIN_ON					1
#define PWR_MAIN_OFF				0


typedef enum {
    CHARGER_NOCHARGE = 0,
    CHARGER_CHARGE,
    CHARGER_FULL,
    CHARGER_RESERVED =  '.'
} CHARGER_STATE_E;

typedef enum {
    POWERON_STATE_OFF_NOCHARGE = 0,
    POWERON_STATE_OFF_CHARGE,
    POWERON_STATE_ON_NOCHARGE,
    POWERON_STATE_ON_CHARGE,
    POWERON_STATE_RESERVED =  '.'
} POWERON_STATE_E;

typedef enum
{
	MAINBOARD_OFF = 0,
	MAINBOARD_ON,
	MAINBOARD_SLEEP,
	MAINBOARD_RESERVED
}MAINBOARD_STATUS_T;

typedef struct {
    uint8_t hard_version;
    uint8_t hard_switch;
    uint8_t hard_switch_hold_time;
    uint32_t hard_switch_press_cnt;
    uint8_t hard_switch_press_flag;
    uint32_t hard_switch_debug_mode_time;
    uint8_t debug_mode;
    uint8_t charging;
    POWERON_STATE_E state;
    char switch_detect_interval;
    uint32_t reset12v_maintain_time;
    char reset_flag;
    MAINBOARD_STATUS_T mainboard_status;
    MAINBOARD_STATUS_T iot_switch_status;
} RESET_DEV_T;

extern RESET_DEV_T g_st_reset_dev;
extern uint16_t pit0_time;
extern volatile bool pit0_time_flag;
extern volatile bool pit1_time_flag;
extern uint32_t switch_hode_time;

extern volatile uint8_t gHWVer;
extern char gd32_version[30] ;

extern uint32_t gd32_reset_rcu;
extern uint8_t gd32_reset_rcu_count;
extern volatile bool gd32_reset_rcu_flag;
extern volatile bool gd32_reset_report_flag;

void reset_init();
void check_hard_switch();
int8_t check_hard_switch_power_on();
void main_board_sleep();
//int8_t power_on_init();
//void poweron_get_status(uint8_t hard_switch, uint8_t charging);
//void poweron_status_update() ;
void gd32_version_report();
int StringToHex(char *str, char *out, uint8_t *outlen);
char* substring(char* ch,int pos,int length);

#endif /* RESET_H_ */
