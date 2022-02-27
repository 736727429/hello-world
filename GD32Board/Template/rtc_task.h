/*
 * rtc_task.h
 *
 *  Created on: Jun 13, 2018
 *      Author: yang
 */

#ifndef RTC_TASK_H_
#define RTC_TASK_H_


#include "rtc_rx8010.h"
#include "stdbool.h"
#include "global.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


typedef struct {
	rx8010_module_t *rtc;
	rtc_time_t	*rtc_time;
	rtc_alarm_t *rtc_alarm_time;
	uint32_t timestamp;
	uint8_t rtc_time_ask_status;
	uint8_t rtc_time_set_status;
} RTC_DEV_T;

/* Variables */
extern RTC_DEV_T g_rtc_dev;
extern uint8_t sync_flag;
extern uint8_t time_set[7];
extern uint8_t alarm_poweron_time[4];
extern uint8_t alarm_poweroff_time[4];
extern uint8_t alarm_poweroff_flag;
extern volatile bool rtc_time_ask_flag;
extern volatile bool rtc_alarm_timeon_ask_flag;
extern volatile bool rtc_alarm_timeoff_ask_flag;
extern volatile bool rtc_alarm_set_flag;

extern uint8_t alarm_set_time_count;
extern uint8_t alarm_set_time[40][6];
extern uint8_t alarm_set_time_t[6];
extern uint8_t alarm_set_time_t2[6];
extern uint8_t current_time_t[6];
extern uint16_t alarm_flash_count;

/* Functions */
void rtc_rx8010_dev_init(void);
void rtc_rx8010_init();
void rtc_rc8010_set_time();
void rtc_rx8010_timer_init();
void rtc_rx8010_get_time(void);
int8_t rtc_rx8010_set_alarm(uint8_t *data);
uint8_t days_of_month(rtc_time_t *rtc);
int8_t rtc_rx8010_set_time(uint8_t *data);
int8_t rtc_rx8010_sync_time(rtc_time_t *rtc);
void rtc_rx8010_task();

#endif /* RTC_TASK_H_ */
