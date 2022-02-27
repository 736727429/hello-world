#ifndef _IOT_H_
#define _IOT_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

extern volatile bool iot_wakeup_enable;

void Iot_rec_task();
void Iot_wakeup_check();
void Iot_wakeup_exti_init(void);
void Iot_sw(uint8_t num);







#endif

