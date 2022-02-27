#ifndef _TIMER_H_
#define _TIMER_H_

#include "stdint.h"
#include "stdbool.h"

extern volatile bool timer2_flag;
extern volatile bool timer13_flag;

void timer2_nvic_enable(void);
void timer2_nvic_disable(void);
void timer13_nvic_enable(void);
void timer13_nvic_disable(void);
void timer_config(void);



#endif