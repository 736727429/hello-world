#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include "gd32e230.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"


#define ERROR				-1
#define FAIL    		-2
#define OK					0
#define LEAVE_VAL		1
#define DEVICE_ON		1
#define DEVICE_OFF	0


#define GD_USER_LED_GPIO 	 GPIOB
#define GD_USER_LED_PIN	 	 GPIO_PIN_4 
#define GD_ADP_DET_GPIO 	 GPIOB
#define GD_ADP_DET_PIN	 	 GPIO_PIN_3
#define GD_WAKEUP_NB_GPIO  	 GPIOB
#define GD_WAKEUP_NB_PIN	 GPIO_PIN_2
#define GD_RTC_IRQ_GPIO 	 GPIOA
#define GD_RTC_IRQ_PIN	 	 GPIO_PIN_15
#define MAIN_PWR_CTRL_GPIO 	 GPIOA
#define MAIN_PWR_CTRL_PIN	 GPIO_PIN_0
#define X86_12V_EN_GPIO 	 GPIOA
#define X86_12V_EN_PIN		 GPIO_PIN_1 
#define EXT_12V_EN_GPIO 	 GPIOA
#define EXT_12V_EN_PIN	     GPIO_PIN_2
#define ON_OFF_SW_GPIO 		 GPIOB
#define ON_OFF_SW_PIN	 	 GPIO_PIN_1
#define GD_WAKEUP_NB_GPIO    GPIOB
#define GD_WAKEUP_NB_PIN	 GPIO_PIN_2
#define GD_IOT_SW_GPIO		 GPIOB
#define GD_IOT_SW_PIN    	 GPIO_PIN_5

#define DEBUG_ENABLE 0

#endif