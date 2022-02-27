#include "Iot.h"
#include "global.h"
#include "hardware_switch.h"
#include "reset.h"

volatile bool iot_wakeup_enable = false;

void Iot_rec_task()
{
	if(iot_wakeup_enable)
	{
		iot_wakeup_enable = false;
		if((g_st_reset_dev.mainboard_status == MAINBOARD_OFF)&&(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0))
		{			
			printf("iot_control_poweron.......\n");
			xHWSwitchDev.xHWSMode = eHWSPwrOn;
		}
	}
}

void Iot_wakeup_exti_init(void)
{
    /* enable the CFGCMP clock */
    rcu_periph_clock_enable(RCU_CFGCMP);
    /* enable and set key EXTI interrupt to the specified priority */
    nvic_irq_enable(EXTI2_3_IRQn, 1U);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN2);

    /* configure key EXTI line */
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_2);
}

void Iot_sw(uint8_t num)
{
	if(num == MAINBOARD_OFF)
	{
		if(xHWSwitchDev.ulGD32ShutdownDelayTime <= 0)
		{
			gpio_bit_reset(GD_IOT_SW_GPIO,GD_IOT_SW_PIN);
//			printf("MAINBOARD_OFF\r\n");
		}
	}
	else if(num == MAINBOARD_ON)
	{
		gpio_bit_set(GD_IOT_SW_GPIO,GD_IOT_SW_PIN);
//		printf("MAINBOARD_ON\r\n");
	}
}
