#include "timer.h"
#include "gd32e230.h"
#include <stdio.h>
#include "gd32e230c_eval.h"

volatile bool timer2_flag = false;
volatile bool timer13_flag = false;

/**
    \brief      configure the TIMER interrupt
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer2_nvic_enable(void)
{
    nvic_irq_enable(TIMER2_IRQn, 2);
}

void timer2_nvic_disable(void)
{
    nvic_irq_disable(TIMER2_IRQn);
}

void timer13_nvic_enable(void)
{
    nvic_irq_enable(TIMER13_IRQn, 2);
}

void timer13_nvic_disable(void)
{
    nvic_irq_disable(TIMER13_IRQn);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void)
{
    /* ----------------------------------------------------------------------------
    TIMER13 Configuration: 
    TIMER13CLK = SystemCoreClock/3600 = 20KHz.
    TIMER13 configuration is timing mode, and the timing is 0.2s(4000/20000 = 0.2s).
    CH0 update rate = TIMER13 counter clock/CH0CV = 20000/4000 = 5Hz.
    ---------------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    /* enable the peripherals clock */
	rcu_periph_clock_enable(RCU_TIMER13);

    /* deinit a TIMER */
	timer_deinit(TIMER13);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER13 configuration */
    timer_initpara.prescaler         = 3599;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 2000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_init(TIMER13, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* configure TIMER channel output function */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	timer_channel_output_config(TIMER13, TIMER_CH_0, &timer_ocinitpara);
		
	/* configure TIMER channel output pulse value */
    timer_channel_output_pulse_value_config(TIMER13, TIMER_CH_0, 2000);
    /* CH0 configuration in OC timing mode */
    timer_channel_output_mode_config(TIMER13, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    /* configure TIMER channel output shadow function */
    timer_channel_output_shadow_config(TIMER13, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
		
	/* enable the TIMER interrupt */
    timer_interrupt_enable(TIMER13, TIMER_INT_CH0);
    /* enable a TIMER */
    timer_enable(TIMER13);
		
	timer13_nvic_disable();
}