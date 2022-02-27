#include "bcd.h"
#include "rtc_rx8010.h"
#include "systick.h"
#include <stdio.h>
#include "rtc_task.h"

rx8010_i2c_bus_t rx8010_i2c_bus = {
	i2c_write_byte,
	i2c_read_byte
};

static void rx8010_irq_1_handler(int irq, void *dev_id)
{
	int flagreg;

/*	spin_lock(&rx8010->flags_lock);

	flagreg = i2c_smbus_read_byte_data(client, RX8010_FLAG);

	if (flagreg <= 0) {
		spin_unlock(&rx8010->flags_lock);
		return IRQ_NONE;
	}

	if (flagreg & RX8010_FLAG_VLF)
		dev_warn(&client->dev, "Frequency stop detected\n");

	if (flagreg & RX8010_FLAG_TF) {
		flagreg &= ~RX8010_FLAG_TF;
		rtc_update_irq(rx8010->rtc, 1, RTC_PF | RTC_IRQF);
	}

	if (flagreg & RX8010_FLAG_AF) {
		flagreg &= ~RX8010_FLAG_AF;
		rtc_update_irq(rx8010->rtc, 1, RTC_AF | RTC_IRQF);
	}

	if (flagreg & RX8010_FLAG_UF) {
		flagreg &= ~RX8010_FLAG_UF;
		rtc_update_irq(rx8010->rtc, 1, RTC_UF | RTC_IRQF);
	}

	i2c_smbus_write_byte_data(client, RX8010_FLAG, flagreg);

	spin_unlock(&rx8010->flags_lock);*/
}


int8_t rx8010_module_init(rx8010_module_t *p_module)
{
	p_module->slave_addr = RX8010_SLAVE_ADDR;
	p_module->i2c_bus = &rx8010_i2c_bus;

	return OK;
}

int8_t rx8010_init(rx8010_module_t *p_module)
{
	uint8_t ctrl = 0;
	uint8_t need_clear = 0;
	int8_t ret = 0;

	/*Init reserved registers ad specified*/
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_RESV17, 0xD8);
	if(ret < 0)
		return ERROR;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_RESV30, 0x00);
	if(ret < 0)
		return ERROR;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_RESV31, 0x08);
	if(ret < 0)
		return ERROR;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_IRQ, 0x00);
	if(ret < 0)
		return ERROR;

	ctrl = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(ctrl & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		delay_ms(50);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_FLAG, 0x00);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_CTRL, 0x00);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_CTRL, 0x80);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, 0x60, 0xD3);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, 0x66, 0x03);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, 0x6B, 0x02);
		p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, 0x6B, 0x01);
	}
	if(ctrl & RX8010_FLAG_AF) {
		need_clear = 1;
		printf("Alarm was detected\n");
	}
	if(ctrl & RX8010_FLAG_TF) {
		need_clear = 1;
		printf("Timer was detected\n");
	}
	if(ctrl & RX8010_FLAG_UF) {
		need_clear = 1;
		printf("Update was detected\n");
	}

	if(need_clear) {
		ctrl &= ~(RX8010_FLAG_AF | RX8010_FLAG_TF | RX8010_FLAG_UF);
		ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_FLAG, ctrl);
		if(ret < 0)
			return ERROR;
	}

	return OK;
}

int8_t rx8010_check_vlf(rx8010_module_t *p_module)
{
	int8_t flagreg;
	int8_t ret;

	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0) {
		printf("I2C ERROR-1\n");
		return ERROR;
	}
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		flagreg &= ~RX8010_FLAG_VLF;
		ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_FLAG, flagreg);
		if(ret < 0)
			printf("I2C ERROR-2\n");
		return ERROR;
	}

	return OK;
}

int8_t rx8010_get_time(rx8010_module_t *p_module, rtc_time_t *dt)
{
	int8_t flagreg;
	uint8_t data[7];
	uint8_t i;

	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0)
		return ERROR;
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		return ERROR;
	}

	for (i = 0; i < 7; i++) {
		data[i] = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_SEC + i);
		if(data[i] < 0)
			return ERROR;
	}

	dt->time.sec = BCD2BIN(data[0] & 0x7f);
	dt->time.min = BCD2BIN(data[1] & 0x7f);
	dt->time.hour = BCD2BIN(data[2] & 0x3f);
	dt->week = BCD2BIN(data[3] & 0x7f);
	dt->date.day = BCD2BIN(data[4] & 0x3f);
	dt->date.month = BCD2BIN(data[5] & 0x1f);
	dt->year = BCD2BIN(data[6]) + 2000;

	return OK;
}

int8_t rx8010_set_time(rx8010_module_t *p_module, rtc_time_t *dt)
{
	int8_t flagreg;
	uint8_t data[7];
	int8_t ret;
	int8_t ctrl;
	uint8_t i;

	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		return ERROR;
	}

	data[0] = BIN2BCD(dt->time.sec);
	data[1] = BIN2BCD(dt->time.min);
	data[2] = BIN2BCD(dt->time.hour);
	data[3] = BIN2BCD(dt->week);
	data[4] = BIN2BCD(dt->date.day);
	data[5] = BIN2BCD(dt->date.month);
	data[6] = BIN2BCD(dt->year);

	/* set stop bit before changing time data */
	ctrl = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_CTRL);
	if(ctrl < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}
	ctrl |= RX8010_CTRL_STOP;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_CTRL, ctrl);
	if(ret < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}


	for (i = 0; i < 7; i++) {
		ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_SEC + i, data[i]);
		if(ret < 0) {
			printf("I2C ERROR\n");
			return ERROR;
		}
	}

	/* clear stop bit after changing time data */
	ctrl = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_CTRL);
	if(ctrl < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}
	ctrl &= ~RX8010_CTRL_STOP;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_CTRL, ctrl);
	if(ret < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}
	/* check VLF flag */
	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0) {
		printf("I2C ERROR\n");
		return ERROR;
	}
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		return ERROR;
	}

	return OK;
}


int8_t rx8010_read_alarm(rx8010_module_t *p_module, rtc_alarm_t *at)
{
	uint8_t alarmval[3];
	int8_t flagreg;
	uint8_t i;

	/* check VLF flag */
	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0)
		return ERROR;
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		return ERROR;
	}

	/* Read Alarm Reg */
	for (i = 0; i < 3; i++) {
		alarmval[i] = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_ALMIN + i);
		if (alarmval[i] < 0)
			return ERROR;
	}

	at->time.sec = 0;
	at->time.min = BCD2BIN(alarmval[0] & 0x7f);
	at->time.hour = BCD2BIN(alarmval[1] & 0x3f);

	if (!(alarmval[2] & RX8010_ALARM_AE))
		at->date.day = BCD2BIN(alarmval[2] & 0x7f);

	return OK;
}

int8_t rx8010_set_alarm(rx8010_module_t *p_module, rtc_alarm_t *at)
{
	uint8_t alarmval[3];
	int8_t extreg, flagreg;
	uint8_t i;
	int8_t ret;

	/* check VLF flag */
	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if(flagreg < 0)
		return ERROR;
	if(flagreg & RX8010_FLAG_VLF) {
		printf("Frequency stop was detected\n");
		return ERROR;
	}

	alarmval[0] = BIN2BCD(at->time.min);
	alarmval[1] = BIN2BCD(at->time.hour);
	alarmval[2] = BIN2BCD(at->date.day);
//		alarmval[0] = at->time.min;
//		alarmval[1] = at->time.hour;
//		alarmval[2] = at->date.day;

	/* Read Alarm Reg */
	for (i = 0; i < 3; i++) {
		ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_ALMIN + i, alarmval[i]);
		if (ret < 0)
			return ERROR;
	}
	extreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_EXT);
	if(extreg < 0)
		return ERROR;
	extreg |= RX8010_EXT_WADA;
	ret = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_EXT, extreg);
	if(ret < 0)
		return ERROR;

	return OK;
}

int8_t rx8010_alarm_irq_enable(rx8010_module_t *p_module, unsigned int enabled)
{
	int flagreg;
	uint8_t ctrl;
	int err;

	ctrl = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_CTRL);
	if (ctrl < 0)
		return ctrl;

	if (enabled) {
			ctrl |= (RX8010_CTRL_AIE | RX8010_CTRL_UIE);
	} else {
			ctrl &= ~RX8010_CTRL_AIE;
	}

	flagreg = p_module->i2c_bus->i2c_read_byte(p_module->slave_addr, RX8010_FLAG);
	if (flagreg < 0)
		return flagreg;

	flagreg &= ~RX8010_FLAG_AF;
	err = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_FLAG, flagreg);
	if (err < 0)
		return err;

	err = p_module->i2c_bus->i2c_write_byte(p_module->slave_addr, RX8010_CTRL, ctrl);
	if (err < 0)
		return err;

	return OK;
}

int rtc_rx8010_set_alarm_cal(uint8_t *data)
{
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	day = BCD2BIN(data[0]) + g_rtc_dev.rtc_time->date.day;
	hour = BCD2BIN(data[1]) + g_rtc_dev.rtc_time->time.hour;
	min = BCD2BIN(data[2]) + g_rtc_dev.rtc_time->time.min;
	sec = BCD2BIN(data[3]) + g_rtc_dev.rtc_time->time.sec;

	data[5] = g_rtc_dev.rtc_time->year - 2000;
	data[4] = g_rtc_dev.rtc_time->date.month;
	data[3] = sec % 60;
	data[2] =(sec / 60 + min ) % 60;
	data[1] =(( sec / 60 + min ) / 60 + hour) % 24 ;
	data[0] =((( sec / 60 + min ) / 60 + hour) / 24) + day;

	if(data[0] > days_of_month(g_rtc_dev.rtc_time))
	{
		data[0] = 1;
		data[4] += 1; 
		if(data[4] > 12)
		{
			data[4] = 1;
			data[5] += 1;
		}
	}
}
