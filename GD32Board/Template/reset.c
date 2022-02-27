#include "reset.h"
#include "timer.h"
#include "global.h"
#include "hardware_switch.h"
#include "systick.h"
#include "rtc_task.h"
#include "uart.h"
#include "assert.h"

RESET_DEV_T g_st_reset_dev;
uint8_t switch_on_time = 0;
uint8_t switch_off_time = 0;
uint32_t switch_hode_time = 0;
uint32_t main_poweroff_publish_count = 0;
uint16_t pit0_time = 0;
volatile bool pit0_time_flag = false;
volatile bool pit1_time_flag = false;

uint32_t gd32_reset_rcu = 0;
uint8_t gd32_reset_rcu_count = 0;
volatile bool gd32_reset_rcu_flag = false;
volatile bool gd32_reset_report_flag = true;

void reset_init()
{
	g_st_reset_dev.charging = 0;
	g_st_reset_dev.hard_switch = 0;
	g_st_reset_dev.hard_switch_hold_time = 0;
	g_st_reset_dev.hard_switch_press_cnt = 0;
	g_st_reset_dev.hard_switch_press_flag = 0;
	g_st_reset_dev.mainboard_status = MAINBOARD_OFF;
	g_st_reset_dev.iot_switch_status = MAINBOARD_OFF;
}

void check_hard_switch()
{
	if(!gpio_input_bit_get(ON_OFF_SW_GPIO,ON_OFF_SW_PIN)) {
		if(g_st_reset_dev.hard_switch_hold_time++ > 4) {
			g_st_reset_dev.hard_switch_hold_time = 4;
		}
	} else {
		g_st_reset_dev.hard_switch_hold_time = 0;
	}
}


int8_t check_hard_switch_power_on(void)
{
	uint8_t switch_time = 0;
	timer2_nvic_enable();
	while (1) {
		if(timer2_flag)
		{
			timer2_flag = false;
			RT1020_charge();
			if(!gpio_input_bit_get(ON_OFF_SW_GPIO,ON_OFF_SW_PIN)) {
				switch_time++;
			} else {
				switch_time = 0;
			}
			if (switch_time >= 3) {
				g_st_reset_dev.hard_switch_hold_time = 3;
				break;
			}
		}
	}
	timer2_nvic_disable();
	xHWSwitchDev.ucSwitchPressOffFlag = 0;
	if (xHWSwitchDev.xHWSMode == eHWSPwrOff) {
		xHWSwitchDev.xHWSMode = eHWSPwrOn;
		g_st_reset_dev.mainboard_status = MAINBOARD_ON;
		g_st_reset_dev.iot_switch_status = MAINBOARD_ON;
		gpio_bit_set(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_en
		/*open X86_12V&EXT_12V*/
//		delay_ms(1000);
//		gpio_bit_reset(EXT_12V_EN_GPIO,EXT_12V_EN_PIN); //12V_pwr_en
//		delay_ms(1000);
//		gpio_bit_reset(X86_12V_EN_GPIO,X86_12V_EN_PIN); //x86_pwr_en
//		delay_ms(1000);		
//		msg_x86_poweron_report(data,1) ;
		printf("pwr_main on.\n");
		timer13_nvic_enable();
	} else if (xHWSwitchDev.xHWSMode == eHWSPwrOn) {
		xHWSwitchDev.xHWSMode = eHWSPwrOff;
		gpio_bit_reset(MAIN_PWR_CTRL_GPIO,MAIN_PWR_CTRL_PIN);		//main_board_pwr_en
	}
	return 0;
}

//int8_t power_on_init()
//{
//	pit_init();
//	GPIO_PortClear(BOARD_INITPINS_LED_USER_GPIO_PORT, 1 << BOARD_INITPINS_LED_USER_PIN);
//	return 0;
//}

//void poweron_get_status(uint8_t hard_switch, uint8_t charging)
//{
//    uint8_t state;
//    state = hard_switch + (charging << 1);
//    switch(state) {
//        case 1: g_st_reset_dev.state = POWERON_STATE_ON_NOCHARGE; break;
//        case 0: g_st_reset_dev.state = POWERON_STATE_OFF_NOCHARGE; break;
//        case 3: g_st_reset_dev.state = POWERON_STATE_ON_CHARGE; break;
//        case 2: g_st_reset_dev.state = POWERON_STATE_OFF_CHARGE; break;
//        default:break;
//    }
//}

//void poweron_status_update() 
//{
//    POWERON_STATE_E state_tmp;
//    state_tmp = g_st_reset_dev.state;
//    poweron_get_status(g_st_reset_dev.hard_switch, g_st_reset_dev.charging);

//}

uint32_t version_time = 0;

void gd32_version_report()
{
	uint8_t ver1_data[8] = {0};
	uint8_t ver2_data[8] = {0};
	
//	printf("ver:%s\r\n",gd32_version);
	memset(ver1_data,0,sizeof(ver1_data));
	memset(ver2_data,0,sizeof(ver2_data));
	
	ver1_data[0] = gHWVer;
	char *p = NULL;
	char *q = NULL;
	uint8_t len = 0;
	p = strstr(gd32_version,"V");
	q = substring(p+1,0,3);
	char out[3] = {0};
	if(q != NULL)
	{
		len = 0;
		StringToHex(q,out,&len);
	}
	ver1_data[1] = ((out[0] << 4) | out[1]) & 0xff;
	ver1_data[2] = (((out[0] << 4) | out[1]) >> 8) & 0xff;
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"M");
	q = substring(p+1,0,2);
	memset(out,0,sizeof(out));
	if(q != NULL)
	{
		len = 0;
		StringToHex(q,out,&len);
	}
	ver1_data[3] = out[0];
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"R");
	q = substring(p+1,0,3);
	memset(out,0,sizeof(out));
	if(q != NULL)
	{
		len = 0;
		StringToHex(q,out,&len);
	}
	ver1_data[4] = ((out[0] << 4) | out[1]) & 0xff;
	ver1_data[5] = (((out[0] << 4) | out[1]) >> 8) & 0xff;
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"C");
	q = substring(p+1,0,3);
	memset(out,0,sizeof(out));
	if(q != NULL)
	{
		len = 0;
		StringToHex(q,out,&len);
	}
	ver1_data[6] = ((out[0] << 4) | out[1]) & 0xff;
	ver1_data[7] = (((out[0] << 4) | out[1]) >> 8) & 0xff;
	msg_gd32_version1_report(ver1_data,8);
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"B");
	q = substring(p+1,0,3);
	memset(out,0,sizeof(out));
	if(q != NULL)
	{
		len = 0;
		StringToHex(q,out,&len);
	}
	ver2_data[0] = ((out[0] << 4) | out[1]) & 0xff;
	ver2_data[1] = (((out[0] << 4) | out[1]) >> 8) & 0xff;
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"SP");
	memset(out,0,sizeof(out));
	if(p != NULL)
	{
		q = substring(p+2,0,1);
		if(q != NULL){
			len = 0;
			StringToHex(q,out,&len);
		}
	}
	ver2_data[2] = out[0] ;
	p = NULL;
	q = NULL;
	p = strstr(gd32_version,"-g");
	memset(out,0,sizeof(out));
	if(p != NULL)
	{
		q = substring(p+2,0,6);
		if(q != NULL){
			len = 0;
			StringToHex(q,out,&len);
		}
	}
	ver2_data[3] = out[2] ;
	ver2_data[4] = out[1] ;
	ver2_data[5] = out[0] ;
	msg_gd32_version2_report(ver2_data,6);
}

int StringToHex(char *str, char *out, uint8_t *outlen)
{
	char *p = str;
	char high = 0, low = 0;
	int tmplen = strlen(p), cnt = 0;
	if (str == NULL)
		return -1;
	tmplen = strlen(p);
	while(cnt < (tmplen / 2))
	{
		high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
		low = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
		out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
		p ++;
		cnt ++;
	}
	if(tmplen % 2 != 0) out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

	if(outlen != NULL) *outlen = tmplen / 2 + tmplen % 2;
	return tmplen / 2 + tmplen % 2;
}

char* subch;
char* substring(char* ch,int pos,int length)
{
    char* pch=ch;
//定义一个字符指针，指向传递进来的ch地址。
	free(subch);
    subch=(char*)calloc(sizeof(char),length+1);
//通过calloc来分配一个length长度的字符数组，返回的是字符指针。
     int i;
 //只有在C99下for循环中才可以声明变量，这里写在外面，提高兼容性。
     pch=pch+pos;
 //是pch指针指向pos位置。
     for(i=0;i<length;i++)
    {
        subch[i]=*(pch++);
//循环遍历赋值数组。
    }
    subch[length]='\0';//加上字符串结束符。
    return subch;       //返回分配的字符数组地址。
}
