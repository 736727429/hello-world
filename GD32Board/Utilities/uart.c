#include "uart.h"
#include "gd32e230c_eval.h"
#include "systick.h"
#include "crc8.h"
#include "rtc_task.h"
#include "hardware_switch.h"
#include "bcd.h"
#include "reset.h"
#include "at_hand.h"
#include "nbiot_task.h"
#include "updateflag.h"

UART_PRIMARY_DEV_T  g_st_uart[2];

UART1_TRANSMITE_ST   g_st_rt1020_dev;
MSG_PROTOCOL_T g_st_msg_protocol;
BATTERY_REPORT_T battery_report;
uint8_t X86ECDET = 1;//X86ElectricityCurrentDetection

void uart0_init()
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(USART0_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(USART0_CLK);

    /* connect port to USARTx_Tx */
    gpio_af_set(USART0_GPIO_PORT, USART0_AF, USART0_TX_PIN);

    /* connect port to USARTx_Rx */
    gpio_af_set(USART0_GPIO_PORT, USART0_AF, USART0_RX_PIN);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(USART0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_TX_PIN);
    gpio_output_options_set(USART0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, USART0_TX_PIN);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(USART0_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART0_RX_PIN);
    gpio_output_options_set(USART0_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, USART0_RX_PIN);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_enable(USART0);
		
		/* enable USART TBE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_TBE);
}


void uart1_init()
{
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(USART1_GPIO_CLK);

    /* enable USART clock */
    rcu_periph_clock_enable(USART1_CLK);

    /* connect port to USARTx_Tx */
    gpio_af_set(USART1_GPIO_PORT, USART1_AF, USART1_TX_PIN);

    /* connect port to USARTx_Rx */
    gpio_af_set(USART1_GPIO_PORT, USART1_AF, USART1_RX_PIN);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(USART1_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART1_TX_PIN);
    gpio_output_options_set(USART1_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, USART1_TX_PIN);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(USART1_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART1_RX_PIN);
    gpio_output_options_set(USART1_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, USART1_RX_PIN);

    /* USART configure */
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    usart_enable(USART1);
		
		/* enable USART TBE interrupt */  
    usart_interrupt_enable(USART1, USART_INT_TBE);
}

/**/
int uart_primary_init(uint32_t usart_periph) 
{
    uint32_t uart_index;
    uart_index = (usart_periph - USART0)/(USART1 - USART0);
    memset(g_st_uart[uart_index].read_buf, 0, sizeof(g_st_uart[uart_index].read_buf));

    g_st_uart[uart_index].write_index = 0;
    g_st_uart[uart_index].read_index= 0;
    g_st_uart[uart_index].over_flow_num= 0;  

    return OK;
}


int uart_primary_write_noblock(uint32_t usart_periph, const uint8_t *buffer, uint32_t length)
{
    uint32_t index = 0;
    uint32_t uart_index;
    uart_index = (usart_periph - USART0)/(USART1 - USART0);
//		printf("debug t10.\r\n");

    while(index != length) {
        usart_data_transmit(usart_periph, (uint8_t)buffer[index]);
				while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE))
				{}
        index++;
    }
//    printf("debug t11.\r\n");
    return  length;
}

int uart_primary_read_block(uint32_t usart_periph,  uint8_t *buffer, uint32_t length)
{
    uint32_t copy_index = 0;
    uint32_t tmp_len = length;
    uint32_t uart_index;
    uart_index = (usart_periph - USART0)/(USART1 - USART0);
//    printf("debug t1.uart_index=%d\r\n",uart_index);

    while(tmp_len) { 
        if(g_st_uart[uart_index].read_index == g_st_uart[uart_index].write_index) {
//					  printf("debug t1.\r\n");
//						delay_ms(200);
					  return copy_index;
				}
				else{
//            printf("debug t2.\r\n");    
            buffer[copy_index] = g_st_uart[uart_index].read_buf[g_st_uart[uart_index].read_index];
            copy_index++;
						tmp_len--;
            g_st_uart[uart_index].read_index++;
            if(256 <= g_st_uart[uart_index].read_index) {
                g_st_uart[uart_index].read_index = 0;
            }
        }
    }
    return copy_index;
}

int rt1020_dev_init(void)
{
    int i;
    memset(&g_st_rt1020_dev, 0, sizeof(g_st_rt1020_dev));
    for(i = 0; i < 10; i ++) {
        g_st_rt1020_dev.num_buf[i] = 255;
    }
    g_st_rt1020_dev.num_index = 9;
    g_st_rt1020_dev.current_num = 255;
    g_st_rt1020_dev.beat_num = 0;
    return OK;
}
int rt1020_uart_init(void)
{
    int retval;
    retval = uart_primary_init(RT1020_COM);
    if(OK != retval) {
        return ERROR;
    }
		g_st_rt1020_dev.status = UART1_STATUS_INIT;
    retval = gd_eval_com_init(RT1020_COM,RT1020_COM_ID);
    if(OK != retval) {
        return ERROR;
    }
		g_st_rt1020_dev.status = UART1_STATUS_CONFIG;
    return OK;
}
int rt1020_terminal_ready()
{
    g_st_rt1020_dev.status = UART1_STATUS_READY;
    return OK;
}
int rt1020_response_ok()
{
    g_st_rt1020_dev.status = UART1_STATUS_TRANSMITE;
    return OK;
}

void rt1020_send_task()
{
    uint8_t frame[32];
    MSG_PACKET_T packet;
    printf("rs232_send_task\r\n");


//    while(UART1_STATUS_READY != g_st_rt1020_dev.status) {
//        delay_ms(50);
//    }
//		
//		if(OK  != rt1020_response_ok()) {
//				printf("Error connect to pc.\r\n");        
//		}
		while(UART1_STATUS_TRANSMITE == g_st_rt1020_dev.status) {

				memset(frame, 0, sizeof(frame));
				if (1) {
						msg_frame_produce(&packet,  frame);

						if (ERROR==  uart_primary_write_noblock(RT1020_COM, frame, packet.length)) {
								printf("Error during UART1 send.\r\n");
							}
				}
		}
}

void rt1020_recv_task()
{
    int retval = OK;
    uint8_t buf[64];
    uint8_t ret_byte;

		memset(buf, 0, sizeof(buf));
		
//		if(LEAVE_VAL != retval) {
				retval = uart_primary_read_block(RT1020_COM, buf, 1);
				if( retval == ERROR) {
						printf("uart_primary_read_block 1st error\n");
				}else if (0 == buf[0]) {
//        	printf("uart_primary_read_block 1st ok\n");
				}          
//		} else {
//				buf[0] = ret_byte;
//		}
		retval = msg_get_from_uart(buf, &ret_byte, RT1020_COM, &g_st_rt1020_dev.receive_msg);
		if(OK > retval) {
//				printf("msg_get_from_uart error\n");
		}
		else  if(OK == retval) {
				retval = msg_issue(&g_st_rt1020_dev.receive_msg);
				if(ERROR == retval) {
						printf("msg_issue error\n");
				}
		}
		delay_ms(10);						
}

int msg_num_record(uint8_t record_num, uint8_t    *record_buf, uint8_t *record_index)
{
    
    (*record_index)++;
    
    if(*record_index >= 10) {
        *record_index = 0;
    }
    record_buf[*record_index] = record_num;

    return OK;
}

int msg_num_compare_with_record(uint8_t record_num, uint8_t    *record_buf)
{
    int i;
    for(i = 0; i < MSG_RECORD_BUF_LEN; i ++) {
        if(record_buf[i] == record_num)
            return ERROR;
    }
    return OK;
}

int msg_vaild(uint8_t packet_num, uint8_t *current_num_wifi)
{
    uint8_t  *current_num;
    
    current_num = current_num_wifi;
    if(*current_num > packet_num) {
        if(*current_num == 255) {
            *current_num = 0;
        }
        if(*current_num <= packet_num) {
            *current_num = packet_num;
        } else if((uint32_t)*current_num >= ((uint32_t)packet_num + 10)) {
            return ERROR;//drop
        } 
    } else if(*current_num < packet_num) {
        if(packet_num >= 246) {
            if((uint32_t)*current_num + 256 >= packet_num ) {
                return ERROR;//drop
            }
        } else {
            *current_num = packet_num;
        }
    }
    
    return OK;
}

int msg_frame_produce(MSG_PACKET_T   *p_packet,  uint8_t   *p_frame)
{
    int msg_len = 0;
    uint8_t crc_ret;
    if(NULL == p_frame || NULL == p_packet) {
        return ERROR;
    }
    p_frame[0] = p_packet->head & 0xff;
    p_frame[1] = (p_packet->head >> 8) & 0xff;
        
    *(uint16_t *)(p_frame+2) = p_packet->length;

    p_frame[4] = p_packet->set_ack;    
    p_frame[5] = p_packet->num;  
    p_frame[6] = p_packet->command;  
    p_frame[7] = p_packet->type; 
    
    memcpy(&p_frame[8], p_packet->u_content.content, p_packet->length - 11);
    msg_len = p_packet->length - 7;
    crc_ret = MSG_CRC_VALUE;
    getStrCrc8(&crc_ret, &p_frame[4], msg_len);
    p_frame[p_packet->length - 3] = crc_ret;
    
    p_frame[p_packet->length - 2] = p_packet->tail & 0xff;
    p_frame[p_packet->length - 1] = (p_packet->tail >> 8) & 0xff;

    return OK;
}

int msg_packet_produce(uint8_t set_ack, uint8_t num, uint8_t command, uint8_t type, uint8_t *p_content, uint8_t content_num, MSG_PACKET_T   *p_packet)
{
    if(NULL == p_packet) {
        return ERROR;
    }

    p_packet->head = MSG_HEAD_BYTE1 |(MSG_HEAD_BYTE2 << 8);
    p_packet->length = 11 + content_num;
    p_packet->set_ack = set_ack;
    p_packet->num = num;
    p_packet->command = command;
    p_packet->type = type;
    if(NULL != p_content &&  0 <= content_num) {
        memcpy(&p_packet->u_content.content, p_content, content_num);
    }

    p_packet->ecc = 0;
    p_packet->tail = MSG_TAIL_BYTE1 | (MSG_TAIL_BYTE2 << 8);

    return OK;
}

int msg_issue(MSG_PACKET_T  *packet)
{
	uint8_t dev_num;
	uint8_t data[64] = {0};
    packet->set_ack &=0x0f;
    packet->set_ack += (0 << 4);
	if(1) {
			dev_num = (packet->set_ack & 0xf0) >> 4;
			packet->set_ack &= 0x0f;
			if((packet->set_ack & 0x0f) == 0) {
					switch(packet->command) {
						case MSG_CMD_RTC:
							if(MSG_TYPE_RTC_SET == packet->type)
							{
								memcpy(data,packet->u_content.content,packet->length);
								for(uint8_t i =0;i < 7;i++)
									data[i] = BCD2BIN(data[i]);
								rtc_rx8010_set_time(data);
							}
							else if(MSG_TYPE_RTC_GET == packet->type)
							{
								rtc_time_ask_flag = true;
							}
							else if(MSG_TYPE_ALARM_POWERON_TIME_SET == packet->type)
							{
								memcpy(data,packet->u_content.content,packet->length);
								rtc_rx8010_set_alarm_cal(data);
								//printf("set_alarm:%d-%d-%d   %d:%d:0\n",data[5],data[4],data[0],data[1],data[2]);
								
								if(alarm_set_time_count == 0)
								{
									alarm_set_time_t[0] = data[5];//year
									alarm_set_time_t[1] = data[4];//month
									alarm_set_time_t[2] = data[0];//day
									alarm_set_time_t[3] = data[1];//hour
									alarm_set_time_t[4] = data[2];//min
									alarm_set_time_t[5] = 0;//sec
									
								}
								else
								{
									if((data[5] < alarm_set_time_t[0]) || ((data[5] == alarm_set_time_t[0]) && (data[4] < alarm_set_time_t[1]))\
									|| ((data[5] == alarm_set_time_t[0]) && (data[4] == alarm_set_time_t[1]) && (data[0] < alarm_set_time_t[2]))\
									|| ((data[5] == alarm_set_time_t[0]) && (data[4] == alarm_set_time_t[1]) && (data[0] == alarm_set_time_t[2]) && (data[1] < alarm_set_time_t[3]))\
									|| ((data[5] == alarm_set_time_t[0]) && (data[4] == alarm_set_time_t[1]) && (data[0] == alarm_set_time_t[2]) && (data[1] == alarm_set_time_t[3]) && (data[2] < alarm_set_time_t[4])))
									{
										alarm_set_time_t[0] = data[5];//year
										alarm_set_time_t[1] = data[4];//month
										alarm_set_time_t[2] = data[0];//day
										alarm_set_time_t[3] = data[1];//hour
										alarm_set_time_t[4] = data[2];//min
										alarm_set_time_t[5] = 0;//sec
									}
									
								}
								alarm_set_time[alarm_set_time_count][0] = data[5];//year
								alarm_set_time[alarm_set_time_count][1] = data[4];//month
								alarm_set_time[alarm_set_time_count][2] = data[0];//day
								alarm_set_time[alarm_set_time_count][3] = data[1];//hour
								alarm_set_time[alarm_set_time_count][4] = data[2];//min
								alarm_set_time[alarm_set_time_count][5] = 0;//sec
								alarm_set_time_count++;
								if(alarm_set_time_count == 40)
									alarm_set_time_count = 0;
//								printf("alarm_set_time_count:%d\r\n",alarm_set_time_count);
								rtc_alarm_set_flag = true;
							}
							else if(MSG_TYPE_ALARM_POWERON_TIME_GET == packet->type)
							{
								rtc_alarm_timeon_ask_flag = true;
							}
							else if(MSG_TYPE_ALARM_POWERON_TIME_SET_CANCEL == packet->type)
							{
								rx8010_alarm_irq_enable(g_rtc_dev.rtc,0);
								memset(alarm_set_time,0,sizeof(alarm_set_time));
								alarm_set_time_count = 0;
								rtc_alarm_set_flag = false;
//								printf("rtc_rx8010_set_alarm off.\n");
							}
							else if(MSG_TYPE_ALARM_POWEROFF_TIME_SET == packet->type)
							{
								memcpy(data,packet->u_content.content,packet->length);
								rtc_rx8010_set_alarm(data);								
							}
							else if(MSG_TYPE_ALARM_POWEROFF_TIME_GET == packet->type)
							{
								rtc_alarm_timeoff_ask_flag = true;
							}
							else if(MSG_TYPE_ALARM_POWEROFF_TIME_SET_CANCEL == packet->type)
							{
								rx8010_alarm_irq_enable(g_rtc_dev.rtc,0);
							}
							break;
						case MSG_CMD_ROBOT_STATUS:	
							if(MSG_TYPE_ROBOT_STATUS == packet->type)
							{
							
							}
							break;
						case MSG_CMD_HEARTBEAT:
							if(MSG_TYPE_HEARTBEAT_REPORT == packet->type)
							{
//								printf("heart_beat.\n");
								msg_robot_heartbeat_ack_report(packet->u_content.content,1);
							}
							break;
						case MSG_CMD_ROBOT:
							if(MSG_TYPE_RT1020_POWEROFF == packet->type)
							{
								gd32_poweroff_flag = true;
							}
							else if(MSG_TYPE_ROBOT_RESET == packet->type)
							{
								gd32_reset_flag = true;
							}
							else if(MSG_TYPE_ELEC_CURRENT == packet->type)
							{
								X86ECDET = packet->u_content.content[0];
//								if(packet->u_content.content[1] == 1)
//								  xHWSwitchDev.charge_status = CHARGE;
//								else if(packet->u_content.content[1] == 0)
//									xHWSwitchDev.charge_status = NOCHARGE;
//								printf("X86ECDET: %d \r\n",packet->u_content.content[0]);
							}
//							if(MSG_TYPE_RT1020_LOWPOWER_OFF == packet->type)
//							{
//								gd32_poweroff_flag = true;
//								gd32_lowpower_flag = true;
//							}
//							else if(MSG_TYPE_RT1020_CHARGE_ERROR == packet->type)
//							{
//								gd32_charge_protect_flag = true;
//							}
							break;
						case MSG_CMD_VERSION:
							if(MSG_TYPE_VERSION_ASK == packet->type)
							{
								gd32_version_report();
							}
							break;
						case MSG_CMD_IOTSET_RT1021:
							if(MSG_TYPE_IOTSET_RT1021 == packet->type)
							{
								if(0 == packet->u_content.content[0])
								{
									iot_setok_from_rt1020 = false;
//									printf("iot set error\r\n");
								}
								else if(1 == packet->u_content.content[0]) 
								{
									iot_setok_from_rt1020 = true;
									xNbiotDev.ucInitStatus = 1;
									xNbiotDev.ucConnectServerFlag = 1;
//									printf("iot set ok\r\n");
								}
							}
							break;
						default :
							printf("debug DEFAULT\r\n");
							break;
					}
			} else if((packet->set_ack & 0x0f) == 1) {
					switch(packet->command) {
						case MSG_CMD_ROBOT:
							if(MSG_TYPE_RT1020_POWEROFF == packet->type)
							{
								gd32_poweroff_flag = true;
							}
							else if(MSG_TYPE_ROBOT_RESET == packet->type)
							{
								gd32_reset_flag = true;
							}
							break;
						case MSG_CMD_RESET:
							if(MSG_TYPE_RESET_RCU == packet->type)
							{
								gd32_reset_report_flag = false;
							}
							break;
						default:
							break;
					}
			} 
	}
    return OK;
}

int msg_process( MSG_PACKET_T  *receive_msg, uint8_t  *buf, uint32_t len)
{
    int i;
    uint8_t crc_ret;    
    uint8_t  *p_tmp_buf = buf;
    
    if(NULL == buf || NULL == receive_msg) {
        return ERROR;
    }
    memset(receive_msg, 0, sizeof(MSG_PACKET_T));
    
    receive_msg->head = p_tmp_buf[0] | (p_tmp_buf[1] << 8);
    p_tmp_buf +=2;
    receive_msg->length = p_tmp_buf[0] | (p_tmp_buf[1] << 8);
    p_tmp_buf +=2;
    receive_msg->set_ack = p_tmp_buf[0];
    receive_msg->num= p_tmp_buf[1];
    receive_msg->command = p_tmp_buf[2];
    receive_msg->type = p_tmp_buf[3];       

    for(i = 0; i < (len - 11); i++) {
    receive_msg->u_content.content[i]= p_tmp_buf[4 + i];   
    }        
    receive_msg->ecc = p_tmp_buf[4 + i];
    receive_msg->tail = p_tmp_buf[5 + i] | (p_tmp_buf[6 + i] << 8);

    crc_ret = 0;
    getStrCrc8(&crc_ret, &p_tmp_buf[0], receive_msg->length - 7);      
  /* 
    printf("debug recv msg:head=%x,length=%x,set_ack=%x,num=%x,command=%x,type=%x,tail=%x.\r\n", \
                                receive_msg->head,receive_msg->length, receive_msg->set_ack, \
                                receive_msg->num, receive_msg->command, receive_msg->type,  \
                                receive_msg->tail);            
    printf("debug content=");
    for(i = 0; i < receive_msg->length  - 10; i ++) {
        printf("%x,", receive_msg->u_content.content[i]);
    }   
    printf("\r\n");

    printf("debug crc dst=%x,crc src=%x.\r\n", receive_msg->ecc, crc_ret);    */
    if((crc_ret == receive_msg->ecc) && (receive_msg->tail == (MSG_TAIL_BYTE1 | (MSG_TAIL_BYTE2 << 8)))) {
        return OK;            
    }
    return ERROR;
}

int msg_get_from_uart(uint8_t *p_buf, uint8_t    *byte_ret, uint32_t usart_periph, MSG_PACKET_T  *receive_msg)
{
    int retval;
    size_t received;
    uint16_t packet_len = 0;
    uint16_t last_packet_len = 0;
    int16_t tmp_packet_len = 0;
    uint8_t *p_tmp_buf = p_buf;
    
    /* check packet header */
    if (p_tmp_buf[0] != 0xaa) {
        return FAIL;
    }
    p_tmp_buf++;

    retval = uart_primary_read_block(usart_periph, p_tmp_buf,1);
    if ( (retval < 0)) {
        return retval;
    }

    if (p_tmp_buf[0] != 0xcc) {
        *byte_ret = p_tmp_buf[0];
        return LEAVE_VAL;
    }
    p_tmp_buf++;
    /* get packet's length */

    retval = uart_primary_read_block(usart_periph, p_tmp_buf, 2);
    if ( (retval < 0)) {
        return retval;
    }  

    packet_len = p_tmp_buf[0] | (p_tmp_buf[1] << 8);
    if (packet_len > MSG_MAX_PACKET_SIZE) {
        printf("Message length is longer than %d.\r\n", MSG_MAX_PACKET_SIZE);
        return FAIL;
    }
    p_tmp_buf += 2;
    /* get following packet data */
    last_packet_len = packet_len - 4;
    received = 0;
    tmp_packet_len = last_packet_len;
    do {
//				printf("debug_t3\n");
        p_tmp_buf += received;
        tmp_packet_len -= received;
		if(tmp_packet_len < 0)
			return FAIL;
//		printf("tmp_packet_len:%d\n",tmp_packet_len);
		retval = uart_primary_read_block(usart_periph, p_tmp_buf, tmp_packet_len);
		if ( (retval < 0)) {
//						printf("debug_t4\n");
				return retval;
		}else{
				received = retval;	
//						printf("debug_t5\n");					
		}
    } while(retval != 0);
    if(ERROR == msg_process(receive_msg, p_buf, packet_len)) {
//				printf("debug_t6\n");	
        return FAIL;
    }
    return OK;
}

int msg_dev_init(void)
{
    memset(&g_st_msg_protocol, 0, sizeof(g_st_msg_protocol));

    g_st_msg_protocol.num = 255;

    return OK;
}

int msg_init(void)
{
    int retval;
    
    retval =msg_dev_init();
    if(OK != retval) {
        return ERROR;
    }
    return OK;
}

int msg_report(MSG_PACKET_T *packet)
{
	uint8_t frame[64];
	msg_frame_produce(packet,  frame);
	if(ERROR == uart_primary_write_noblock(RT1020_COM, frame, packet->length))
	{
		printf("Error during UART1 send.\r\n");
	}
	return OK;
}

int msg_rtc_time_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

    msg_packet_produce(1, 0, MSG_CMD_RTC, MSG_TYPE_RTC_GET, (uint8_t *)data, length, &packet);
	msg_report(&packet);
	
    return OK;
}

int msg_rtc_alarm_report(uint8_t *data,uint8_t length,uint8_t uart_msg_type) 
{
    MSG_PACKET_T packet;
	if(MSG_TYPE_ALARM_POWERON_TIME_GET == uart_msg_type)
		msg_packet_produce(1, 0, MSG_CMD_RTC, MSG_TYPE_ALARM_POWERON_TIME_GET, (uint8_t *)data, length, &packet);
	else if (MSG_TYPE_ALARM_POWEROFF_TIME_GET == uart_msg_type)
		msg_packet_produce(1, 0, MSG_CMD_RTC, MSG_TYPE_ALARM_POWEROFF_TIME_GET, (uint8_t *)data, length, &packet);			
	msg_report(&packet);
    return OK;
}

int msg_robot_pwroff_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(1, 0, MSG_CMD_ROBOT, MSG_TYPE_ROBOT_POWEROFF, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_robot_heartbeat_ack_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(1, 0, MSG_CMD_HEARTBEAT, MSG_TYPE_HEARTBEAT_REPORT, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_robot_status_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_ROBOT_STATUS, MSG_TYPE_ROBOT_STATUS, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_x86_poweron_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_X86_POWERON, MSG_TYPE_X86_POWERON, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_iotset_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_IOTSET_GD32, MSG_TYPE_IOTSET_GD32, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_version1_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_VERSION, MSG_TYPE_VERSION_ACK1, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_version2_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_VERSION, MSG_TYPE_VERSION_ACK2, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_reset_rcu_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_RESET, MSG_TYPE_RESET_RCU, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_current_time_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_GD32_TIME, MSG_TYPE_CURRENT_TIME, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

int msg_gd32_alarm_time_report(uint8_t *data,uint8_t length) 
{
    MSG_PACKET_T packet;

	msg_packet_produce(0, 0, MSG_CMD_GD32_TIME, MSG_TYPE_ALARM_TIME, (uint8_t *)data, length, &packet);		
	msg_report(&packet);
    return OK;
}

