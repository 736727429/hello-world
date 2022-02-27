#ifndef _UART_H_
#define _UART_H_

#include "gd32e230_usart.h"
#include "global.h"

/* definition for UART0, connected to USART0 */
#define USART0_CLK                     RCU_USART0

#define USART0_TX_PIN                  GPIO_PIN_9
#define USART0_RX_PIN                  GPIO_PIN_10

#define USART0_GPIO_PORT               GPIOA
#define USART0_GPIO_CLK                RCU_GPIOA
#define USART0_AF                      GPIO_AF_1

/* definition for UART1, connected to USART1 */
#define USART1_CLK                     RCU_USART1

#define USART1_TX_PIN                  GPIO_PIN_2
#define USART1_RX_PIN                  GPIO_PIN_3

#define USART1_GPIO_PORT               GPIOA
#define USART1_GPIO_CLK                RCU_GPIOA
#define USART1_AF                      GPIO_AF_1

#define MSG_HEAD_BYTE1   0xaa
#define MSG_HEAD_BYTE2   0xcc
#define MSG_TAIL_BYTE1   '\n'
#define MSG_TAIL_BYTE2   '\r'
#define MSG_MAX_PACKET_SIZE 64

#define MSG_RECORD_BUF_LEN  10
#define MSG_CRC_VALUE  0

typedef struct {
    uint8_t read_buf[256];
    uint16_t write_index;
    uint16_t read_index;
    uint16_t over_flow_num;
} UART_PRIMARY_DEV_T;

typedef enum {
    MSG_ACK_OK = 0,
    MSG_ACK_TIMEOUT,    
    MSG_ACK_REPEAT,   
    MSG_ACK_UNREACHABLE,
		MSG_ACK_NODEVICE,
    MSG_ACK_RESERVED
}RESPONSE_STATUS_E;


typedef enum {
    MSG_CMD_E524 =  'U',
    MSG_CMD_PLATE =  'B',
    MSG_CMD_VL53L0X =  'V',
    MSG_CMD_MOTOR =  'W',
    MSG_CMD_GYRO =  'G',
    MSG_CMD_STEER =  'M',
	MSG_CMD_BATTERY = 'C',
    MSG_CMD_LED_EVENT =  'L',
    MSG_CMD_HT7M21X6 =  'H',
    MSG_CMD_VERSION =  'V',
    MSG_CMD_OPT =  'T',
    MSG_CMD_HEARTBEAT =  'I',
    MSG_CMD_DANCE =  'D',
    MSG_CMD_INIT =  'i',
    MSG_CMD_RESET =  'r',
    MSG_CMD_RTC = 'K',
	MSG_CMD_ROBOT = 'P',
	MSG_CMD_ROBOT_STATUS = 'Q',
	MSG_CMD_X86_POWERON = 'R',
	MSG_CMD_IOTSET_GD32 = 'M',
	MSG_CMD_IOTSET_RT1021 = 'M',
	MSG_CMD_GD32_TIME = 't',	

    MSG_CMD_RESERVED =  '.'
} MSG_CMD_E;

typedef enum {
    MSG_TYPE_E524_INTERVAL_SET = 'A',
    MSG_TYPE_E524_REPORT = 'B',
    MSG_TYPE_E524_ONOFF_SET = 'C',
    MSG_TYPE_E524_STORE = 'E',
    MSG_TYPE_E524_SET_TT = 'F',
    MSG_TYPE_E524_CAL = 'D',
    MSG_TYPE_PLATE_REPORT = 'B',

    MSG_TYPE_VL53L0X_ONOFF_SET = 'A',
    MSG_TYPE_VL53L0X_INTERVAL_SET = 'B',
    MSG_TYPE_VL53L0X_REPORT = 'C',


    MSG_TYPE_MOTOR_SPEED_SET = 'A',
    MSG_TYPE_MOTOR_SPEED_REPORT = 'H',
    MSG_TYPE_MOTOR_DISTANCE_REPORT = 'E',
    MSG_TYPE_GYRO_REPORT = 'B',

    MSG_TYPE_STEER_LOCATION = 'B',
    MSG_TYPE_STEER_ACT = 'A',
    MSG_TYPE_STEER_LOCATION_WITH_SPEED = 'C',
	MSG_TYPE_STEER_HEAD = 'D',

    MSG_TYPE_LED_EVENT_REPORT = 'A',
    MSG_TYPE_LED_ACCESS_REPORT = 'B',

    MSG_TYPE_HT7M21X6_REPORT = 'A',
    MSG_TYPE_SOFT_VERSION = 'A',
    MSG_TYPE_HARD_VERSION = 'B',
    MSG_TYPE_OPT_INT = 'T',
    MSG_TYPE_OPT_STEER = 'S',
    MSG_TYPE_HEARTBEAT_INTERVAL = 'A',
    MSG_TYPE_HEARTBEAT_REPORT = 'B',

    MSG_TYPE_DANCE_START = 'F',
    MSG_TYPE_DANCE_STOP = 'G',
    MSG_TYPE_INIT_OK = 'G',
    MSG_TYPE_RESET =  'F',

	MSG_TYPE_RTC_SET = 'A',
	MSG_TYPE_RTC_GET = 'D',
	MSG_TYPE_ALARM_POWERON_TIME_SET = 'B',
	MSG_TYPE_ALARM_POWERON_TIME_GET = 'E',
	MSG_TYPE_ALARM_POWERON_TIME_SET_CANCEL = 'C',
	MSG_TYPE_ALARM_POWEROFF_TIME_SET = 'b',
	MSG_TYPE_ALARM_POWEROFF_TIME_GET = 'e',
	MSG_TYPE_ALARM_POWEROFF_TIME_SET_CANCEL = 'c',

	MSG_TYPE_ROBOT_POWERON = 'A',
	MSG_TYPE_ROBOT_RESET = 'C',
	MSG_TYPE_ROBOT_POWEROFF = 'D',
	MSG_TYPE_RT1020_POWEROFF = 'E',
	MSG_TYPE_RT1020_LOWPOWER_OFF = 'F',
	MSG_TYPE_RT1020_CHARGE_ERROR = 'G',

	MSG_TYPE_ROBOT_STATUS = 'A',
	
	MSG_TYPE_BATTERY = 'A',
	
	MSG_TYPE_X86_POWERON = 'A',
	
	MSG_TYPE_IOTSET_GD32 = 'A',
	MSG_TYPE_IOTSET_RT1021 = 'B',
	
	MSG_TYPE_VERSION_ASK = 'A',
	MSG_TYPE_VERSION_ACK1 = 'B',
	MSG_TYPE_VERSION_ACK2 = 'C',
	
	MSG_TYPE_ELEC_CURRENT = 'I',
	
	MSG_TYPE_RESET_RCU = 'B',
	
	MSG_TYPE_CURRENT_TIME = 'A',
	MSG_TYPE_ALARM_TIME = 'B',

    MSG_TYPE_RESERVED = '.'
} MSG_TYPE_E;


typedef struct {
    int l_distance;
    int r_distance;
}__attribute__((packed)) MOTOR_DISTANCE_REPORT_T;



typedef struct {
    uint8_t response;
    char    version[17];
}__attribute__((packed)) VERSION_RESPONSE_T;

typedef struct _MSG_PACKET{
    uint16_t head;
    uint16_t length;
    uint8_t set_ack;
    uint8_t num;
    uint8_t command;
    uint8_t type;
    union {
#ifdef MSG_DEBUG_FOR_TRANSMIT
    int32_t   test_buf[4];
#endif
    uint8_t content[18];
    }u_content ;
    uint8_t ecc;
    uint16_t tail;
}__attribute__((packed)) MSG_PACKET_T;

typedef enum {  
         UART1_STATUS_INIT = 0,
         UART1_STATUS_CONFIG,
         UART1_STATUS_READY,
         UART1_STATUS_RECONFIG,
         UART1_STATUS_TRANSMITE,

         UART1_STATUS_RESERVED
}UART1_STATUS_EM;

typedef struct
{
    uint8_t rs232_background_buffer[64];     
    MSG_PACKET_T  receive_msg;
    uint8_t num_buf[10];
    uint8_t num_index;
    uint8_t current_num;
		uint32_t usart_periph; 
    UART1_STATUS_EM status;
    uint16_t beat_num;
} UART1_TRANSMITE_ST;

typedef struct {
		uint16_t voltage;
		uint16_t current;
		uint8_t relativecharge;
		uint8_t status;
		uint8_t charge_status;

}BATTERY_REPORT_T;

typedef struct
{
    uint16_t num;
}MSG_PROTOCOL_T;

extern MSG_PROTOCOL_T g_st_msg_protocol;
extern  UART1_TRANSMITE_ST   g_st_rt1020_dev;
extern UART_PRIMARY_DEV_T  g_st_uart[2];
extern uint8_t X86ECDET;

void uart0_init();
void uart1_init();
int uart_primary_init(uint32_t usart_periph) ;
int uart_primary_write_noblock(uint32_t usart_periph, const uint8_t *buffer, uint32_t length);
int uart_primary_read_block(uint32_t usart_periph,  uint8_t *buffer, uint32_t length);
int rt1020_dev_init(void);
int rt1020_uart_init(void);
int rt1020_terminal_ready();
int rt1020_response_ok();
void rt1020_send_task(void);
void rt1020_recv_task(void);
int msg_num_record(uint8_t record_num, uint8_t    *record_buf, uint8_t *record_index);
int msg_num_compare_with_record(uint8_t record_num, uint8_t    *record_buf);
int msg_vaild(uint8_t packet_num, uint8_t *current_num_wifi);
int msg_frame_produce(MSG_PACKET_T   *p_packet,  uint8_t   *p_frame);
int msg_packet_produce(uint8_t set_ack, uint8_t num, uint8_t command, uint8_t type, uint8_t *p_content, uint8_t content_num, MSG_PACKET_T   *p_packet);
int msg_issue(MSG_PACKET_T  *packet);
int msg_get_from_uart(uint8_t *p_buf, uint8_t    *byte_ret, uint32_t usart_periph, MSG_PACKET_T  *receive_msg);
int msg_init(void);
void msg_process_task(void);
int msg_num_add(uint8_t * p_num)  ;
int msg_report(MSG_PACKET_T *packet);
int msg_rtc_time_report(uint8_t *data,uint8_t length) ;
int msg_rtc_alarm_report(uint8_t *data,uint8_t length,uint8_t uart_msg_type) ;
int msg_robot_pwroff_report(uint8_t *data,uint8_t length) ;
int msg_robot_heartbeat_ack_report(uint8_t *data,uint8_t length) ;
int msg_robot_status_report(uint8_t *data,uint8_t length) ;
int msg_x86_poweron_report(uint8_t *data,uint8_t length) ;
int msg_gd32_iotset_report(uint8_t *data,uint8_t length) ;
int msg_gd32_version1_report(uint8_t *data,uint8_t length) ;
int msg_gd32_version2_report(uint8_t *data,uint8_t length) ;
int msg_gd32_reset_rcu_report(uint8_t *data,uint8_t length) ;
int msg_gd32_current_time_report(uint8_t *data,uint8_t length);
int msg_gd32_alarm_time_report(uint8_t *data,uint8_t length);

#endif