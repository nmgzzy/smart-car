//双单片机uart通信
//基于逐飞k60库
#ifndef _COMMUNICATE_H
#define _COMMUNICATE_H

#include "headfile.h"

#define COM_UART        uart5
#define COM_UART_BAUD   115200//460800//921600
#define MT_DATA_NUM     25                //数组大小>=所需传输字节数+5

#define CMD_ 0x01
#define ERR_ 0x02
#define PAR_ 0x03
#define ANG_ 0x04
#define BUZ_ 0x05
#define BLC_ 0x06
#define DAT_ 0x07
#define ROD_ 0x08
#define SWC_ 0x09
#define BAR_ 0x0A

void communicate_init(void);
void communicate_uart_interrupt(void);
void communicate_send(uint8* data, uint8 format, uint32 len);
void communicate_wait(uint8 format);

extern uint8 com_receive_flag;                   //收到数据标志位
extern uint8 com_receive_data[MT_DATA_NUM-5];    //收到的数据
extern uint8 com_format;

#endif

