/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr.h
 * @brief      		中断函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#ifndef _isr_h
#define _isr_h


#include "headfile.h"

void CounterDecrease(void);
void DataScope_send(void);
void stepping_motor(int8 turn);
//void PORTA_IRQHandler(void);
//void PORTB_IRQHandler(void);
void UART5_RX_TX_IRQHandler(void);

extern uint16 set_time;
extern uint32 time_count;
extern uint8 obstacle_cnt;
extern uint8 broken_road_cnt;

#endif
