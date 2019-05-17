/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_IIC.h
 * @brief      		模拟IIC函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 * @note
					模拟IIC接线定义
					------------------------------------
						SDA                 C17
						SCL                 C16
					------------------------------------
 ********************************************************************************************************************/



#ifndef _SEEKFREE_IIC_h
#define _SEEKFREE_IIC_h



#include "headfile.h"



typedef enum IIC       //DAC模块
{
    IIC,
    SCCB
} IIC_type;



void  IIC_start(void);
void  IIC_stop(void);
void  IIC_ack_main(uint8 ack_main);
void  send_ch(uint8 c);
uint8 read_ch(uint8 send_ack);
void  simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat);
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type);
void  simiic_read_buf(uint8 dev_add, uint8 reg, IIC_type type, uint8 *buff, uint8 num);
void  IIC_init(void);

void  IIC_start2(void);
void  IIC_stop2(void);
void  IIC_ack_main2(uint8 ack_main);
void  send_ch2(uint8 c);
uint8 read_ch2(uint8 send_ack);
void  simiic_write_reg2(uint8 dev_add, uint8 reg, uint8 dat);
uint8 simiic_read_reg2(uint8 dev_add, uint8 reg, IIC_type type);
void  simiic_read_buf2(uint8 dev_add, uint8 reg, IIC_type type, uint8 *buff, uint8 num);
void  IIC_init2(void);









#endif

