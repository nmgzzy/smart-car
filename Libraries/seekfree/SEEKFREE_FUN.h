/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_FUN.h
 * @brief      		扩展功能函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#ifndef _SEEKFREE_FUN_h
#define _SEEKFREE_FUN_h

#include "headfile.h"

float trapezoid_fun(uint32 x, uint16 up, uint16 t, uint16 down, float max);
float signal(int data);
float faverage(float* data, uint8 num);
int   myabs(int dat);
float myfabs(float dat);
void  my_delay(long t);
int   limit(int x, uint16 y);
float flimit(float x, uint16 y);
int   limit_ab(int x, int16 min, int16 max);
float flimit_ab(float x, int16 min, int16 max);











#endif
