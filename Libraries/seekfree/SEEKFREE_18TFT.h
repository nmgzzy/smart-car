/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_18TFT.h
 * @brief      		TFT液晶函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.2 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 * @note
					1.8 TFT液晶接线定义：
					------------------------------------
						模块管脚            单片机管脚
						SCL                 A15
						SDA                 A16
						RES                 B16
						DC                  B17
						CS                  A14

						电源引脚
						BL  3.3V电源（背光控制引脚，也可以接PWM来控制亮度）
						VCC 3.3V电源
						GND 电源地

					------------------------------------

 ********************************************************************************************************************/



#ifndef _SEEKFREE_18TFT_H
#define _SEEKFREE_18TFT_H

#include "headfile.h"


#define	TFT_X_MAX	128	//液晶X方宽度
#define TFT_Y_MAX	160 //液晶Y方宽度


#define DC_PIN		C19	//液晶命令位硬件定义
#define REST_PIN	D0 //液晶复位引脚定义


#define DC(x)   	gpio_set(DC_PIN,x);
#define REST(x) 	gpio_set(REST_PIN,x);

//-------常用颜色----------
#define TFT_RED     	0XF800    //红色
#define TFT_GREEN   	0X0F01    //绿色
#define TFT_BLUE    	0X085F    //蓝色

#define TFT_PINK    	0XF81F    //粉紫
#define TFT_YELLOW    	0XFFE0    //黄色
#define TFT_CYAN    	0X07FF    //青色
#define TFT_ORANGE    	0XFBE0    //橙色

#define TFT_BLACK   	0X0000    //黑色
#define TFT_WHITE   	0XFFFF    //白色


//定义写字笔的颜色
#define PENCOLOR TFT_RED

//定义背景颜色
#define BGCOLOR	 TFT_WHITE


void lcd_init(void);
void dsp_single_colour(int color);
void lcd_showchar(uint16 x,uint16 y,uint8 dat);
void lcd_showstr(uint16 x,uint16 y,uint8 dat[]);
void lcd_showint8(uint16 x,uint16 y,int8 dat);
void lcd_showint16(uint16 x,uint16 y,int16 dat);
void lcd_showuint8(uint16 x,uint16 y,uint8 dat);
void lcd_showuint16(uint16 x,uint16 y,uint16 dat);
void lcd_showimage(const unsigned char *p, uint8 x, uint8 y, uint8 width, uint8 height);
void lcd_showimage_bin(uint8 p[][51], uint8 x, uint8 y, uint8 width, uint8 height);
//void lcd_showpoint(uint8 x, uint8 y, int color);
//void lcd_showline(int16 xa, int16 ya, int16 xb, int16 yb, int color);
void displayimage032(uint8 *p);
void displayimage032_bin(uint8 *p);



#endif
