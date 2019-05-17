/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_IPS.c
 * @brief      		IPS 液晶驱动
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.2 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-12
 * @note
					IPS液晶接线定义：
					------------------------------------
						模块管脚            单片机管脚
						SCL                 A15
						SDA                 A16
						RES                 B17
						DC                  B16
						CS                  A14

						电源引脚
						BL  3.3V电源（背光控制引脚，也可以接PWM来控制亮度）
						VCC 3.3V电源
						GND 电源地

					------------------------------------

 ********************************************************************************************************************/

#ifndef _SEEKFREE_IPS_H
#define _SEEKFREE_IPS_H

#include "headfile.h"


//-----------------液晶大小定义------------------------------
#define IPS_SIZE        0           //1.14寸 IPS液晶
//#define IPS_SIZE        1           //1.3寸  IPS液晶(暂未推出)

#if  (0 == IPS_SIZE)
#define X_MAX   135
#define Y_MAX   240

#elif  (1 == IPS_SIZE)
#define X_MAX   240
#define Y_MAX   240

#else
#error "IPS_SIZE 定义错误"

#endif

/*#define WHITE         	 0xFFFF
#define BLACK         	 0x0000
#define BLUE         	 0x001F
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
*/
//-------常用颜色----------
//#define IPS_BRED    	0XF81F
//#define IPS_GRED    	0XFFE0    //灰色
//#define IPS_GBLUE   	0X07FF    //
#define IPS_BLACK   	0X0000    //黑色
#define IPS_WHITE   	0XFFFF    //白色
#define IPS_RED     	0XF800    //红色
#define IPS_GREEN   	0X0F01    //绿色
#define IPS_BLUE    	0X085F    //蓝色
#define IPS_PINK    	0XF81F    //粉紫
#define IPS_YELLOW    	0XFFE0    //黄色
#define IPS_CYAN    	0X07FF    //青色
#define IPS_ORANGE    	0XFBE0    //橙色

//定义写字笔的颜色
#define IPS_PENCOLOR    IPS_WHITE

//定义背景颜色
#define IPS_BGCOLOR     IPS_BLACK


//定义显示方向
//0 竖屏模式
//1 竖屏模式  旋转180
//2 横屏模式
//3 横屏模式  旋转180
#define IPS_DISPLAY_DIR 3

#if (0==IPS_DISPLAY_DIR || 1==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	X_MAX	//液晶X方宽度
#define IPS_Y_MAX	Y_MAX   //液晶Y方宽度

#elif (2==IPS_DISPLAY_DIR || 3==IPS_DISPLAY_DIR)
#define	IPS_X_MAX	Y_MAX	//液晶X方宽度
#define IPS_Y_MAX	X_MAX   //液晶Y方宽度

#else
#error "IPS_DISPLAY_DIR 定义错误"

#endif


void ips_init(void);
void ips_clear(uint16 color);
void ips_draw_point(uint16 x,uint16 y,uint16 color);
void ips_show_char(uint16 x,uint16 y,uint8 dat);
void ips_show_str(uint16 x,uint16 y,uint8 dat[]);
void ips_show_buff(uint16 x,uint16 y,uint8 dat[], uint8 n);
void ips_show_int8(uint16 x,uint16 y,int8 dat);
void ips_show_uint8(uint16 x,uint16 y,uint8 dat);
void ips_show_int16(uint16 x,uint16 y,int16 dat);
void ips_show_uint16(uint16 x,uint16 y,uint16 dat);
void ips_show_float(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum);
void ips_displayimage032(uint8 *p, uint16 width, uint16 height);
void ips_displayimage032_zoom(uint8 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips_displayimage7725(uint8 *p, uint16 width, uint16 height);
void ips_display_chinese(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color);

#endif
