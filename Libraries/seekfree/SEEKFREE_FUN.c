/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		SEEKFREE_FUN.c
 * @brief      		扩展功能函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#include "SEEKFREE_FUN.h"

float trapezoid_fun(uint32 x, uint16 up, uint16 t, uint16 down, float max)
{
    float y;
    if(x >= up && x <= up + t)
        y = max;
    else if(x < up)
        y = max*x/up;
    else if(x > up+t && x < up+t+down)
        y = max - max/down*(x-up-t);
    else
        y = 0;
    return y;
}

float signal(int data)
{
    if(data < 0)
        return -1;
    else if(data > 0)
        return 1;
    else
        return 0;
}

float faverage(float* data, uint8 num)
{
    float sum = 0;
    uint8 i;
    for(i = 0; i<num; i++)
    {
        sum += *(data+i);
    }
    return sum/num;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      绝对值函数
//  @param      dat				需要求绝对值的数
//  @return     int				返回绝对值
//  @since      v1.0
//  Sample usage:				dat = myabs(dat);//将dat变成正数
//-------------------------------------------------------------------------------------------------------------------
int  myabs(int dat)
{
    return dat>=0 ? dat : -dat;
}

float  myfabs(float dat)
{
    return dat>=0 ? dat : -dat;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      软件延时
//  @param      t				延时时间
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void my_delay(long t)
{
    uint8 i;
    while(t--)
    {
        i=98;
        while(i--);
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      限幅
//  @param      x				被限幅的数据
//  @param      y				限幅范围(数据会被限制在-y至+y之间)
//  @return     float			限幅之后的数据
//  @since      v1.0
//  Sample usage:				float dat = limit(500,300);//数据被限制在-300至+300之间  因此返回的结果是300
//-------------------------------------------------------------------------------------------------------------------
int limit(int x, uint16 y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

float flimit(float x, uint16 y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      双边限幅
//  @param      x				被限幅的数据
//  @param      a				限幅范围左边界
//  @param      b				限幅范围右边界
//  @return     float			限幅之后的数据
//  @since      v1.0
//  Sample usage:				int16 dat = limit_ab(500,-300，400);//数据被限制在-300至+400之间  因此返回的结果是400
//-------------------------------------------------------------------------------------------------------------------
int limit_ab(int x, int16 min, int16 max)
{
    if(x<min) x = min;
    else if(x>max) x = max;
    return x;
}

float flimit_ab(float x, int16 min, int16 max)
{
    if(x<min) x = min;
    else if(x>max) x = max;
    return x;
}

