/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr.c
 * @brief      		中断函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/

#include "isr.h"
//#define _SEND_DATA_

uint16 set_time = 50;
uint32 time_count = 0;
uint32 swich_time = 0, swich_time2 = 0;
uint8 obstacle_pix = 0;
uint8 broken_road_cnt = 0;
uint8 flag_broken_road_cnt = 0;
uint8 swich = 1;
uint8 line_cy = 0, line_width = 0;
float img_err = 0;

void PIT0_IRQHandler(void)
{
    PIT_FlAG_CLR(pit0);
    //pit_time_start(pit2);
    static uint8 cnt=0;
    static int angle_out = 0, speed_out = 0, dir_out = 0;
    time_count++;
    if(time_count>500*set_time && flag.mode != MODE_DEBUG)
    {
        flag.lost = 1;
        printLog("Time out");
    }

    angle_out = BalanceControl();
    speed_out = SpeedControl();
    dir_out = DirectionControl();
    motor_out(angle_out, speed_out, dir_out);
    buzzer_control();

    if(gpio_get(HALL_PIN) == 0 && time_count > stop_time*500 && flag.stop == 0)
    {
        if(flag.mode != MODE_DEBUG)
        {
            flag.stop = 1;
            printLog("Hall stop");
            int8 buff[20];
            sprintf(buff, ">> %.1f s \0", time_count/500.0);
            printLog(buff);
        }
    }

    ftestVal[0] = flag.obstacle*100;

#ifdef _SEND_DATA_
    if(cnt%5 == 0)
        DataScope_send();
#endif

    if(cnt >= 50)
    {
        cnt = 0;
    }
    cnt++;
    /*itestVal[1] = pit_get_us(pit2);
    if(time_count > 1000 && time_count <= 2000)
        itestVal[0] += itestVal[1];*/
}

void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
    uint8 cmdf[2] = {3,~3};//串口调试?使用的前命令
    uint8 cmdr[2] = {~3,3};//串口调试?使用的后命令
    uart_putbuff(DEBUG_UART,cmdf,2);//先发送前命令
    uart_putbuff(DEBUG_UART,wareaddr,waresize);//发送数据
    uart_putbuff(DEBUG_UART,cmdr,2);//发送后命令
}

void DataScope_send(void)
{
    uint8 i = 0;
    uint8 debug_buff[32];
    float t;
    t = time_count/2;

    Float2Byte(&ftestVal[0], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[1], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[2], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[3], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[4], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[5], &debug_buff[i]);   i+=4;
    Float2Byte(&ftestVal[6], &debug_buff[i]);   i+=4;
    Float2Byte(&t, &debug_buff[i]);             i+=4;

    vcan_sendware(debug_buff, i);
}


void UART5_RX_TX_IRQHandler(void)
{
    static uint8 br_cnt = 0;
    float t;
    if(UART5->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        communicate_uart_interrupt();
        if(com_receive_flag == 1 && time_count > 0)
        {
            com_receive_flag = 0;
            flag.broken_road_last = flag.broken_road;
            obstacle_pix = (uint8)(com_receive_data[0] * 0.8f + obstacle_pix * 0.2f);
            broken_road_cnt = (uint8)(com_receive_data[1] * 0.7f + broken_road_cnt * 0.3f);
            if(Balance_mode)
            {
                line_cy = com_receive_data[2];
                line_width = com_receive_data[3];
                if(line_cy != 0 && line_width != 0)
                {
                    t = (65-line_width<0)?0:(65-line_width)/2.0f;//83//75//65
                    img_err = 0.3f*img_err+0.7f*((56-line_cy>0)?56.0-line_cy+t:56.0-line_cy-t);
                    pid_img[Balance_mode].error = img_err;
                    pid_img[Balance_mode].deriv = pid_img[Balance_mode].error - pid_img[Balance_mode].preError[0];
                    pid_img[Balance_mode].preError[0] = pid_img[Balance_mode].error;
                    pid_img[Balance_mode].output = pid_img[Balance_mode].p * pid_img[Balance_mode].error + pid_img[Balance_mode].d * pid_img[Balance_mode].deriv;
                }
                else
                {
                    img_err = 0;
                }
            }
            if(broken_road_cnt > 170 && flag_broken_road_cnt < 10)
                flag_broken_road_cnt++;
            else if(flag_broken_road_cnt > 0)
                flag_broken_road_cnt--;
            if(flag_broken_road_cnt > 3)
            {
                flag.broken_road = 1;
                flag.buzz = 5;
            }
            else
                flag.broken_road = 0;
            if(swich)
            {
                if(flag.broken_road == 1 && flag.broken_road_last == 0
                   && (time_count - swich_time > 500*3 || (swich_time == 0 && time_count > 500))
                   && flag.obstacle == 0
                   && flag.mode != MODE_DEBUG)
                {
                    swich_time = time_count;
                    if(Balance_mode == 0)
                        Balance_mode = 1;
                    else
                        flag.mode_switch = 1;
                }
                if(flag.mode_switch == 1)
                {
                    br_cnt++;
                    if(br_cnt > 15)
                    {
                        br_cnt = 0;
                        flag.mode_switch = 0;
                        Balance_mode = 0;
                        swich_time2 = time_count;
                    }
                }
            }
        }
    }
    if(UART5->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}



/*
void PORTE_IRQHandler(void)
{
    if(!flag_inf)
        inf_ex_interrupt();
    PORTC_FLAG_CLR(INF_PIN);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA中断执行函数
//  @return     void
//  @since      v1.0
//  Sample usage:               当A口启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTA->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
	//PORTA_FLAG_CLR(A1);

}


void PORTC_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTC->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
	//PORTC_FLAG_CLR(C6);
    VSYNC();
}


void DMA0_IRQHandler(void)
{
	DMA_IRQ_CLEAN(DMA_CH0);
    row_finished();
}


中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了DMA0中断，然后就到下面去找哪个是DMA0的中断函数名称，找到后写一个该名称的函数即可
void DMA0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位


DMA0_IRQHandler
DMA1_IRQHandler
DMA2_IRQHandler
DMA3_IRQHandler
DMA4_IRQHandler
DMA5_IRQHandler
DMA6_IRQHandler
DMA7_IRQHandler
DMA8_IRQHandler
DMA9_IRQHandler
DMA10_IRQHandler
DMA11_IRQHandler
DMA12_IRQHandler
DMA13_IRQHandler
DMA14_IRQHandler
DMA15_IRQHandler
DMA_Error_IRQHandler
MCM_IRQHandler
FTFL_IRQHandler
Read_Collision_IRQHandler
LVD_LVW_IRQHandler
LLW_IRQHandler
Watchdog_IRQHandler
RNG_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
CAN0_ORed_Message_buffer_IRQHandler
CAN0_Bus_Off_IRQHandler
CAN0_Error_IRQHandler
CAN0_Tx_Warning_IRQHandler
CAN0_Rx_Warning_IRQHandler
CAN0_Wake_Up_IRQHandler
I2S0_Tx_IRQHandler
I2S0_Rx_IRQHandler
CAN1_ORed_Message_buffer_IRQHandler
CAN1_Bus_Off_IRQHandler
CAN1_Error_IRQHandler
CAN1_Tx_Warning_IRQHandler
CAN1_Rx_Warning_IRQHandler
CAN1_Wake_Up_IRQHandler
Reserved59_IRQHandler
UART0_LON_IRQHandler
UART0_RX_TX_IRQHandler
UART0_ERR_IRQHandler
UART1_RX_TX_IRQHandler
UART1_ERR_IRQHandler
UART2_RX_TX_IRQHandler
UART2_ERR_IRQHandler
UART3_RX_TX_IRQHandler
UART3_ERR_IRQHandler
UART4_RX_TX_IRQHandler
UART4_ERR_IRQHandler
UART5_RX_TX_IRQHandler
UART5_ERR_IRQHandler
ADC0_IRQHandler
ADC1_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
CMP2_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
CMT_IRQHandler
RTC_IRQHandler
RTC_Seconds_IRQHandler
PIT0_IRQHandler
PIT1_IRQHandler
PIT2_IRQHandler
PIT3_IRQHandler
PDB0_IRQHandler
USB0_IRQHandler
USBDCD_IRQHandler
ENET_1588_Timer_IRQHandler
ENET_Transmit_IRQHandler
ENET_Receive_IRQHandler
ENET_Error_IRQHandler
Reserved95_IRQHandler
SDHC_IRQHandler
DAC0_IRQHandler
DAC1_IRQHandler
TSI0_IRQHandler
MCG_IRQHandler
LPTimer_IRQHandler
Reserved102_IRQHandler
PORTA_IRQHandler
PORTB_IRQHandler
PORTC_IRQHandler
PORTD_IRQHandler
PORTE_IRQHandler
Reserved108_IRQHandler
Reserved109_IRQHandler
SWI_IRQHandler
*/



