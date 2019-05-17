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
//#define _SWICH_
//#define _SEND_DATA_

uint16 set_time = 12;
uint32 time_count = 0;
uint32 swich_time = 0;

void PIT0_IRQHandler(void)
{
    PIT_FlAG_CLR(pit0);
    //uint8 buf[2];
    //uint16 distance = 1500;
    //pit_time_start(pit1);
    static uint8 cnt=0;//,i, barrier_cnt = 0;
    static int angle_out = 0, speed_out = 0, dir_out = 0;
    //uint8 send[24] = {0};
    time_count++;
    CounterDecrease();
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

    if(flag.mode == MODE_START)
    {
        /*if(time_count - swich_time < 750 && swich_time != 0)
            stepping_motor(Balance_mode == 1 ? 1: -1);
        else
            stepping_motor(0);*/
    }

    if(cnt%10 == 0)
    {
#ifdef _SEND_DATA_
        DataScope_send();
#endif
    }

    if(cnt >= 50)
    {
        //communicate_send(&Balance_mode, BLC_, 1);
        cnt = 0;
    }
    cnt++;
    //if(time_count > 0 && time_count <= 1000)
        //itestVal[0] += pit_get_us(pit1);
}

void CounterDecrease(void)
{

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
    Float2Byte(&t, &debug_buff[i]);             i+=4;

    vcan_sendware(debug_buff, i);
}


void UART5_RX_TX_IRQHandler(void)
{
    uint8 i;
    if(UART5->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        communicate_uart_interrupt();
        if(com_receive_flag == 1 && time_count > 0)
        {
            com_receive_flag = 0;
            /*if(com_format == ERR_)
            {
                for(i = 4; i > 0; i--)
                    image_error[i] = image_error[i-1];
                image_error[0] = Byte2Float(com_receive_data);
                if(myfabs(image_error[0] - image_error[1]) > 10)
                    image_error[0] = faverage(image_error, 5);
            }
            if(com_format == BUZ_)
            {
                //flag.buzz = com_receive_data[0];
            }
            if(com_format == ROD_)
            {
                flag.broken_road = com_receive_data[0];
            }
#ifdef _SWICH_
            if(com_format == SWC_ && flag.barrier == 0)
            {
                Balance_mode = Balance_mode == 0 ? 1 : 0;
                swich_time = time_count;
            }
#endif
            */
        }
    }
    if(UART5->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3中断执行函数
//  @return     void
//  @since      v1.0
//  Sample usage:               当UART3启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
/*void UART3_RX_TX_IRQHandler(void)
{
    uint8 i;
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        communicate_uart_interrupt();
        if(com_receive_flag == 1 && time_count > 0)
        {
            com_receive_flag = 0;
            if(com_format == ERR)
            {
                for(i = 4; i > 0; i--)
                    image_error[i] = image_error[i-1];
                image_error[0] = Byte2Float(com_receive_data);
                if(myfabs(image_error[0] - image_error[1]) > 10)
                    image_error[0] = faverage(image_error, 5);
            }
            if(com_format == BUZ)
            {
                flag.buzz = com_receive_data[0];
            }
        }
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}*/

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



