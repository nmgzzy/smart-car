//双单片机uart通信
//基于逐飞k60/lpc库
#include "communicate.h"

//public
uint8 com_receive_flag = 0;                  //收到数据标志位
uint8 com_receive_data[MT_DATA_NUM-5] = {0}; //收到的数据
uint8 com_format = 0;


//初始化UART
void communicate_init(void)
{
    uart_init(COM_UART, COM_UART_BAUD);
    uart_rx_irq_en(COM_UART);
}

//在接收中断调用
void communicate_uart_interrupt(void)
{
    static uint16 com_len=0;
    static uint8 com_receive[MT_DATA_NUM]={0};
    static uint32 com_receive_num = 0;
    static uint8 sum = 0;
    uint32 i;
    uart_getchar(COM_UART, &com_receive[com_receive_num]);
    com_receive_num++;
    //校验帧头-F0 A5
    if(com_receive_num == 1 && com_receive[0]!=0xF0) com_receive_num = 0;
    else if(com_receive_num == 2 && com_receive[1]!=0xA5)
    {
        if(com_receive[1]==0xF0)
        {
            com_receive_num = 1;
            com_receive[0]=0xF0;
        }
        else com_receive_num = 0;
    }
    else if(com_receive_num == 3) com_len = com_receive[com_receive_num-1];//记录数据长度
    else if(com_receive_num == com_len+4 && com_receive[com_receive_num-1] == sum)//结束标志校验
    {
        for(i=0; i<com_len; i++)
        {
            com_receive_data[i] = com_receive[i+3];//更新数据
        }
        com_receive_num = 0;
        com_receive_flag = 1;
        sum=0;
    }
    else if(com_receive_num == MT_DATA_NUM)//数组越界清零
    {
        com_receive_num = 0;
        sum=0;
    }
    else if(com_receive_num>3 && com_receive_num<com_len+4)//计算校验位
    {
        sum += com_receive[com_receive_num-1];
    }
}

//发送数据
void communicate_send(uint8* data, uint32 len)
{
    uint32 i;
    uint8 sum=0;
    for(i=0; i<len; i++)
    {
        sum += data[i];
    }
    uart_putchar(COM_UART, 0xF0);
    uart_putchar(COM_UART, 0xA5);
    uart_putchar(COM_UART, len);
    uart_putbuff(COM_UART, data, len);
    uart_putchar(COM_UART, sum);//校验位
}


void communicate_wait(uint8 format)
{
    while(com_receive_flag ==0 || com_format != format);
    com_receive_flag = 0;
}
