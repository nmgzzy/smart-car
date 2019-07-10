#ifndef _OTHER_H
#define _OTHER_H

#include "headfile.h"

#define buzz_on()  gpio_set(BUZZER_PIN, 1)
#define buzz_off() gpio_set(BUZZER_PIN, 0)
#define NUM_OF_PARAMETER 120

#define CoderDelay 16
#define CLKL0                    gpio_set(ENCODER_CLKL,0) //H2输出低电平
#define CLKL1                    gpio_set(ENCODER_CLKL,1) //H2输出高电平
#define CSnL0                    gpio_set(ENCODER_CSnL,0) //E7输出低电平
#define CSnL1                    gpio_set(ENCODER_CSnL,1) //E7输出高电平

#define CLKR0                    gpio_set(ENCODER_CLKR,0)    //A0输出低电平
#define CLKR1                    gpio_set(ENCODER_CLKR,1)   //A0输出高电平
#define CSnR0                    gpio_set(ENCODER_CSnR,0)    //A1输出低电平
#define CSnR1                    gpio_set(ENCODER_CSnR,1)    //A1输出高电平

#define DATEL                   gpio_get(ENCODER_DATEL)  //读取
#define DATER                   gpio_get(ENCODER_DATER)  //读取

void buzzer_control(void);
void data_read(uint8 n);
void data_save(uint8 n);
int16 Speed_Get(unsigned char select);
float Byte2Float(unsigned char* buf);
void Float2Byte(float *target, unsigned char *buf);
int Byte2Int(unsigned char* buf);
void Int2Byte(int *target, unsigned char *buf);



#endif