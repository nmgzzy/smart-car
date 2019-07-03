#ifndef __INIT_H_
#define __INIT_H_

#include "headfile.h"

typedef struct
{
    float p;
    float i;
    float d;
    float error;
    float preError;
    float integ;
    int16 intlimit;
    int16 errlimit;
    float deriv;
    float output;

}PID_t;

typedef struct{
    uint8 buzz;
    uint8 data_n;
    uint8 lost;
    uint8 stop;
    uint8 start;
    uint8 mode;            //1-debug
    uint8 spd_rate;        //1低速 2中速 3高速
    uint8 obstacle;
    uint8 En_std;
    uint8 En_spd;
    uint8 En_dir;
    uint8 broken_road;
    uint8 broken_road_last;
    uint8 circle;
    uint8 mode_switch;
}Flag_t;

void pidInit(void);
void speed_encoder_init(void);
void ADC_init(void);

extern Flag_t flag;
extern float camera_angle_p;
extern PID_t pid_angle[2], pid_speed[2], pid_dir[2], pid_yaw[2];
extern float pid_dir_pset[2], pid_dire_pset[2];
extern float pid_spd_set[2];
extern int   itestVal[3];
extern float ftestVal[8],testPar[3];

#endif