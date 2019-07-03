#include "init.h"

Flag_t flag;
PID_t pid_angle[2], pid_speed[2], pid_dir[2], pid_yaw[2];
float camera_angle_p = 0;
float pid_dir_pset[2], pid_dire_pset[2];;
float pid_spd_set[2];
int   itestVal[3] = {0};
float ftestVal[8] = {0};
float testPar[3] = {0};

void pidInit(void)
{
    flag.buzz = 0;
    flag.data_n = 0;
    flag.lost = 0;
    flag.stop = 0;
    flag.start = 0;
    flag.mode = 0;            //1-debug 2-stand 3-start
    flag.spd_rate = 0;        //1低速 2中速 3高速
    flag.En_std = 1;
    flag.En_spd = 1;
    flag.En_dir = 1;
    flag.obstacle = 0;
    flag.broken_road = 0;
    flag.broken_road_last = 0;
    flag.mode_switch = 0;
    //---------------直立--------------------
    //角度pid
    pid_angle[0].p = 125;
    pid_angle[0].i = 0;//
    pid_angle[0].d = 2.3;

    //速度pid
    pid_speed[0].p = 18;
    pid_speed[0].i = 0.1;
    pid_speed[0].d = 0;
    pid_speed[0].intlimit = 1000;
    pid_speed[0].errlimit = (int16)(0.6f * target_speed_max[0]);
    pid_speed[0].integ = 0;

    //方向pid
    pid_dir_pset[0] = 10;
    pid_dir[0].p = pid_dir_pset[0];//
    pid_dir[0].i = 0;//
    pid_dir[0].d = 5;

    //方向角速度pid
    pid_yaw[0].p = 2.7;
    pid_yaw[0].i = 0;//
    pid_yaw[0].d = 3.4f;

    //---------------三轮--------------------
    //角度pid
    pid_angle[1].p = 30;
    pid_angle[1].i = 0;//
    pid_angle[1].d = 1;

    //速度pid
    pid_speed[1].p = 20;
    pid_spd_set[0] = pid_speed[1].p;
    pid_speed[1].i = 0.1;
    pid_spd_set[1] = pid_speed[1].i;
    pid_speed[1].d = 0.1;
    pid_speed[1].intlimit = 300;
    pid_speed[1].errlimit = (int16)(0.7f * target_speed_max[1]);
    pid_speed[1].integ = 0;

    //方向pid
    pid_dir_pset[1] = 22;
    pid_dir[1].p = pid_dir_pset[1];//
    pid_dir[1].i = 0;//
    pid_dir[1].d = 27.5;//

    //方向角速度pid
    pid_yaw[1].p = 2.7;
    pid_yaw[1].i = 0;//
    pid_yaw[1].d = 3.4f;
}

//初始化旋转编码器ftm计数
void speed_encoder_init(void)
{
    //////////////旋转编码器初始化       //Coder_init(); ////////////// //旋编初始化  （已检查）
    //--------------------左旋编------------------------//
    gpio_init(ENCODER_DATEL,GPI,1);//输入
    gpio_init(ENCODER_CLKL,GPO,1);//输出
    gpio_init(ENCODER_CSnL,GPO,1);//输出
    //--------------------右旋编-----------------------//
    gpio_init(ENCODER_DATER,GPI,1);//输入
    gpio_init(ENCODER_CLKR,GPO,1);//输出
    gpio_init(ENCODER_CSnR,GPO,1);//输出
}


void ADC_init(void)
{
    adc_init(ADC_LH);
    adc_init(ADC_LV);
    adc_init(ADC_LX);
    adc_init(ADC_MD);
    adc_init(ADC_RH);
    adc_init(ADC_RV);
    adc_init(ADC_RX);
    adc_init(OTHERAD);
    adc_init(VBAT_PIN);
}
