#include "balance.h"

/*
float my_acc[3] = {1,1,1};
const float fRad2Deg = 57.29578f;

static void rectify_data(void)//把数据转化成角度
{
    my_acc[0] = mpu_acc_x / 16384.0f;
    my_acc[1] = mpu_acc_y / 16384.0f;
    my_acc[2] = mpu_acc_z / 16384.0f;
    my_gyro_angle = mpu_gyro_y / 16.384f;
    my_gyro_dir = mpu_gyro_x / 16.384f;//左负右正
}

float get_acc_angle(void)//计算acc角度
{
    float acc_angle = 1, temp = 1;
    static float acc_angle_last = 0;
    rectify_data();
    temp = sqrt(my_acc[1]*my_acc[1] + my_acc[2]*my_acc[2]);
    acc_angle = fRad2Deg * atan(temp / my_acc[0]);
    if(mpu_acc_z<0) acc_angle = -acc_angle;
    acc_angle = acc_angle*0.5f + acc_angle_last*0.5f;//滤波
    acc_angle_last = acc_angle;
    return acc_angle;
}

//-*************************************************
名称：void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
功能：陀螺仪数据与加速度计数据通过滤波算法融合
输入参数：
  	float angle_m   加速度计计算的角度
	float gyro_m    陀螺仪角速度
	float *angle_f  融合后的角度
	float *angle_dot_f  融合后的角速度
输出参数：滤波后的角度及角速度
返回值：无
//-**************************************************
static void kalman_filter1(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
{
    //------------------------------
    static float angle, angle_dot;
    const float Q_angle = 0.000001, Q_gyro = 0.0001, R_angle = 0.5, dt = 0.002;
    static float P[2][2]={
                       { 1, 0 },
                       { 0, 1 }
                     };
    static float Pdot[4] = {0, 0, 0, 0};
    const uint8 C_0 = 1;
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    //------------------------------
    angle += (gyro_m - q_bias) * dt;

    Pdot[0]  =Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    angle_err = angle_m - angle;

    PCt_0=C_0 * P[0][0];
    PCt_1=C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    angle	+= K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m - q_bias;

    *angle_f = angle;
    *angle_dot_f = angle_dot;
}


float acc_angle = 1;// pitch当前角度
float target_angle = 20;

int AngleControl(void)
{
    int angle_out = 0;
    float angle_k = 0, gyro_k = 0;//
    Get_AccData();                          //读取数据
    Get_Gyro();
    acc_angle = get_acc_angle();         //加速度计计算角度
    kalman_filter1(acc_angle, my_gyro_angle, &angle_k, &gyro_k);
    if((angle_k > 35 || angle_k < -33) && flag.mode != MODE_DEBUG && time_count>1000)
    {
        flag.lost = 1;
        OLED_P8x16Str(10,1,"angle_out");
    }

    pid_angle.error = target_angle - angle_k;
    pid_angle.output = pid_angle.p * pid_angle.error + pid_angle.d * (-gyro_k);
    angle_out = (int)pid_angle.output;

    return angle_out;
}

*/