 #include "control.h"

#define _PROTECT_

//---------------平衡控制---------------
EulerAngleTypedef      CarAttitude;            /////姿态角
EulerAngleTypedef      CarAttitudeRate;        /////姿态角速度
float AccZAngle = 0;
float target_angle[2] = {0, -24.5};

uint8 Balance_mode = 0; //0-直立 1-三轮

int BalanceControl(void)
{
    //static float camera_angle, camera_angle_last;
    //uint8 offset = 0;
    static uint8 cnt = 0;
    int angle_out = 0;
    cnt++; if(cnt >= 10) cnt = 0;

    //if(cnt)
        BMX055_DataRead(&Q_raw, 0);
    //else
        //BMX055_DataRead(&Q_raw, 1);

    Gyro.Xdata = (Q_raw.GyroX) * 0.030517578f;// - GyroOffset.Xdata   //  1000/32768
    Gyro.Ydata = (Q_raw.GyroY) * 0.030517578f;// - GyroOffset.Ydata
    Gyro.Zdata = (Q_raw.GyroZ) * 0.030517578f;// - GyroOffset.Zdata
    Acc.Xdata = Q_raw.AccX * 0.000976563f;    //   2/2048
    Acc.Ydata = Q_raw.AccY * 0.000976563f;
    Acc.Zdata = Q_raw.AccZ * 0.000976563f;

    if(time_count == 1)
    {
        Quaternion_init();
    }

    Attitude_UpdateGyro();                /////快速更新
    Attitude_UpdateAcc();                 /////深度融合更新

    CarAttitude.Pitch = -EulerAngle.Roll / PI * 180;            ////转为角度   俯仰角
    CarAttitude.Roll = EulerAngle.Pitch / PI * 180;             ////翻滚角
    CarAttitude.Yaw = EulerAngle.Yaw / PI * 180;                ////偏航角
    CarAttitudeRate.Pitch = -EulerAngleRate.Roll / PI * 180;   ////俯仰角速度
    CarAttitudeRate.Yaw = EulerAngleRate.Yaw / PI * 180;       ////偏航角速度

    float AccZ, AccZAdjust;
    AccZ = -Acc.Zdata;
    if (AccZ > 1)
        AccZ = 1;
    if (AccZ < -1)
        AccZ = -1;
    AccZAngle = asinf(AccZ) * 180 / PI;
    AccZAdjust = (AccZAngle - CarAttitude.Pitch);
    CarAttitude.Pitch += (-Gyro.Xdata + AccZAdjust) * PERIODS;

    //摄像头画面控制
    /*camera_angle = CarAttitude.Pitch;
    camera_angle = flimit_ab(camera_angle,-25,10);
    camera_angle = 0.2f*camera_angle + 0.8f*camera_angle_last;
    camera_angle_last = camera_angle;

    if(camera_angle < -24) offset = 0;
    else if(camera_angle > 0) offset = 29;
    else offset = 29-(uint8)(-camera_angle*1.208333);

    //if(cnt%5==0) communicate_send(&offset, ANG_, 1);*/

    if((CarAttitude.Pitch > 60 || CarAttitude.Pitch < -110 || myfabs(CarAttitude.Roll) > 30)
       && flag.mode != MODE_DEBUG && time_count>1000)
    {
        flag.lost = 1;
        printLog("Angle out of range");
    }

    if(Balance_mode == 0)//直立
    {
        pid_angle[Balance_mode].error = target_angle[Balance_mode] - CarAttitude.Pitch;
        pid_angle[Balance_mode].output = pid_angle[Balance_mode].p * pid_angle[Balance_mode].error
            + pid_angle[Balance_mode].d * (-CarAttitudeRate.Pitch);
    }
    else if(Balance_mode == 1)//三轮
    {
        pid_angle[Balance_mode].error = CarAttitude.Pitch - target_angle[Balance_mode];
        if(pid_angle[Balance_mode].error < 0)
            pid_angle[Balance_mode].error = 0;
        pid_angle[Balance_mode].output = -pid_angle[Balance_mode].p * pid_angle[Balance_mode].error * pid_angle[Balance_mode].error
            + pid_angle[Balance_mode].d * (-CarAttitudeRate.Pitch);
    }
    angle_out = (int)pid_angle[Balance_mode].output;

    if(flag.En_std == 0)
        angle_out = 0;

    return angle_out;
}

//---------------平衡控制-以上-------------------



//----------------方向控制---------------------
uint16 ad_data_now[NUM_OF_AD] = {0};

float k_hv[2] = {8, 8};//横竖电感差比例
float k_md[2] = {0.3, 0.3};//中间电感比例
float k_adc = 1.0f;
float k_ke[2] = {10.0f, 10.0f};
//float image_error[5] = {0};

void ADC_get(void)
{
    uint8 i,j;
    uint16 ad_raw_data_now[NUM_OF_AD]={0};
    static uint16 ad_raw_data_pre[NUM_OF_AD][5]={0};
#define SAMPLING_NUM 3
    for(i = 0; i < SAMPLING_NUM; i++)
    {
        ad_raw_data_now[LH] += adc_once(ADC_LH,ADC_12bit);//k_adc *
        ad_raw_data_now[RH] += adc_once(ADC_RH,ADC_12bit);
        ad_raw_data_now[LV] += adc_once(ADC_LV,ADC_12bit);
        ad_raw_data_now[RV] += adc_once(ADC_RV,ADC_12bit);
        ad_raw_data_now[LX] += adc_once(ADC_LX,ADC_12bit);
        ad_raw_data_now[RX] += adc_once(ADC_RX,ADC_12bit);
        ad_raw_data_now[MD] += adc_once(ADC_MD,ADC_12bit);
    }
    ad_raw_data_now[LH] /= SAMPLING_NUM;
    ad_raw_data_now[RH] /= SAMPLING_NUM;
    ad_raw_data_now[LV] /= SAMPLING_NUM;
    ad_raw_data_now[RV] /= SAMPLING_NUM;
    ad_raw_data_now[LX] /= SAMPLING_NUM;
    ad_raw_data_now[RX] /= SAMPLING_NUM;
    ad_raw_data_now[MD] /= SAMPLING_NUM;

    //记录原始数据
    for(i = 0; i < NUM_OF_AD; i++)
    {
        for(j = 4; j > 0  ; j--)
        {
            ad_raw_data_pre[i][j] = ad_raw_data_pre[i][j-1];
        }
        ad_raw_data_pre[i][0] = ad_raw_data_now[i];
    }

    //滑动加权滤波
    for(i = 0; i < NUM_OF_AD; i++)
    {
        ad_data_now[i] = (uint16)(0.45f*ad_raw_data_pre[i][0] + 0.25f*ad_raw_data_pre[i][1] +
            0.15f*ad_raw_data_pre[i][2] + 0.1f*ad_raw_data_pre[i][3] + 0.05f*ad_raw_data_pre[i][4]);
    }
}

float k_circle[5] = {1,1,1,1,1};//入环偏差系数
int8 circle_offset[5] = {20,20,20,20,20};
uint8 cl_num = 3;//入环偏差系数
uint16 cl_out_delay = 400, cl_time = 40;//环参数
int16 circle_time_count[5] = {0};
uint8 crcl_cnt = 1;

float ErrorCalculate(uint8 mode)
{
    float error1;//, error2, error3, errorH, errorV, errorVH
    float error_out;//, circle_val_dev
    //static float yaw_integ = 0;//, circle_val[5], mid[3]
    static int16 difX_div[5] = {0};
    int16 difH, difV, difX;//, a, b;
    uint16 sumH, sumV, sumX, sumHM, sumHM2;
    //float error_a = 0;
    //int8 sgn = 0;
    //float error_offset = 0;
    ADC_get();
    difH = ad_data_now[LH]-ad_data_now[RH];
    difV = ad_data_now[LV]-ad_data_now[RV];
    difX = ad_data_now[LX]-ad_data_now[RX];
    sumH = ad_data_now[LH]+ad_data_now[RH];
    sumV = ad_data_now[LV]+ad_data_now[RV];
    sumX = ad_data_now[LX]+ad_data_now[RX];
    sumHM = sumH + ad_data_now[MD] + 1;
    sumHM2 = (uint16)(sumH + k_md[mode]*ad_data_now[MD] + 1);


    error1 = k_ke[mode] * (k_hv[mode]*difH + (10.0f-k_hv[mode])*difV)/sumHM2;
    error_out = error1;
    //errorH = k_ke[mode] * difH/sumH;
    //errorV = k_ke[mode] * difV/sumV;
    //errorVH = k_ke[mode] * difV/sumH;

    //ftestVal[1] = difH;             /////////////////////////
    //ftestVal[2] = difV;             ///////////////////////
    ftestVal[3] = difX;            /////////////////////////
    //ftestVal[4] = sumH + ad_data_now[MD]-3000;             ///////////////////////
    //ftestVal[5] = sumV;              /////////////////////////
    //ftestVal[6] = sumX;             ///////////////////////
    //ftestVal[7] = ad_data_now[MD];   /////////////////////////
    ftestVal[1] = sumH + ad_data_now[MD]-3000;         ///////////////////////
    ftestVal[2] = ad_data_now[MD];         ///////////////////////
    ftestVal[4] = 0;        ///////////////////////
    if(Balance_mode == 0)
    {
        //--------------检环-------------------
        if(time_count>1000 && sumHM > 3000 && ad_data_now[MD] > 700 && flag.circle <= 1)
        {
            ftestVal[4] = 1000;        ///////////////////////
            flag.circle = 1;
            for(uint8 i=4; i>0; i--)
                difX_div[i] = difX_div[i-1];
            difX_div[0] = (int16)(difX*0.5f + difX_div[0]*0.5f);
            if((difX_div[0] - difX_div[2])*(difX_div[2] - difX_div[4]) < 0)
            {
                flag.buzz = 3;
                ftestVal[5] = 2000;        ///////////////////////
            }
        }
        //if()


        if(flag.circle > 0)
        {
            circle_time_count[0]++;
            if(circle_time_count[0] > 4*500)
            {
                circle_time_count[0] = 0;
                flag.circle = 0;
            }
        }
        //--------------检环结束------------------
    }
    else if(Balance_mode == 1)
    {
        //--------------检环-------------------
        if(time_count>1000 && sumHM > 5700 && ad_data_now[MD] > 2200 && flag.circle <= 1)
        {
            ftestVal[4] = 1000;        ///////////////////////
            flag.circle = 1;
            for(uint8 i=4; i>0; i--)
                difX_div[i] = difX_div[i-1];
            difX_div[0] = (int16)(difX*0.5f + difX_div[0]*0.5f);
            if((difX_div[0] - difX_div[2])*(difX_div[2] - difX_div[4]) < 0)
            {
                flag.buzz = 3;
                ftestVal[5] = 2000;        ///////////////////////
            }
        }
        //if()


        if(flag.circle > 0)
        {
            circle_time_count[0]++;
            if(circle_time_count[0] > 4*500)
            {
                circle_time_count[0] = 0;
                flag.circle = 0;
            }
        }
        //--------------检环结束------------------
    }

    /*if(Balance_mode == 0)
    {
        //--------------检环-------------------
        if(time_count>1000 && sumHM > 3000 && ad_data_now[MD] > 500)
        {
            if(flag.circle < 2 && flag.circle_out==0)
            {
                flag.circle = 1;
                mid[2] = mid[1];
                mid[1] = mid[0];
                mid[0] = (mid[1]+mid[2]+ad_data_now[MD])/3;
                circle_val_dev = myfabs(mid[0] - mid[2]);

                //ftestVal[2] = circle_val_dev*10;                  ///////////////////////
                //ftestVal[3] = myfabs(difV*100/sumHM)*10;          ///////////////////////

                if(ad_data_now[MD] > 1500 && circle_val_dev < 30 && myfabs(difH*100/sumHM) < 15)//
                {
                    flag.circle = 2;
                    yaw_integ = 0;
                    crcl_cnt++;
                    if(crcl_cnt>cl_num) crcl_cnt = 1;
                    flag.buzz = 1;
                    tc[0] = time_count;
                    tc[1] = tc[0] + cl_time;
                    tc[2] = tc[1] + 200;
                    tc[3] = tc[2] + cl_time;
                }
            }
        }
        else if(flag.circle != 2)
        {
            flag.circle = 0;
        }
        //--------------检环结束------------------

        //--------------进环-------------------
        if(flag.circle == 1)
        {
            error_out = 4.5*k_ke[mode]*((difH+difV) /(sumH + 1));
        }
        else if(flag.circle == 2)
        {
            yaw_integ += CarAttitudeRate.Yaw*0.002f;
            error2 = k_ke[mode] * difV/(sumH + 0.3f*ad_data_now[MD] + 1);
            error3 = 4.5*k_ke[mode]*((difH+difV) /(sumH + 1));
            if(time_count < tc[1])
            {
                error_out = (k_circle[0] * error2 + circle_offset[0])*(time_count - tc[0])/(tc[1] - tc[0])
                    + error3*(tc[1] - time_count)/(tc[1] - tc[0]);
            }
            else if(time_count < tc[2])
            {
                error_out = k_circle[0] * error2 + circle_offset[0];

            }
            else if(time_count < tc[3])
            {
                error_out = (k_circle[0] * error2 + circle_offset[0])*(time_count - tc[2])/(tc[3] - tc[2])
                    + error1*(tc[3] - time_count)/(tc[3] - tc[2]);
            }
            else
            {
                error_out = error1;
            }
            //pid_dir.p = 0.7f * pid_dir_pset;
            if(yaw_integ > 330 || yaw_integ < -330)
            {
                flag.circle = 0;
                flag.circle_out = 1;
                yaw_integ = 0;
                flag.buzz = 3;
                tc[4] = time_count + 500;
            }
            if(time_count == tc[0]+1500 && flag.circle > 0)//长时间未检测到出环自动清除标志位
            {
                flag.circle = 0;
                yaw_integ = 0;
                flag.buzz = 2;
            }
        }
        if(flag.circle_out == 1)
        {
            if(time_count == tc[4])
            {
                flag.circle_out = 0;
            }
            a = limit_ab(ad_data_now[LH]-ad_data_now[LV], 0, 3000);
            b = limit_ab(ad_data_now[RH]-ad_data_now[RV], 0, 3000);
            error_out = flimit(k_ke[mode]*testPar[1]*(a-b)/(sumH + k_md[mode]*ad_data_now[MD] + 1),30);

        }
        //--------------进环结束-------------------


        *ftestVal[1] = error1;
        error2 = k_ke[mode]*(9*difH + 1*difV)
                    / (sumH + 0.5f*ad_data_now[MD] + 1);
        ftestVal[2] = error2;
        error3 = k_ke[mode]*(4*difH + 4*difV)
                    / (0.8f*sumH + 0.5*ad_data_now[MD] + 1);
        ftestVal[3] = error3;
        if(time_count>1000 && sumHM > 3000)
        {
            if(flag_circle < 2 && flag_circle_out==0)
            {
                flag_circle = 1;
                circle_val = (sumHM-3000)/10.0f*sumHM/(myfabs(difH)+1);
                if(circle_val > 500)
                {
                    tc[0] = time_count;
                    flag_circle = 2;
                    crcl_cnt++;
                    if(crcl_cnt>cl_num) crcl_cnt = 1;
                    flag.buzz = 1;
                }
            }
        }
        else
        {
            flag_circle = 0;
        }

        if(flag_circle == 1)
        {
            error_out = error2;
        }
        else if(flag_circle == 2)
        {
            yaw_integ += CarAttitudeRate.Yaw*0.002f;
            if(tc[0] - time_count > 0 && tc[0] - time_count < 3.14f*50)//circle_offset[crcl_cnt-1]
                error_offset = signal(difH+difV) * sin((float)(tc[0] - time_count)/50)*50;//k_circle[crcl_cnt-1]
            else
                error_offset = 0;
            error_out = error3 + error_offset;

            if(time_count < tc[0]+400)
            {
                if(time_count < tc[0]+cl_time)
                {
                    error_out = (signal(difH+difV)*circle_offset[crcl_cnt-1]* k_circle[crcl_cnt-1]
                                 * (time_count-tc[0]) + error2 *(cl_time-time_count+tc[0]))
                                    /(float)cl_time;
                }
                if(time_count >= tc[0]+cl_time && time_count < tc[0]+cl_time+10)
                {
                    error_out = signal(difH+difV)*circle_offset[crcl_cnt-1]* k_circle[crcl_cnt-1];
                }
                else if(time_count >= tc[0]+cl_time+10 && time_count < tc[0]+400)
                {
                    error_out = (error1 *(time_count-tc[0]-cl_time-10)
                                 + signal(difH+difV)*circle_offset[crcl_cnt-0]* k_circle[crcl_cnt-1]
                                     * (400-time_count+tc[0]))/(float)(400-cl_time-10);
                }
            }*
            //pid_dir.p = 0.7f * pid_dir_pset;
            if(yaw_integ > 330 || yaw_integ <-330)
            {
                flag_circle = 0;
                flag_circle_out = 1;
                yaw_integ = 0;
                //flag.buzz = 3;
                tc[1] = time_count;
            }
            if(time_count == tc[0]+1500 && flag_circle > 0)//长时间未检测到出环自动清除标志位
            {
                flag_circle = 0;
                yaw_integ = 0;
                //flag.buzz = 2;
            }
        }
        if(flag_circle_out == 1)
        {
            if(time_count < tc[1]+cl_out_delay)
            {
                error_out = error2;
                //pid_dir.p = 0.7f * pid_dir_pset;
            }
            if(time_count == tc[1]+cl_out_delay)
            {
                flag_circle_out = 0;
            }
        }*
    }
    else if(Balance_mode == 1)
    {
        //--------------检环-------------------
        if(time_count>1000 && sumHM > 4000 && ad_data_now[MD] > 2700)
        {
            if(flag.circle < 2 && flag.circle_out==0)
            {
                flag.circle = 1;
                mid[2] = mid[1];
                mid[1] = mid[0];
                mid[0] = (mid[0]+ad_data_now[MD])/2;
                circle_val_dev = myfabs(mid[0] - mid[2]);
                if(ad_data_now[MD] > 3500 && circle_val_dev < 10)
                {
                    //tc[0] = time_count;
                    //flag_circle = 2;
                    ///crcl_cnt++;
                    //if(crcl_cnt>cl_num) crcl_cnt = 1;
                    flag.buzz = 1;
                }
            }
        }
        else
        {
            flag.circle = 0;
        }
        //--------------检环结束------------------

    }*/
    return error_out;
}


void CounterDecrease(void)
{

}

uint16 barrier_time = 0;
uint16 barrier_turn_t[2] = {70,80};
uint16 barrier_turn_k[2] = {400,360};
uint16 barrier_delay[2]  = {95,110};
uint16 barrier_delay_out[2] = {90,80};
int8 barrier_turn_dir = -1;
uint16 bt[2][8];

int DirectionControl(void)
{
    int dir_out = 0;
    static int dir_out_last = 0;
    float E_error = 0;
    float error_offset = 0;
    E_error = ErrorCalculate(Balance_mode);
    if(time_count == 1)
    {
        bt[0][0] = 0;                               bt[1][0] = 0;
        bt[0][1] = barrier_turn_t[0];               bt[1][1] = barrier_turn_t[1];
        bt[0][2] = bt[0][1] + barrier_delay[0];     bt[1][2] = bt[1][1] + barrier_delay[1];
        bt[0][3] = bt[0][2] + barrier_turn_t[0];    bt[1][3] = bt[1][2] + barrier_turn_t[1];
        bt[0][4] = bt[0][3] + barrier_delay_out[0]; bt[1][4] = bt[1][3] + barrier_delay_out[1];
        bt[0][5] = bt[0][4] + barrier_turn_t[0];    bt[1][5] = bt[1][4] + barrier_turn_t[1];
        bt[0][6] = bt[0][5] + barrier_delay[0];     bt[1][6] = bt[1][5] + barrier_delay[1];
        bt[0][7] = bt[0][6] + barrier_turn_t[0];    bt[1][7] = bt[1][6] + barrier_turn_t[1];
    }
    flag.barrier = 0;
    if(flag.barrier)
    {
        barrier_time++;
        if(barrier_time > bt[Balance_mode][0] && barrier_time < bt[Balance_mode][1] || barrier_time > bt[Balance_mode][6] && barrier_time < bt[Balance_mode][7])
            error_offset =  barrier_turn_dir * barrier_turn_k[Balance_mode];
        else if(barrier_time > bt[Balance_mode][2] && barrier_time < bt[Balance_mode][3] || barrier_time > bt[Balance_mode][4] && barrier_time < bt[Balance_mode][5])
            error_offset = -barrier_turn_dir * barrier_turn_k[Balance_mode];
        else
            error_offset = 0;
        if(barrier_time > bt[Balance_mode][7])
        {
            flag.barrier = 0;
        }
        //角速度内环---------
        pid_yaw[Balance_mode].error = CarAttitudeRate.Yaw + error_offset;
        pid_yaw[Balance_mode].deriv = pid_yaw[Balance_mode].error - pid_yaw[Balance_mode].preError;
        pid_yaw[Balance_mode].preError = pid_yaw[Balance_mode].error;
        pid_yaw[Balance_mode].output = pid_yaw[Balance_mode].p * pid_yaw[Balance_mode].error + pid_yaw[Balance_mode].d * pid_yaw[Balance_mode].deriv;
        dir_out = (int)pid_yaw[Balance_mode].output;
    }
    else
    {
        barrier_time=0;
    //方向pid--------------------
        //偏差外环---------
        pid_dir[Balance_mode].error = E_error;
        pid_dir[Balance_mode].deriv = pid_dir[Balance_mode].error - pid_dir[Balance_mode].preError;
        pid_dir[Balance_mode].preError = pid_dir[Balance_mode].error;
        pid_dir[Balance_mode].output = pid_dir[Balance_mode].p * pid_dir[Balance_mode].error + pid_dir[Balance_mode].d * pid_dir[Balance_mode].deriv;
        //角速度内环---------
        pid_yaw[Balance_mode].error = pid_dir[Balance_mode].output + CarAttitudeRate.Yaw;
        pid_yaw[Balance_mode].deriv = pid_yaw[Balance_mode].error - pid_yaw[Balance_mode].preError;
        pid_yaw[Balance_mode].preError = pid_yaw[Balance_mode].error;
        pid_yaw[Balance_mode].output = pid_yaw[Balance_mode].p * pid_yaw[Balance_mode].error + pid_yaw[Balance_mode].d * pid_yaw[Balance_mode].deriv;
        dir_out = (int)pid_yaw[Balance_mode].output;
    }
    //输出限幅
    if(flag.En_dir == 0)
    {
        dir_out = 0;
    }
    else
    {//出界保护
#ifdef _PROTECT_
        if((ad_data_now[LH]>100 || ad_data_now[MD]>200 || ad_data_now[RH]>100) && flag.start==0)
        {
            flag.start = 1;
        }
        if(ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]<120 && flag.En_dir == 1 && flag.mode != MODE_DEBUG && flag.start>0 && flag.barrier==0)
        {
            flag.lost = 1;
            printLog("Signal lost");
        }
#endif
    }

    dir_out = limit(dir_out, DIROUTMAX);
    dir_out = (int)(dir_out_last * 0.2f + dir_out * 0.8f);
    dir_out_last = dir_out;
    ftestVal[0] = dir_out;    ////////////////////////////
    ////////////////////////////
    return dir_out;
}
//---------------方向控制-以上--------------------


//---------------速度控制-------------------------
int16 target_speed[2] = {200, 200}, target_speed_max[2] = {200, 200};
uint16 spd_acc = 15;
float speed_k_limit = 1, car_speed_now = 0;
double path_length = 0;

int SpeedControl(void)
{
    static int16    left_spd_sum=0, right_spd_sum=0;
    static float    speed_out=0;
    static float    speed_last = 0;
    static float    Speedold[5]={0};
    static uint8    speed_cnt=0,spd_error = 0;
    static float    speed_out_old = 0, speed_out_pre = 0, speed_ave_out = 0;
    float SpeedRate = 1;
    speed_cnt++;
    left_spd_sum += Speed_Get('L');
    right_spd_sum += Speed_Get('R');
    if(speed_cnt % 5 == 0)//10ms计算
    {
        car_speed_now = (left_spd_sum + right_spd_sum) / 2.0f;
        left_spd_sum = 0;
        right_spd_sum = 0;
        car_speed_now = car_speed_now * 0.7f + speed_last * 0.3f;
        path_length += car_speed_now/1000;
        if(myfabs(car_speed_now) > 600) spd_error++;
        else spd_error = 0;
        if(spd_error > 50 && flag.mode != MODE_DEBUG)//速度连续不正常保护
        {
            flag.lost = 1;
            printLog("Speed out of range");
        }
        //限制加速度
        if( (car_speed_now-Speedold[4]) > 30 )
            car_speed_now=Speedold[4]+30;
        else if( (car_speed_now-Speedold[4]) < -30 )
            car_speed_now=Speedold[4]-30;
        Speedold[4]=Speedold[3];
        Speedold[3]=Speedold[2];
        Speedold[2]=Speedold[1];
        Speedold[1]=Speedold[0];
        Speedold[0]=car_speed_now;
        car_speed_now = flimit_ab(car_speed_now, -200, 450);//当前速度限幅
        speed_last = car_speed_now;
        if(target_speed_max[Balance_mode] > 0 && speed_cnt >= 50)
        {   //---------------转向与速度挂钩-----------------
            SpeedRate = car_speed_now/target_speed_max[Balance_mode];
            if(SpeedRate>1.3f)SpeedRate=1.3f;
            if(SpeedRate>0.9f)
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-(1.0-SpeedRate)*S_STEP1);
            else if(SpeedRate>0.8f)
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-0.1f*S_STEP1-(0.9f-SpeedRate)*S_STEP2);
            else if(SpeedRate>0.7f)
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-0.1f*S_STEP1-0.1f*S_STEP2-(0.8f-SpeedRate)*S_STEP3);
            else if(SpeedRate>0.6f)
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-0.1f*S_STEP1-0.1f*S_STEP2-0.1f*S_STEP3-(0.7f-SpeedRate)*S_STEP4);
            else if(SpeedRate>0.5f)
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-0.1f*S_STEP1-0.1f*S_STEP2-0.1f*S_STEP3-0.1f*S_STEP4-(0.6f-SpeedRate)*S_STEP5);
            else
                pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*(1.0f-0.1f*S_STEP1-0.1f*S_STEP2-0.1f*S_STEP3-0.1f*S_STEP4-0.1f*S_STEP5-(0.5f-SpeedRate)*S_STEP6);
            if(pid_dir[Balance_mode].p<pid_dir_pset[Balance_mode]*0.5f)   pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*0.5f;      //最小0.5f
            if(pid_dir[Balance_mode].p>pid_dir_pset[Balance_mode]*1.2f)   pid_dir[Balance_mode].p=pid_dir_pset[Balance_mode]*1.2f;      //最大
        }
    }
    if(Balance_mode == 0)//直立
    {
        if(speed_cnt >= 50)//100ms
        {
            //OLED_Print_int16(0, 0, car_speed_now, 0, 1);
            //速度输出pid
            pid_speed[Balance_mode].preError = pid_speed[Balance_mode].error;
            pid_speed[Balance_mode].error = target_speed[Balance_mode] - car_speed_now;
            pid_speed[Balance_mode].deriv = pid_speed[Balance_mode].error - pid_speed[Balance_mode].preError;
            pid_speed[Balance_mode].integ += pid_speed[Balance_mode].error * pid_speed[Balance_mode].i;
            //积分限幅
            pid_speed[Balance_mode].integ = flimit(pid_speed[Balance_mode].integ, (int16)(pid_speed[Balance_mode].intlimit * speed_k_limit));
            if(myfabs(pid_speed[Balance_mode].error) > pid_speed[Balance_mode].errlimit)
                pid_speed[Balance_mode].output=pid_speed[Balance_mode].p*pid_speed[Balance_mode].error-pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv;
            else
                pid_speed[Balance_mode].output=pid_speed[Balance_mode].p*pid_speed[Balance_mode].error-pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv+pid_speed[Balance_mode].integ;
            speed_ave_out = (pid_speed[Balance_mode].output-speed_out_old)/50.0f;
            speed_ave_out = flimit(speed_ave_out, 20);
            //---------------------------
            speed_out_old=speed_out;
        }
        speed_out += speed_ave_out;
        speed_out = flimit_ab(speed_out, -1000, (uint16)(3000*speed_k_limit));
    }
    else if(Balance_mode == 1)//三轮
    {
        if(pid_angle[Balance_mode].error > 10)
        {
            pid_speed[Balance_mode].p = 0.7f * pid_spd_set[0];
            pid_speed[Balance_mode].i = 0.7f * pid_spd_set[1];
        }
        else if(pid_angle[Balance_mode].error > 5)
        {
            pid_speed[Balance_mode].p = 0.9f * pid_spd_set[0];
            pid_speed[Balance_mode].i = 0.9f * pid_spd_set[1];
        }
        else
        {
            pid_speed[Balance_mode].p = pid_spd_set[0];
            pid_speed[Balance_mode].i = pid_spd_set[1];
        }
        //速度输出pid
        pid_speed[Balance_mode].preError = pid_speed[Balance_mode].error;
        pid_speed[Balance_mode].error = target_speed[Balance_mode] - 0.3f*pid_dir[Balance_mode].error - pid_dir[Balance_mode].deriv - car_speed_now;
        pid_speed[Balance_mode].deriv = pid_speed[Balance_mode].error - pid_speed[Balance_mode].preError;
        pid_speed[Balance_mode].integ += pid_speed[Balance_mode].error * pid_speed[Balance_mode].i * 0.1f;
        //积分限幅
        pid_speed[Balance_mode].integ = flimit(pid_speed[Balance_mode].integ, (int16)(pid_speed[Balance_mode].intlimit * speed_k_limit));
        if(myfabs(pid_speed[Balance_mode].error) > pid_speed[Balance_mode].errlimit)
            pid_speed[Balance_mode].output = target_speed[Balance_mode] + 0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error+pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv;
        else
            pid_speed[Balance_mode].output = target_speed[Balance_mode] + 0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error+pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv+pid_speed[Balance_mode].integ;
        //---------------------------
        speed_out = pid_speed[Balance_mode].output;
        if(speed_out - speed_out_pre > spd_acc)
            speed_out = speed_out_pre + spd_acc;
    }

    if(flag.En_spd == 0)       //直立模式或调试模式不输出速度
    {
        speed_out = 0;
    }
    if(speed_cnt >= 50)//100ms计数器清零
    {
        speed_cnt = 0;
    }
    speed_out = 0.4f*speed_out_pre + 0.6f*speed_out;//低通滤波
    speed_out_pre = speed_out;
    return (int)speed_out;
}
//-----------------速度控制-以上-----------------------------

//----------------电机输出--以下不必大改-----------------------
void motor_out(int angle_out, int speed_out, int dir_out)
{
    static uint16 error=0;
    static int L_Speed=0, R_Speed=0;                        //左右轮最终输出
    static int AngleSpeedSum=0;                            //角度控制和速度控制之和
    static int DirectionMore=0;                             //转向饱和后寄存变量
    if(Balance_mode == 0)//两轮
    {
        AngleSpeedSum = angle_out-speed_out;
        AngleSpeedSum = limit_ab(AngleSpeedSum, -700, 900);
    }
    else if(Balance_mode == 1)//三轮
    {
        AngleSpeedSum = angle_out+speed_out;
        //AngleSpeedSum = limit_ab(AngleSpeedSum, 0, 800);
    }

    if(myfabs(AngleSpeedSum/(myfabs(car_speed_now)+1)) > 50 && myabs(AngleSpeedSum )> 100 && time_count>1000) error++;
    else error = 0;
    if(error > 500 && time_count>1000 && flag.mode != MODE_DEBUG)
    {
        flag.lost = 1;
        printLog("locked-rotor");
    }
    if(dir_out>0)                //判断转向输出符号
    {
        R_Speed=AngleSpeedSum+dir_out; //加速轮
        if(R_Speed>PWMOUTMAX)
        {
            DirectionMore=R_Speed-PWMOUTMAX;       //将多余部分补偿到减速论
            L_Speed=AngleSpeedSum-dir_out-DirectionMore;
        }
        else
            L_Speed=AngleSpeedSum-dir_out;
    }
    else
    {
        L_Speed=AngleSpeedSum-dir_out;
        if(L_Speed>PWMOUTMAX)
        {
            DirectionMore=L_Speed-PWMOUTMAX;
            R_Speed=AngleSpeedSum+dir_out-DirectionMore;
        }
        else
            R_Speed=AngleSpeedSum+dir_out;
    }

    //以下为电机正反控制，以L_Speed>0和R_Speed>0为电机正转，不作修改！！！！
    if(L_Speed>0&&L_Speed<20.0) L_Speed=20.0;       //克服电机死区电压
    else if(L_Speed<0&&L_Speed>-20.0) L_Speed=-20.0;

    if(R_Speed>0&&R_Speed<20.0) R_Speed=20.0;
    else if(R_Speed<0&&R_Speed>-20.0) R_Speed=-20.0;

    if(time_count<500)
    {
        L_Speed *= time_count/500;
        R_Speed *= time_count/500;
    }

    if(flag.mode == MODE_DEBUG || (flag.lost == 1 && flag.mode != MODE_PWM_TEST))//
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);
        if(flag.lost == 1)
        {
            buzz_off();
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, 450);
            DisableInterrupts;
            int8 buff[20];
            sprintf(buff, ">> %d s, %3.1f m\0", time_count/500, path_length);
            printLog(buff);
        }
    }
    else if(flag.mode == MODE_PWM_TEST)
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,300);//L-
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,300);//R-
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);//L+
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);//R+
    }
    else
    {
        if(L_Speed>0)
        {
            if(L_Speed>PWMOUTMAX)
                L_Speed = PWMOUTMAX;                            //限幅
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,(int)L_Speed);            //左轮正传
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,0);
        }
        else
        {
            L_Speed = -L_Speed;                                   //求正数
            if(L_Speed>PWMOUTMAX)
                L_Speed = PWMOUTMAX;                            //限幅
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,(int)L_Speed);            //左轮反传
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);
        }
        if(R_Speed>0)
        {
            if(R_Speed>PWMOUTMAX)
                R_Speed = PWMOUTMAX;                            //右轮限幅
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,(int)R_Speed);            //右轮正转
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,0);
        }
        else
        {
            R_Speed = -R_Speed;                                 //取反
            if(R_Speed>PWMOUTMAX)
                R_Speed = PWMOUTMAX;                            //右轮限幅
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,(int)R_Speed);            //右轮反转
            ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);
        }
    }
}
