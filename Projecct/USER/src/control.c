#include "control.h"

#define _PROTECT_

//---------------平衡控制---------------
EulerAngleTypedef      CarAttitude;            /////姿态角
EulerAngleTypedef      CarAttitudeRate;        /////姿态角速度
float AccZAngle = 0;
float target_angle[2] = {-2, -21};
uint16 mag_threshold = 1550;
uint8 stop_time = 10;

uint8 Balance_mode = 0; //0-直立 1-三轮

void RampDetection(uint8 cnt)
{
    static uint8 ramp_cnt = 0;
    static uint16 ramp_timecnt= 0;
    static float pitch_rate_queue[25] = {0};
    float pitch_rate_max, pitch_rate_min, pitch_rate_avr;
    static int32 time = -500;
    if(Balance_mode)
    {
        if(flag.ramp == 0 && CarAttitude.Pitch > -21 && time_count > 750
           && !flag.broken_road && !flag.obstacle && flag.circle < 2
           && time_count - time > 1500)
        {
            ramp_cnt++;
            if(ramp_cnt > 15)
            {
                flag.ramp = 1;
                time = time_count;
                ramp_cnt = 0;
                flag.buzz = 2;//////////////////////
            }
        }
        if(flag.ramp == 1 && CarAttitude.Pitch < -34)
        {
            flag.ramp = 2;
        }
        if(flag.ramp == 2)
        {
            if(myfabs(CarAttitude.Pitch+31) < 4)
                ramp_cnt++;
            else
                ramp_cnt = 0;
            if(ramp_cnt > 70)
            {
                flag.ramp = 0;
                ramp_cnt = 0;
                ramp_timecnt = 0;
            }
        }
        if(flag.ramp > 0)
        {
            ramp_timecnt++;
            if(ramp_timecnt > 800)
            {
                flag.ramp = 0;
                flag.buzz = 1;///////////////////////
            }
        }
        else
            ramp_timecnt = 0;
    }
    else
    {
        if(cnt % 2 == 0)
        {
            pitch_rate_max = CarAttitudeRate.Pitch;
            pitch_rate_min = CarAttitudeRate.Pitch;
            pitch_rate_avr = CarAttitudeRate.Pitch;
            for(int i=24; i>0; i--)
            {
                pitch_rate_queue[i] = pitch_rate_queue[i-1];
                if(pitch_rate_max < pitch_rate_queue[i])
                    pitch_rate_max = pitch_rate_queue[i];
                if(pitch_rate_min > pitch_rate_queue[i])
                    pitch_rate_min = pitch_rate_queue[i];
                pitch_rate_avr += pitch_rate_queue[i];
            }
            pitch_rate_queue[0] = CarAttitudeRate.Pitch;
            pitch_rate_avr /= 25;
        }
        if((pitch_rate_max > 100 || pitch_rate_min < -100) && myfabs(pitch_rate_avr)<50
           && ad_data_now[MD] > 1488-40.88f*CarAttitude.Pitch && flag.ramp == 0
           && !flag.broken_road && !flag.obstacle && flag.circle < 2 && time_count > 500
           && time_count - time > 1500)
        {
            flag.ramp = 1;
            flag.buzz = 2;///////////////////////////////////
            ramp_timecnt = 0;
            time = time_count;
        }
        else if(flag.ramp == 1)
        {
            ramp_timecnt++;
            if(ramp_timecnt > 150)
            {
                ramp_timecnt = 0;
                flag.ramp = 2;
            }
        }
        else if(flag.ramp == 2)
        {
            ramp_timecnt++;
            if(ramp_timecnt > 350)
            {
                ramp_timecnt = 0;
                flag.ramp = 0;
                flag.buzz = 1;///////////////////////////////////////
            }
        }
    }
}

int BalanceControl(void)
{
    static float camera_angle, camera_angle_last;
    uint8 offset = 0;
    static uint8 cnt = 0,error=0;
    int angle_out = 0;
    float t;
    cnt++; if(cnt >= 10) cnt = 0;

    if(cnt)
        BMX055_DataRead(&Q_raw, 0);
    else
    {
        BMX055_DataRead(&Q_raw, 1);
        if(myabs(Q_raw.Mag-mag_threshold) > 500 && time_count > stop_time*500
           && flag.stop == 0 && flag.mode == MODE_START)
        {
            flag.stop = 1;
            printLog("Mag stop");
            int8 buff[20];
            sprintf(buff, ">> %.1f s \0", time_count/500.0);
            printLog(buff);
        }
    }
    //正常1500-1800
    //有磁铁Mag<1400||Mag>1900 变化率很大

    Gyro.Xdata = (Q_raw.GyroX-0.2f) * 0.030517578f;// - GyroOffset.Xdata   //  1000/32768
    Gyro.Ydata = (Q_raw.GyroY+1.5f) * 0.030517578f;// - GyroOffset.Ydata
    Gyro.Zdata = (Q_raw.GyroZ-2.8f) * 0.030517578f;// - GyroOffset.Zdata
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
    camera_angle = CarAttitude.Pitch;
    camera_angle = flimit_ab(camera_angle,-31,2);
    camera_angle = 0.3f*camera_angle + 0.7f*camera_angle_last;
    camera_angle_last = camera_angle;

    if(camera_angle <= -31) offset = 0;
    else if(camera_angle >= 2) offset = 127;
    else offset = (uint8)((camera_angle+31)*3.848f);
    if(Balance_mode == 1)
        offset = offset | 0x80;

    if(cnt%5==0) uart_putchar(COM_UART, offset);

    if((CarAttitude.Pitch > 30 || CarAttitude.Pitch < -80 || myfabs(CarAttitude.Roll) > 30)
       && flag.mode != MODE_DEBUG && time_count>1000)
        error++;
    else
        error = 0;
    if(error>50 && flag.lost == 0 && flag.mode == MODE_START)
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
        t = pid_angle[Balance_mode].d * (-CarAttitudeRate.Pitch);
        if(t > -20) t = 0;
        if(pid_angle[Balance_mode].error > 10)
            pid_angle[Balance_mode].error = (pid_angle[Balance_mode].error-10)*2+10;
        pid_angle[Balance_mode].output = -pid_angle[Balance_mode].p * pid_angle[Balance_mode].error + t;
    }
    RampDetection(cnt);
    angle_out = (int)pid_angle[Balance_mode].output;
    if(flag.En_std == 0)
        angle_out = 0;


//    ftestVal[4] = Q_raw.GyroX-0.2f;///////////////////
//    ftestVal[5] = Q_raw.GyroY+1.5f;///////////////////
//    ftestVal[6] = Q_raw.GyroZ-2.8f;////////////////////

    return angle_out;
}

//---------------平衡控制-以上-------------------



//----------------方向控制---------------------
uint16 ad_data_now[NUM_OF_AD] = {0};

float k_hv[2] = {6.5, 4};//横竖电感差比例
float k_x[2] = {0, 0.5};//横竖电感差比例
float k_hv_cin[2] = {3, 2};//进环横竖电感差比例
float k_hv_cout[2] = {9, 6};//出环横竖电感差比例
float k_md[2] = {0.3, 0.4};//中间电感比例
float k_adc = 1.0f;
float k_ei = 1;

float k_circle[5] = {1,1,1,1,1};//入环系数
int16 circle_offset[5] = {25,-20,20,20,20};//入环偏差
float k_cout[5] = {1.0, 1.0, 1.0, 1.0, 1.0};//出环偏差系数
float k_cout_offset[5] = {1.0, 1.0, 1.0, 1.0, 1};//出环系数
uint8 circle_size[5] = {1,1,5,3,3};//出环偏差
uint8 cl_num = 1;
uint16 cl_out_delay = 400, cl_time = 250;//环参数
int16 circle_time_count[2] = {0};
int8 run_dir = 1;
int8 crcl_cnt = -1;
int8 crcl_cnt2 = 0;
int8 circle_dir = 0;
float yaw_integ = 0;
uint8 cross_time = 180;

float dist_kp = 1;

void ADC_get(void)
{
    uint8 i,j;
    uint16 ad_raw_data_now[NUM_OF_AD]={0};
    static uint16 ad_raw_data_pre[NUM_OF_AD][5]={0};
#define SAMPLING_NUM 3
    for(i = 0; i < SAMPLING_NUM; i++)
    {
        ad_raw_data_now[LH] += adc_once(ADC_LH,ADC_12bit);
        ad_raw_data_now[RH] += adc_once(ADC_RH,ADC_12bit);
        ad_raw_data_now[LV] += adc_once(ADC_LV,ADC_12bit);
        ad_raw_data_now[RV] += adc_once(ADC_RV,ADC_12bit);
        ad_raw_data_now[LX] += adc_once(ADC_LX,ADC_12bit);
        ad_raw_data_now[RX] += adc_once(ADC_RX,ADC_12bit);
        ad_raw_data_now[MD] += adc_once(ADC_MD,ADC_12bit);
    }
    ad_raw_data_now[LH] /= SAMPLING_NUM;    //3300  700
    ad_raw_data_now[RH] /= SAMPLING_NUM;    //3300  700
    ad_raw_data_now[LV] /= SAMPLING_NUM;    //3000  2400
    ad_raw_data_now[RV] /= SAMPLING_NUM;    //3000  2400
    ad_raw_data_now[LX] /= SAMPLING_NUM;    //3000  2400
    ad_raw_data_now[RX] /= SAMPLING_NUM;    //3000  2400
    ad_raw_data_now[MD] /= SAMPLING_NUM;    //3000

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
        ad_data_now[i] = (uint16)(k_adc*(0.45f*ad_raw_data_pre[i][0] + 0.25f*ad_raw_data_pre[i][1] +
            0.15f*ad_raw_data_pre[i][2] + 0.1f*ad_raw_data_pre[i][3] + 0.05f*ad_raw_data_pre[i][4]));
    }
}

float ErrorCalculate(uint8 mode)
{
    float error1, error2;
    float error_out, kcl = 0, tAngle = -10;
    static float error_out_last;
    static int16 difX_div[5] = {0};
    int16 difH, difV, difX;
    uint16 sumH, sumHM2, sumHM3, sumV;//, sumX;//, sumHM3, sumHM4;
    static float sumHM[5];
    static uint8 cross_last = 0, cross_cnt = 0;
    ADC_get();
    difH = ad_data_now[LH]-ad_data_now[RH];
    difV = ad_data_now[LV]-ad_data_now[RV];
    difX = ad_data_now[LX]-ad_data_now[RX];
    sumH = ad_data_now[LH]+ad_data_now[RH]+1;
    sumV = ad_data_now[LV]+ad_data_now[RV]+1;
    //sumX = ad_data_now[LX]+ad_data_now[RX]+1;
    for(uint8 i=4; i>0; i--)
        sumHM[i] = sumHM[i-1];
    sumHM[0] = sumH + ad_data_now[MD];
    sumHM3 = (uint16)(0.3f*sumHM[0] + 0.25f*sumHM[1] + 0.2f*sumHM[2] + 0.15f*sumHM[3] + 0.1f*sumHM[4]);
    sumHM2 = (uint16)(sumH + k_md[mode]*ad_data_now[MD]);
    tAngle = flimit_ab(CarAttitude.Pitch, -20, 0);
    for(uint8 i=4; i>0; i--)
        difX_div[i] = difX_div[i-1];
    difX_div[0] = (int16)(myabs(difX)*0.5f + difX_div[0]*0.5f);
    if(flag.cross == 1 && cross_cnt > 10 && cross_cnt < 220)
    {
        error1 = 10 * (8*difH + 2*difV + k_x[mode]*difX)/sumHM2;
        flag.buzz = 6;
    }
    else
        error1 = 10 * (k_hv[mode]*difH + (10.0f-k_hv[mode])*difV + k_x[mode]*difX)/sumHM2;
    error_out = error1;

    //ftestVal[1] = difH;             /////////////////////////
    //ftestVal[2] = difV;             ///////////////////////
    //ftestVal[3] = difX;            /////////////////////////
    //ftestVal[4] = sumHM2;                    ///////////////////////
    //ftestVal[4] = sumV;          ///////////////////////
    //ftestVal[5] = ad_data_now[MD];          ///////////////////////
    //ftestVal[4] = 500*flag.circle;          ///////////////////////
    //ftestVal[5] = sumV;                   /////////////////////////

    //--------------检环-------------------
    if(Balance_mode == 0)
    {
        if(time_count>500 && sumHM3 > -70*tAngle+2920 && ad_data_now[MD] > -36*tAngle+1396//3500  1200
            && sumV < 1800 && flag.circle <= 1 && !flag.obstacle && !flag.broken_road)
            //左中右和很大  中间很大(保证车在中间)
        {
            flag.circle = 1;
            if(((difX_div[0] - difX_div[2])*(difX_div[2] - difX_div[4]) < 0 && myabs(difX_div[2]) > -18.3*tAngle+343)
                || (myabs(difX_div[2]) > -42.4*tAngle+586))//500 700   886 768
            {
                //外八电感之差变号  外八电感之差较大
                flag.circle = 2;
                yaw_integ = 0;
                crcl_cnt++;
                if(crcl_cnt >= cl_num) crcl_cnt = 0;
                if(run_dir < 0)
                    crcl_cnt2 = cl_num - 1 - crcl_cnt;
                else
                    crcl_cnt2 = crcl_cnt;
                if(difX > 50)
                    circle_dir = 1;
                else if(difX < -50)
                    circle_dir = -1;
                else
                    circle_dir = 0;
                flag.buzz = 1;/////////////////////////////////////
                circle_time_count[1] = 0;
            }
        }
        if(flag.circle == 2)
        {
            yaw_integ += CarAttitudeRate.Yaw*0.002f;
            circle_time_count[1]++;
            error2 = k_circle[crcl_cnt2]*6*(k_hv_cin[mode]*difH+(10.0f-k_hv_cin[mode])*difV)/sumHM2
                + circle_dir*circle_offset[crcl_cnt2];
            //k=1,offset=20
            if(circle_size[crcl_cnt2] == 1)
                kcl = trapezoid_fun(circle_time_count[1], 70, 120, 100, 1);
            else if(circle_size[crcl_cnt2] == 2)
                kcl = trapezoid_fun(circle_time_count[1], 90, 120, 140, 1);
            else if(circle_size[crcl_cnt2] == 3)
                kcl = trapezoid_fun(circle_time_count[1], 100, 130, 140, 1);
            else if(circle_size[crcl_cnt2] == 4)
                kcl = trapezoid_fun(circle_time_count[1], 140, 150, 150, 1);
            else if(circle_size[crcl_cnt2] == 5)
                kcl = trapezoid_fun(circle_time_count[1], 170, 160, 200, 1);
            //大环kcircle = 1; circleoffset = 25;
            error_out = kcl*error2 + (1-kcl)*error1;
            if(myfabs(yaw_integ) > 320+5*circle_size[crcl_cnt2])//295+5*circle_size[crcl_cnt2]
            {
                //陀螺仪积分  左中右较大  外八之差较小
                flag.circle = 3;
                flag.buzz = 3;///////////////////////////////////
                circle_time_count[1] = 0;
                yaw_integ = 0;
            }
        }
        else if(flag.circle == 3)
        {
            circle_time_count[1]++;
            if(circle_size[crcl_cnt2] == 1)
            {
                error2 = k_cout[crcl_cnt2]*70*difH/sumHM2
                    - 0.3*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 70, 80, 80, 1);
            }
            else if(circle_size[crcl_cnt2] == 2)
            {
                error2 = k_cout[crcl_cnt2]*63*difH/sumHM2
                    - 0.35f*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 80, 90, 90, 1);
            }
            else if(circle_size[crcl_cnt2] == 3)
            {
                error2 = k_cout[crcl_cnt2]*45*difH/sumHM2
                    - 0.5f*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 100, 120, 110, 1);
            }
            else if(circle_size[crcl_cnt2] == 4)
            {
                error2 = k_cout[crcl_cnt2]*50*difH/sumHM2
                    - 0.7*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 100, 125, 125, 1);
            }
            else if(circle_size[crcl_cnt2] == 5)
            {
                error2 = k_cout[crcl_cnt2]*50*difH/sumHM2
                    - 0.8*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 100, 130, 140, 1);
            }
            error_out = kcl*error2 + (1-kcl)*error1;
            if(circle_time_count[1] > 400 && kcl < 0.1)
                flag.circle = 0;
        }
    }
    else if(Balance_mode == 1)
    {
        if(time_count>500 && sumHM3 >5500 && ad_data_now[MD] > 2800 && sumV < 2800
           && flag.circle <= 1 && !flag.obstacle && !flag.broken_road)
            //左中右和很大，中间很大
        {
            flag.circle = 1;
            if((difX_div[0]-difX_div[2]<0 && difX_div[2]-difX_div[4]>0 && difX_div[2] > 1500)
                || difX_div[2] > 2200)
            {
                flag.circle = 2;
                yaw_integ = 0;
                crcl_cnt++;
                if(crcl_cnt >= cl_num) crcl_cnt = 0;
                if(run_dir < 0)
                    crcl_cnt2 = cl_num - 1 - crcl_cnt;
                else
                    crcl_cnt2 = crcl_cnt;
                if(difX > 50)
                    circle_dir = 1;
                else if(difX < -50)
                    circle_dir = -1;
                else
                    circle_dir = 0;
                flag.buzz = 1;///////////////////////////
                circle_time_count[1] = 0;
            }
        }
        if(flag.circle == 2)
        {
            yaw_integ += CarAttitudeRate.Yaw*0.002f;
            circle_time_count[1]++;
            error2 = k_circle[crcl_cnt2]*14*(k_hv_cin[mode]*difH+(10.0f-k_hv_cin[mode])*difV)/sumHM2
                + circle_dir*(circle_offset[crcl_cnt2]>0?circle_offset[crcl_cnt2]+20:circle_offset[crcl_cnt2]-20);
            if(circle_size[crcl_cnt2] == 1)
                kcl = trapezoid_fun(circle_time_count[1], 50, 100, 100, 1);
            else if(circle_size[crcl_cnt2] == 2)
                kcl = trapezoid_fun(circle_time_count[1], 62, 112, 112, 1);
            else if(circle_size[crcl_cnt2] == 3)
                kcl = trapezoid_fun(circle_time_count[1], 75, 125, 125, 1);
            else if(circle_size[crcl_cnt2] == 4)
                kcl = trapezoid_fun(circle_time_count[1], 87, 137, 137, 1);
            else if(circle_size[crcl_cnt2] == 5)
                kcl = trapezoid_fun(circle_time_count[1], 100, 150, 150, 1);
            error_out = kcl*error2 + (1-kcl)*error1;
            if((myfabs(yaw_integ) > 290+5*circle_size[crcl_cnt2]))
            {
                flag.circle = 3;
                circle_time_count[1] = 0;
                flag.buzz = 3;///////////////////////////
            }
        }
        else if(flag.circle == 3)
        {
            circle_time_count[1]++;
            error2 = k_cout[crcl_cnt2]*55*difH/sumHM2
                - circle_dir*k_cout_offset[crcl_cnt2]*circle_offset[crcl_cnt2];
            if(circle_size[crcl_cnt2] == 1)
                kcl = trapezoid_fun(circle_time_count[1], 70, 140, 90, 1);
            else if(circle_size[crcl_cnt2] == 2)
                kcl = trapezoid_fun(circle_time_count[1], 70, 152, 98, 1);
            else if(circle_size[crcl_cnt2] == 3)
                kcl = trapezoid_fun(circle_time_count[1], 80, 165, 107, 1);
            else if(circle_size[crcl_cnt2] == 4)
                kcl = trapezoid_fun(circle_time_count[1], 90, 178, 108, 1);
            else if(circle_size[crcl_cnt2] == 5)
                kcl = trapezoid_fun(circle_time_count[1], 100, 150, 130, 1);
            error_out = kcl*error2 + (1-kcl)*error1;
            if(circle_time_count[1] > 400 && kcl < 0.1)
                flag.circle = 0;
        }
    }
    if(flag.circle > 0)
        error_out = 0.7*error_out + 0.3*error_out_last;
    if(flag.circle == 1)
    {
        circle_time_count[0]++;
        if(circle_time_count[0] > circle_size[crcl_cnt2]*50+250)
        {
            circle_time_count[0] = 0;
            flag.circle = 0;
            //flag.buzz = 2;////////////////////////////
        }
    }
    else if(flag.circle > 1)
    {
        circle_time_count[0]++;
        if(circle_time_count[0] > circle_size[crcl_cnt2]*250+1250)
        {
            circle_time_count[0] = 0;
            flag.circle = 0;
            flag.buzz = 2;////////////////////////////
        }
    }
    else
        circle_time_count[0] = 0;
    //--------------检环结束------------------

    //-------------检测十字-------------------
    if(flag.cross_pre - cross_last == 1)
    {
        flag.cross = 1;
        cross_cnt = 0;
    }
    if(flag.cross == 1)
        cross_cnt++;
    else
        cross_cnt = 0;
    if(cross_cnt > cross_time)//170~200
        flag.cross = 0;
    cross_last = flag.cross_pre;
    //----------------检测十字结束----------------

    //ftestVal[0] = error1*10;             /////////////////////////
    //ftestVal[1] = error2*10;             /////////////////////////
    //ftestVal[6] = kcl*1500;              ///////////////////////////
    //ftestVal[0] = error_out*100;           ///////////////////////////
    error_out_last = error_out;
    return error_out;
}

void Dir_pid_control(float E_error)
{
    pid_dir[Balance_mode].error = E_error;
    pid_dir[Balance_mode].deriv = pid_dir[Balance_mode].error - pid_dir[Balance_mode].preError[4];
    pid_dir[Balance_mode].preError[4] = pid_dir[Balance_mode].preError[3];
    pid_dir[Balance_mode].preError[3] = pid_dir[Balance_mode].preError[2];
    pid_dir[Balance_mode].preError[2] = pid_dir[Balance_mode].preError[1];
    pid_dir[Balance_mode].preError[1] = pid_dir[Balance_mode].preError[0];
    pid_dir[Balance_mode].preError[0] = pid_dir[Balance_mode].error;
    pid_dir[Balance_mode].output = pid_dir[Balance_mode].p * pid_dir[Balance_mode].error
        + pid_dir[Balance_mode].d * pid_dir[Balance_mode].deriv;
}

int Yaw_pid_control(float Error)
{
    pid_yaw[Balance_mode].error = Error + CarAttitudeRate.Yaw;
    pid_yaw[Balance_mode].deriv = pid_yaw[Balance_mode].error - pid_yaw[Balance_mode].preError[2];
    pid_yaw[Balance_mode].preError[2] = pid_yaw[Balance_mode].preError[1];
    pid_yaw[Balance_mode].preError[1] = pid_yaw[Balance_mode].preError[0];
    pid_yaw[Balance_mode].preError[0] = pid_yaw[Balance_mode].error;
    pid_yaw[Balance_mode].output = pid_yaw[Balance_mode].p * pid_yaw[Balance_mode].error + pid_yaw[Balance_mode].d * pid_yaw[Balance_mode].deriv;
    return (int)pid_yaw[Balance_mode].output;
}


uint16 obstacle_time = 0;
uint16 obstacle_turn_t[2] = {70,80};
uint16 obstacle_turn_k[2] = {400,500};
uint16 obstacle_delay1[2]  = {70,60};
uint16 obstacle_delay2[2] = {10,100};
uint16 obstacle_delay3[2] = {50,10};
int8 obstacle_turn_dir[3] = {1,1,1};
uint16 bt[2][8];
uint8 obstacle_cnt = 0;
uint8 obstacle_cnt2 = 0;
int8 obstacle_step = 0;
uint8 obstacle_yaw[2] = {50,50};
uint8 obstacle_len1[2] = {125,125};
uint16 obstacle_len2[2] = {150,150};
uint8 obstacle_len3[2] = {115,115};
uint16 disTh = 450;

int ObstacleClear(float E_error)
{
    static uint8 tmpcnt = 0;
    //float speed_rate = 1;
    float error_offset = 0;
    int dir_out = 0;
    //float speed_k = 1;
    static float yaw = 0;
    static float len = 0;
    if(flag.obstacle == 2)
    {
        if(obstacle_step == 1)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_turn_t[Balance_mode]*6
               || yaw < -obstacle_turn_t[Balance_mode]*6)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 2;
                    obstacle_time = 0;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 2)
        {
            len += car_speed_now;
            dir_out = Yaw_pid_control(0);
            if(len > obstacle_delay1[Balance_mode]*200)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 3;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 3)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = -obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_turn_t[Balance_mode]*6
               || yaw < -obstacle_turn_t[Balance_mode]*6)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 4;
                    obstacle_time = 0;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 4)
        {
            len += car_speed_now;
            dir_out = Yaw_pid_control(0);
            if(len > obstacle_delay2[Balance_mode]*100)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 5;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 5)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = -1.05*obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_turn_t[Balance_mode]*7
               || yaw < -obstacle_turn_t[Balance_mode]*8)
            {
                tmpcnt++;
                if(tmpcnt > 5)
                {
                    obstacle_step = 6;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 6)
        {
            len += car_speed_now;
            if(ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]>500)
            {
                Dir_pid_control(E_error);
                dir_out = Yaw_pid_control(0.2f*pid_dir[Balance_mode].output);
            }
            else
                dir_out = Yaw_pid_control(0);
            if(len > obstacle_delay3[Balance_mode]*200 && ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]>500)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 7;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                    obstacle_time = 0;
                }
            }
        }
        else if(obstacle_step == 7)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            obstacle_time++;
            error_offset = obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            Dir_pid_control(E_error);
            dir_out = Yaw_pid_control(error_offset+0.4f*pid_dir[Balance_mode].output);
            if(yaw > obstacle_turn_t[Balance_mode]*6
               || yaw < -obstacle_turn_t[Balance_mode]*6
               || obstacle_time > 150)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 0;
                    obstacle_time = 0;
                    len = 0;
                    yaw = 0;
                    tmpcnt = 0;
                    flag.obstacle = 0;
                    obstacle_cnt++;
                    if(obstacle_cnt > 2)
                        obstacle_cnt = 0;
                    if(Balance_mode == 0)
                        obstacle_cnt2++;
                }
            }
        }
    }
    else if(flag.obstacle == 3 && Balance_mode == 1)
    {
        if(obstacle_step == 1)
        {
            if(car_speed_now >= 0)
            {
                Dir_pid_control(E_error);
                dir_out = Yaw_pid_control(pid_dir[Balance_mode].output);
            }
            else
            {
                dir_out = Yaw_pid_control(0);
            }
            if(car_speed_now < 100)
            {
                obstacle_step = 2;
                tmpcnt = 0;
                obstacle_time = 0;
            }
        }
        if(obstacle_step == -1)
        {
            if(car_speed_now >= 0)
            {
                Dir_pid_control(E_error);
                dir_out = Yaw_pid_control(pid_dir[Balance_mode].output);
            }
            else
            {
                dir_out = Yaw_pid_control(0);
            }
            if(car_speed_now < 120 && distance < 300)
            {
                obstacle_step = 2;
                tmpcnt = 0;
                obstacle_time = 0;
            }
        }
        else if(obstacle_step == 2)
        {
            if(car_speed_now >= 0)
            {
                Dir_pid_control(E_error);
                dir_out = Yaw_pid_control(pid_dir[Balance_mode].output);
            }
            else
            {
                dir_out = Yaw_pid_control(0);
            }
            if(distance > 800)
            {
                obstacle_time++;
                if(obstacle_time > 1500)
                {
                    obstacle_time = 0;
                    tmpcnt = 0;
                    flag.obstacle = 0;
                    obstacle_step = 0;
                }
            }
            if(myabs(distance - disTh) < 70 && myfabs(car_speed_now) < 70)
            {
                tmpcnt++;
                if(tmpcnt > 11)
                {
                    flag.obstacle = 4;
                    obstacle_step = 1;
                    obstacle_time = 0;
                    tmpcnt = 0;
                    yaw = 0;
                }
            }
            else
                tmpcnt = 0;
        }
    }
    else if(flag.obstacle == 4)
    {
        if(obstacle_step == 1)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_yaw[Balance_mode]*6
               || yaw < -obstacle_yaw[Balance_mode]*6)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 2;
                    obstacle_time = 0;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 2)
        {
            len += car_speed_now;
            dir_out = Yaw_pid_control(0);
            if(len > obstacle_len1[Balance_mode]*200)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 3;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 3)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = -obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_yaw[Balance_mode]*6
               || yaw < -obstacle_yaw[Balance_mode]*6)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 4;
                    obstacle_time = 0;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 4)
        {
            len += car_speed_now;
            dir_out = Yaw_pid_control(0);
            if(len > obstacle_len2[Balance_mode]*100)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 5;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 5)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            error_offset = -1.05*obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            dir_out = Yaw_pid_control(error_offset);
            if(yaw > obstacle_yaw[Balance_mode]*7
               || yaw < -obstacle_yaw[Balance_mode]*8)
            {
                tmpcnt++;
                if(tmpcnt > 5)
                {
                    obstacle_step = 6;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                }
            }
        }
        else if(obstacle_step == 6)
        {
            len += car_speed_now;
            if(ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]>500)
            {
                Dir_pid_control(E_error);
                dir_out = Yaw_pid_control(0.2f*pid_dir[Balance_mode].output);
            }
            else
                dir_out = Yaw_pid_control(0);
            if(len > obstacle_len3[Balance_mode]*200 && ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]>500)
            {
                tmpcnt++;
                if(tmpcnt > 6)
                {
                    obstacle_step = 7;
                    tmpcnt = 0;
                    yaw = 0;
                    len = 0;
                    obstacle_time = 0;
                }
            }
        }
        else if(obstacle_step == 7)
        {
            yaw += CarAttitudeRate.Yaw*0.02;
            obstacle_time++;
            error_offset = obstacle_turn_dir[obstacle_cnt] * run_dir
                * obstacle_turn_k[Balance_mode];
            Dir_pid_control(E_error);
            dir_out = Yaw_pid_control(error_offset+0.4f*pid_dir[Balance_mode].output);
            if(yaw > obstacle_yaw[Balance_mode]*6
               || yaw < -obstacle_yaw[Balance_mode]*6
               || obstacle_time > 150)
            {
                tmpcnt++;
                if(tmpcnt > 3)
                {
                    obstacle_step = 0;
                    obstacle_time = 0;
                    len = 0;
                    yaw = 0;
                    tmpcnt = 0;
                    flag.obstacle = 0;
                    obstacle_cnt++;
                    if(obstacle_cnt > 2)
                        obstacle_cnt = 0;
                    if(Balance_mode == 0)
                        obstacle_cnt2++;
                }
            }
        }
    }
    ftestVal[0] = dir_out;  /////////////////////////////
    ftestVal[1] = obstacle_step*20;  /////////////////////////////
    ftestVal[2] = yaw;  /////////////////////////////
    ftestVal[3] = len;  /////////////////////////////
    return dir_out;
}

int DirectionControl(void)
{
    int dir_out = 0;
    static int dir_out_last = 0;
    static uint8 err = 0;
    float E_error = 0;
    E_error = ErrorCalculate(Balance_mode);
    if(flag.obstacle)
        dir_out = ObstacleClear(E_error);
    else
    {
        Dir_pid_control(E_error);
        if(Balance_mode == 1 && line_cy!=0 && line_width!=0
           && line_width < 75 && car_speed_now > 270
           && !flag.ramp && !flag.obstacle && !flag.broken_road && !flag.circle)
            dir_out = Yaw_pid_control(k_ei*pid_dir[Balance_mode].output
                +(1-k_ei)*pid_img[Balance_mode].output);
        else
            dir_out = Yaw_pid_control(pid_dir[Balance_mode].output);
    }
    //输出限幅
    if(flag.En_dir == 0)
    {
        dir_out = 0;
    }
    else
    {//出界保护
#ifdef _PROTECT_
        if(ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]<100 && flag.En_dir == 1
           && flag.mode == MODE_START && flag.obstacle==0)
            err++;
        else
            err = 0;
        if(err > 100 && flag.lost == 0)
        {
            flag.lost = 1;
            printLog("Signal lost");
        }
#endif
    }
    dir_out = limit(dir_out, (Balance_mode?600:400));

    dir_out = (int)(dir_out_last * 0.2f + dir_out * 0.8f);
    //ftestVal[0] = dir_out/2;
    dir_out_last = dir_out;

    //ftestVal[5] = obstacle_step*100;    ////////////////////////////
    //ftestVal[1] = img_err;    ////////////////////////////
    //ftestVal[2] = pid_yaw[Balance_mode].error;

    ////////////////////////////
    return dir_out;
}
//---------------方向控制-以上--------------------


//---------------速度控制-------------------------
int16 target_speed[2] = {305, 310}, target_speed_max[2] = {305, 310};
uint16 spd_acc = 9;
float speed_k_limit = 1, car_speed_now = 0;
double path_length = 0;
uint8 swich_mode = 1;
uint16 speed_ramp = 230, speed_broken_road = 260;
int8 straight_speed_add = 0;
float speed_output = 0;



int SpeedControl(void)
{
    static int16    left_spd_sum=0, right_spd_sum=0;
    static float    speed_out=0;
    static float    speed_last = 0;
    static float    Speedold[5]={0};
    static uint8    speed_cnt=0,spd_error = 0;
    static uint16   straight_cnt = 0;
    static float    speed_out_old = 0, speed_out_pre = 0, speed_ave_out = 0;
    float SpeedRate = 1;
    uint8 spd_en = 1;
    speed_cnt++;
    left_spd_sum += Speed_Get(0);
    right_spd_sum += Speed_Get(1);
    if(speed_cnt % 5 == 0)//10ms计算
    {
        car_speed_now = (left_spd_sum + right_spd_sum) / 2.0f;
        left_spd_sum = 0;
        right_spd_sum = 0;
        car_speed_now = car_speed_now * 0.7f + speed_last * 0.3f;
        path_length += car_speed_now/1000;
        if(myfabs(car_speed_now) > 600) spd_error++;
        else spd_error = 0;
        if(spd_error > 50 && flag.mode == MODE_START && flag.lost == 0)//速度连续不正常保护
        {
            flag.lost = 1;
            printLog("Speed out of range");
        }
        if(!Balance_mode)
        {
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
        }
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

    target_speed[Balance_mode] = target_speed_max[Balance_mode];

    //停车标志
    if(flag.stop == 1)
        target_speed[Balance_mode] = 0;
    //大环加速
    else if(flag.circle == 2 && circle_size[crcl_cnt2] > 3
       && myfabs(yaw_integ) > 30 && myfabs(yaw_integ) < 200+circle_size[crcl_cnt2]*10)
        target_speed[Balance_mode] = (int16)(1.2f * target_speed_max[Balance_mode]);
    //其他环减速
    else if(flag.circle > 0)
        target_speed[Balance_mode] = 270;
    //坡道减速
    else if(flag.ramp > 0 && Balance_mode == 1)
    {
        if(flag.ramp == 1)
            target_speed[Balance_mode] = speed_ramp;
        else if(flag.ramp == 2)
            target_speed[Balance_mode] = speed_ramp-30;
    }
    //断路减速
    else if(flag.broken_road > 0)
    {
        target_speed[Balance_mode] = speed_broken_road;
    }
    //减速标志
    else if(flag.slow_down == 1 || time_count > tim.slow_a*500 && time_count < tim.slow_b*500
            || time_count > tim.slow_c*500 && time_count < tim.slow_d*500
            || time_count > tim.slow_e*500 && time_count < tim.slow_f*500)
        target_speed[Balance_mode] = (int16)(0.8f * target_speed_max[Balance_mode]);
    //直道加速
    else if(Balance_mode == 1 && line_cy!=0 && line_width!=0
        && line_width < 75 && myfabs(pid_img[Balance_mode].error) < 25
        && !flag.obstacle && !flag.broken_road)
    {
        if(myfabs(pid_dir[Balance_mode].error) < 25)
        {
            if(straight_cnt < 310)
                straight_cnt++;
            if(straight_cnt < 200)
                target_speed[Balance_mode] = (int16)(1.08f * target_speed_max[Balance_mode]+straight_speed_add);
            else if(straight_cnt < 300)
                target_speed[Balance_mode] = (int16)(1.03f * target_speed_max[Balance_mode]+straight_speed_add);
        }
        else
        {
            straight_cnt = 0;
        }
    }
    //变形特殊处理
    //flag.mode_switch检测到变形标志，swich_mode变形方式
    if(flag.mode_switch == 1 && swich_mode == 0)
        target_speed[1] = 0;
    else if(flag.mode_switch == 1 && swich_mode == 1)
        spd_en = 0;
    else
        spd_en = 1;
    //路障刹车模式特殊处理
    if(flag.obstacle == 3 && Balance_mode)
    {
        if(obstacle_step == 1)
            target_speed[1] = -100;
        else if(obstacle_step == -1)
            target_speed[1] = 60;
        else if(obstacle_step == 2)
            target_speed[1] = (int16)(dist_kp*limit_ab(distance - disTh,-300,300));
    }

    if(Balance_mode == 0)//直立
    {
        if(speed_cnt >= 50)//100ms
        {
            //速度输出pid
            pid_speed[Balance_mode].preError[4] = pid_speed[Balance_mode].preError[3];
            pid_speed[Balance_mode].preError[3] = pid_speed[Balance_mode].preError[2];
            pid_speed[Balance_mode].preError[2] = pid_speed[Balance_mode].preError[1];
            pid_speed[Balance_mode].preError[1] = pid_speed[Balance_mode].preError[0];
            pid_speed[Balance_mode].preError[0] = pid_speed[Balance_mode].error;
            pid_speed[Balance_mode].error = target_speed[Balance_mode] - car_speed_now;
            pid_speed[Balance_mode].deriv = pid_speed[Balance_mode].error - pid_speed[Balance_mode].preError[4];
            if(myfabs(pid_speed[Balance_mode].error) > pid_speed[Balance_mode].errlimit || time_count < 500)
                pid_speed[Balance_mode].output=pid_speed[Balance_mode].p*pid_speed[Balance_mode].error-pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv;
            else
            {
                pid_speed[Balance_mode].integ += pid_speed[Balance_mode].error * pid_speed[Balance_mode].i*0.1f;
                pid_speed[Balance_mode].integ = flimit(pid_speed[Balance_mode].integ, (int16)(pid_speed[Balance_mode].intlimit * speed_k_limit));
                pid_speed[Balance_mode].output=pid_speed[Balance_mode].p*pid_speed[Balance_mode].error-pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv+pid_speed[Balance_mode].integ;
            }
                speed_ave_out = (pid_speed[Balance_mode].output-speed_out_old)/50.0f;
            speed_ave_out = flimit(speed_ave_out, 20);
            //---------------------------
            speed_out_old=speed_out;
        }
        speed_out += speed_ave_out;
        speed_out = flimit_ab(speed_out, (flag.stop?-400:-300), (uint16)(1600*speed_k_limit));
        speed_out = 0.5f*speed_out_pre + 0.5f*speed_out;//低通滤波
        speed_out_pre = speed_out;
        if(flag.ramp == 1)
            speed_out = 0;
        speed_output = speed_out;
    }
    else if(Balance_mode == 1)//三轮
    {
        if(speed_cnt % 2 == 0)//10ms
        {
            if(flag.obstacle == 2 || flag.obstacle == 4)
            {
                target_speed[Balance_mode] = 250;
                pid_speed[Balance_mode].p = 1.1f * pid_spd_set[0];
            }
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
            if(flag.obstacle == 0)
                pid_speed[Balance_mode].error = target_speed[Balance_mode] - myfabs(0.7f*pid_dir[Balance_mode].error)
                    - flimit(8*(myfabs(pid_dir[Balance_mode].error)-myfabs(pid_dir[Balance_mode].preError[4])), 30) - car_speed_now;
            else
                pid_speed[Balance_mode].error = target_speed[Balance_mode] - car_speed_now;

            pid_speed[Balance_mode].deriv = pid_speed[Balance_mode].error - pid_speed[Balance_mode].preError[4];
            pid_speed[Balance_mode].preError[4] = pid_speed[Balance_mode].preError[3];
            pid_speed[Balance_mode].preError[3] = pid_speed[Balance_mode].preError[2];
            pid_speed[Balance_mode].preError[2] = pid_speed[Balance_mode].preError[1];
            pid_speed[Balance_mode].preError[1] = pid_speed[Balance_mode].preError[0];
            pid_speed[Balance_mode].preError[0] = pid_speed[Balance_mode].error;
            if(myfabs(pid_speed[Balance_mode].error) > pid_speed[Balance_mode].errlimit && time_count < 500)
                pid_speed[Balance_mode].output = target_speed[Balance_mode]
                    +0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error
                        +pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv;
            else
            {
                pid_speed[Balance_mode].integ += pid_speed[Balance_mode].error * pid_speed[Balance_mode].i*0.01f;
                pid_speed[Balance_mode].integ = flimit_ab(pid_speed[Balance_mode].integ, (int16)(-pid_speed[Balance_mode].intlimit/2), (int16)(pid_speed[Balance_mode].intlimit));
                pid_speed[Balance_mode].output = target_speed[Balance_mode]
                    +0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error
                        +pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv
                            +pid_speed[Balance_mode].integ;
            }
            //---------------------------
            speed_out = pid_speed[Balance_mode].output;
            speed_out = 0.5f*speed_out_pre + 0.5f*speed_out;//低通滤波
            if(speed_out - speed_out_pre > spd_acc && speed_out > 200)
                speed_out = speed_out_pre + spd_acc;
            speed_out_pre = speed_out;
        }
    }

    //ftestVal[3] = speed_out;/////////////////////////////////////

    if(flag.En_spd == 0 || spd_en == 0)       //直立模式或调试模式不输出速度
    {
        speed_out = 0;
    }
    if(speed_cnt >= 50)//100ms计数器清零
    {
        speed_cnt = 0;
    }

//    ftestVal[0] = target_speed[Balance_mode] - myfabs(0.7f*pid_dir[Balance_mode].error)
//                    - flimit(8*(myfabs(pid_dir[Balance_mode].error)-myfabs(pid_dir[Balance_mode].preError[4])), 30);
//    ftestVal[1] = target_speed[Balance_mode];   /////////////////////////////////////
//    ftestVal[2] = car_speed_now;                /////////////////////////////////////
//    ftestVal[3] = pid_speed[Balance_mode].integ;/////////////////////////////////////
//    ftestVal[4] = speed_out;                    //////////////////////////

    return (int)speed_out;
}
//-----------------速度控制-以上-----------------------------

//----------------电机输出--以下不必大改-----------------------
void motor_out(int angle_out, int speed_out, int dir_out)
{
    static uint16 error=0, cnt = 0;
    static int L_Speed=0, R_Speed=0;                        //左右轮最终输出
    static int AngleSpeedSum=0;                            //角度控制和速度控制之和
    static int DirectionMore=0;                             //转向饱和后寄存变量

    if(flag.stop == 1 && cnt < 500)
        cnt++;
    if(cnt > 20 && cnt < 400)
    {
        AngleSpeedSum = speed_out;
        AngleSpeedSum = limit_ab(AngleSpeedSum, (Balance_mode ? -150 : -300), 200);
    }
    else if(cnt > 400)
    {
        AngleSpeedSum = 0;
        dir_out = 0;
        flag.lost = 1;
    }
    else
    {
        if(Balance_mode == 0)//两轮
        {
            AngleSpeedSum = angle_out-speed_out;
            if(flag.ramp == 1)
                AngleSpeedSum = limit_ab(AngleSpeedSum, 0, 900);
            else if(flag.ramp == 2)
                AngleSpeedSum = limit_ab(AngleSpeedSum, -100, 300);
            else
                AngleSpeedSum = limit_ab(AngleSpeedSum, -700, 900);
        }
        else if(Balance_mode == 1)//三轮
        {
            AngleSpeedSum = angle_out+speed_out;
            if(flag.broken_road == 1)
                AngleSpeedSum = limit_ab(AngleSpeedSum, -250, 800);
            else if(flag.obstacle == 3)
                AngleSpeedSum = limit_ab(AngleSpeedSum, -600, 800);
            else
                AngleSpeedSum = limit_ab(AngleSpeedSum, 0, 800);
        }
    }
    if(myfabs(AngleSpeedSum/(myfabs(car_speed_now)+1)) > 50 && myabs(AngleSpeedSum)> 100 && time_count>1000) error++;
    else error = 0;
    if(error > 500 && flag.mode==MODE_START && flag.lost == 0)
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
    //ftestVal[1] = L_Speed;////////////////////////////
    //ftestVal[2] = R_Speed;////////////////////////////

    //以下为电机正反控制，以L_Speed>0和R_Speed>0为电机正转，不作修改！！！！
    if(L_Speed>0&&L_Speed<20.0) L_Speed=20.0;       //克服电机死区电压
    else if(L_Speed<0&&L_Speed>-20.0) L_Speed=-20.0;

    if(R_Speed>0&&R_Speed<20.0) R_Speed=20.0;
    else if(R_Speed<0&&R_Speed>-20.0) R_Speed=-20.0;

    if(time_count<500)
    {
        if(time_count < 200)
        {
            L_Speed = 0;
            R_Speed = 0;
        }
        else
        {
            L_Speed *= (time_count-200)/300.0;
            R_Speed *= (time_count-200)/300.0;
        }
    }

    if(flag.mode == MODE_DEBUG || (flag.lost == 1 && flag.mode != MODE_PWM_TEST))//
    {
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);
        if(flag.lost == 1 && flag.mode == MODE_START)
        {
            buzz_off();
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, 450);
            DisableInterrupts;
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
