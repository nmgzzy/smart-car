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

int BalanceControl(void)
{
    static float camera_angle, camera_angle_last;
    uint8 offset = 0;
    static uint8 cnt = 0,error=0, ramp_cnt = 0;
    static uint16 ramp_timecnt= 0;
    int angle_out = 0;
    float t;
    cnt++; if(cnt >= 10) cnt = 0;

    if(cnt)
        BMX055_DataRead(&Q_raw, 0);
    else
    {
        BMX055_DataRead(&Q_raw, 1);
        if(myabs(Q_raw.Mag-mag_threshold) > 500 && time_count > stop_time*500 && flag.stop == 0)
        {
            if(flag.mode == MODE_DEBUG)
            {
                flag.buzz = 1;
            }
            else
            {
                flag.stop = 1;
                printLog("Mag stop");
            }
        }
    }
    //正常1500-1800
    //有磁铁Mag<1400||Mag>1900 变化率很大
    //ftestVal[2] = Q_raw.Mag;//////////////////////////////


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
    camera_angle = CarAttitude.Pitch;
    camera_angle = flimit_ab(camera_angle,-31,2);
    camera_angle = 0.2f*camera_angle + 0.8f*camera_angle_last;
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
    if(error>50)
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
        /*if(flag.ramp)
            pid_angle[Balance_mode].output = 0.1f*pid_angle[Balance_mode].output;*/
        if(flag.ramp == 0 && CarAttitude.Pitch > -21 && time_count > 1000
           && !flag.broken_road && !flag.obstacle && flag.circle < 2)
        {
            ramp_cnt++;
            if(ramp_cnt > 10)
            {
                flag.ramp = 1;
                ramp_cnt = 0;
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
            if(ramp_cnt > 100)
            {
                flag.ramp = 0;
                ramp_cnt = 0;
            }
        }
        if(flag.ramp > 0)
        {
            ramp_timecnt++;
            if(ramp_timecnt > 800)
            {
                flag.ramp = 0;
            }
        }
        else
            ramp_timecnt = 0;
    }
    angle_out = (int)pid_angle[Balance_mode].output;
    if(flag.En_std == 0)
        angle_out = 0;


    //ftestVal[0] = (CarAttitude.Pitch)*100;///////////////////
    //ftestVal[1] = (CarAttitudeRate.Yaw)*10;///////////////////
    //ftestVal[5] = angle_out;////////////////////

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
int8 circle_dir = 1;
int8 crcl_cnt = -1;
int8 crcl_cnt2 = 0;
//uint16 cl_threshold[2][4]={0};
float yaw_integ = 0;

uint16 obstacle_time = 0;
uint16 obstacle_turn_t[2] = {70,80};
uint16 obstacle_turn_k[2] = {400,500};
uint16 obstacle_delay[2]  = {70,60};
uint16 obstacle_delay_out[2] = {10,100};
int8 obstacle_turn_dir[3] = {1,1,1};
uint16 bt[2][8];
uint8 obstacle_cnt = 0;

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
        ad_data_now[i] = (uint16)(0.45f*ad_raw_data_pre[i][0] + 0.25f*ad_raw_data_pre[i][1] +
            0.15f*ad_raw_data_pre[i][2] + 0.1f*ad_raw_data_pre[i][3] + 0.05f*ad_raw_data_pre[i][4]);
    }
}

float ErrorCalculate(uint8 mode)
{
    float error1, error2;
    float error_out, kcl = 0, tAngle = -10;
    static float error_out_last;
    static int16 difX_div[5] = {0};
    int16 difH, difV, difX;
    uint16 sumH, sumHM2, sumHM3, sumV;//, sumHM3, sumHM4; sumV,sumX
    static float sumHM[5];
    ADC_get();
    difH = ad_data_now[LH]-ad_data_now[RH];
    difV = ad_data_now[LV]-ad_data_now[RV];
    difX = ad_data_now[LX]-ad_data_now[RX];
    sumH = ad_data_now[LH]+ad_data_now[RH];
    sumV = ad_data_now[LV]+ad_data_now[RV];
    //sumX = ad_data_now[LX]+ad_data_now[RX]+1;
    for(uint8 i=4; i>0; i--)
        sumHM[i] = sumHM[i-1];
    sumHM[0] = sumH + ad_data_now[MD] + 1;
    sumHM3 = (uint16)(0.3f*sumHM[0] + 0.25f*sumHM[1] + 0.2f*sumHM[2] + 0.15f*sumHM[3] + 0.1f*sumHM[4]);
    sumHM2 = (uint16)(sumH + k_md[mode]*ad_data_now[MD] + 1);
    tAngle = flimit_ab(CarAttitude.Pitch, -20, 0);
    for(uint8 i=4; i>0; i--)
        difX_div[i] = difX_div[i-1];
    difX_div[0] = (int16)(myabs(difX)*0.5f + difX_div[0]*0.5f);
    error1 = 10 * (k_hv[mode]*difH + (10.0f-k_hv[mode])*difV + k_x[mode]*difX)/sumHM2;
    error_out = error1;

    ftestVal[0] = difH;             /////////////////////////
    //ftestVal[0] = myabs(difV);             ///////////////////////
    ftestVal[1] = difX_div[2];            /////////////////////////
    ftestVal[2] = sumHM3/2;                    ///////////////////////
    ftestVal[3] = ad_data_now[MD];          ///////////////////////
    ftestVal[4] = 500*flag.circle;          ///////////////////////
    ftestVal[5] = sumV;                   /////////////////////////

    //--------------检环-------------------
    if(Balance_mode == 0)
    {
        if(time_count>1000 && sumHM3 > -70*tAngle+2820 && ad_data_now[MD] > -57*tAngle+736//3500  1200
            && sumV < 1500 && flag.circle <= 1 && !flag.obstacle && !flag.broken_road)
            //左中右和很大  中间很大(保证车在中间)
        {
            flag.circle = 1;
            if(((difX_div[0] - difX_div[2])*(difX_div[2] - difX_div[4]) < 0 && myabs(difX_div[2]) > -14.3*tAngle+443)
                || (myabs(difX_div[2]) > -28.6*tAngle+586))//500 700
            {
                //外八电感之差变号  外八电感之差较大
                flag.circle = 2;
                yaw_integ = 0;
                crcl_cnt++;
                if(crcl_cnt >= cl_num) crcl_cnt = 0;
                if(circle_dir < 0)
                    crcl_cnt2 = cl_num - 1 - crcl_cnt;
                else
                    crcl_cnt2 = crcl_cnt;
                flag.buzz = 1;/////////////////////////////////////
                circle_time_count[1] = 0;
            }
        }
        if(flag.circle == 2)
        {
            yaw_integ += CarAttitudeRate.Yaw*0.002f;
            circle_time_count[1]++;
            error2 = k_circle[crcl_cnt2]*10*(k_hv_cin[mode]*difH+(10.0f-k_hv_cin[mode])*difV)/sumHM2
                + circle_dir*circle_offset[crcl_cnt2];
            //k=1,offset=20
            if(circle_size[crcl_cnt2] == 1)
                kcl = trapezoid_fun(circle_time_count[1], 70, 120, 100, 1);
            else if(circle_size[crcl_cnt2] == 2)
                kcl = trapezoid_fun(circle_time_count[1], 90, 130, 125, 1);
            else if(circle_size[crcl_cnt2] == 3)
                kcl = trapezoid_fun(circle_time_count[1], 110, 140, 150, 1);
            else if(circle_size[crcl_cnt2] == 4)
                kcl = trapezoid_fun(circle_time_count[1], 140, 150, 175, 1);
            else if(circle_size[crcl_cnt2] == 5)
                kcl = trapezoid_fun(circle_time_count[1], 170, 160, 200, 1);
            //大环kcircle = 1; circleoffset = 25;
            error_out = kcl*error2 + (1-kcl)*error1;
            if(myfabs(yaw_integ) > 295+5*circle_size[crcl_cnt2])
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
            if(circle_time_count[1] > 500)
                flag.circle = 0;
            if(circle_size[crcl_cnt2] == 1)
            {
                error2 = k_cout[crcl_cnt2]*16*(10*difH+1*difX-3*difV)/(sumH+1)
                    - 0.3*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 80, 100, 110, 1);
            //小环testPar[0] = 1.6  testPar[3] = 0.3
            }
            else if(circle_size[crcl_cnt2] == 2)
            {
                error2 = k_cout[crcl_cnt2]*14*(10*difH+1*difX-3*difV)/(sumH+1)
                    - 0.42f*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 97, 120, 125, 1);
            }
            else if(circle_size[crcl_cnt2] == 3)
            {
                error2 = k_cout[crcl_cnt2]*11*(10*difH+1*difX-3*difV)/(sumH+1)
                    - 0.8f*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 115, 130, 140, 1);
            }
            else if(circle_size[crcl_cnt2] == 4)
            {
                error2 = k_cout[crcl_cnt2]*8*(10*difH+1*difX-3*difV)/(sumH+1)
                    - 0.9*k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 132, 160, 155, 1);
            }
            else if(circle_size[crcl_cnt2] == 5)
            {
                error2 = k_cout[crcl_cnt2]*6.7f*(10*difH+1*difX-3*difV)/(sumH+1)
                    - k_cout_offset[crcl_cnt2]*circle_dir*circle_offset[crcl_cnt2];
                kcl = trapezoid_fun(circle_time_count[1], 150, 160, 170, 1);
            }
            //大环testPar[0] = 0.67  testPar[3] = 1

            error_out = kcl*error2 + (1-kcl)*error1;
        }
    }
    else if(Balance_mode == 1)
    {
        if(time_count>1000 && sumHM3 >5500 && ad_data_now[MD] > 2800 && sumV < 2800
           && flag.circle <= 1 && !flag.obstacle && !flag.broken_road)
            //左中右和很大，中间很大
        {
            flag.circle = 1;
            flag.buzz = 1;////////////////////////////
            if((difX_div[0]-difX_div[2]<0 && difX_div[2]-difX_div[4]>0 && difX_div[2] > 1500)
                || difX_div[2] > 2000)
            {
                flag.circle = 2;
                yaw_integ = 0;
                crcl_cnt++;
                if(crcl_cnt >= cl_num) crcl_cnt = 0;
                if(circle_dir < 0)
                    crcl_cnt2 = cl_num - 1 - crcl_cnt;
                else
                    crcl_cnt2 = crcl_cnt;
                //flag.buzz = 1;///////////////////////////
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
            //k_circle=1.4, offset=45~50
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
                //flag.buzz = 3;///////////////////////////
            }
        }
        else if(flag.circle == 3)
        {
            circle_time_count[1]++;
            if(circle_time_count[1] > 500)
                flag.circle = 0;
            error2 = k_cout[crcl_cnt2]*6.8*(10*difH+1*difX-2*difV)/(sumH+1)
                - circle_dir*k_cout_offset[crcl_cnt2]*circle_offset[crcl_cnt2];
            if(circle_size[crcl_cnt2] == 1)
                kcl = trapezoid_fun(circle_time_count[1], 60, 140, 90, 1);
            else if(circle_size[crcl_cnt2] == 2)
                kcl = trapezoid_fun(circle_time_count[1], 70, 152, 98, 1);
            else if(circle_size[crcl_cnt2] == 3)
                kcl = trapezoid_fun(circle_time_count[1], 80, 165, 107, 1);
            else if(circle_size[crcl_cnt2] == 4)
                kcl = trapezoid_fun(circle_time_count[1], 90, 178, 116, 1);
            else if(circle_size[crcl_cnt2] == 5)
                kcl = trapezoid_fun(circle_time_count[1], 100, 190, 125, 1);
            error_out = kcl*error2 + (1-kcl)*error1;
        }
    }
    if(flag.circle > 0)
        error_out = 0.7*error_out + 0.3*error_out_last;
    if(flag.circle == 1)
    {
        circle_time_count[0]++;
        if(circle_time_count[0] > circle_size[crcl_cnt2]*125+375)
        {
            circle_time_count[0] = 0;
            flag.circle = 0;
            //flag.buzz = 3;////////////////////////////
        }
    }
    else if(flag.circle > 1)
    {
        circle_time_count[0]++;
        if(circle_time_count[0] > circle_size[crcl_cnt2]*250+1250)
        {
            circle_time_count[0] = 0;
            flag.circle = 0;
            //flag.buzz = 2;////////////////////////////
        }
    }
    else
        circle_time_count[0] = 0;
    //--------------检环结束------------------

    //ftestVal[0] = error1*10;             /////////////////////////
    //ftestVal[1] = error2*10;             /////////////////////////
    ftestVal[6] = kcl*1500;              ///////////////////////////
    //ftestVal[4] = error_out*10;           ///////////////////////////
    error_out_last = error_out;
    return error_out;
}

int DirectionControl(void)
{
    int dir_out = 0;
    static int dir_out_last = 0;
    //static int dir_out_pre[5] = {0};
    float E_error = 0;
    float error_offset = 0;
    float speed_rate = 1;
    float speed_k = 1;
    E_error = ErrorCalculate(Balance_mode);
    if(time_count == 1)
    {
        bt[0][0] = 0;                                bt[1][0] = 0;
        bt[0][1] = obstacle_turn_t[0];               bt[1][1] = obstacle_turn_t[1];
        bt[0][2] = bt[0][1] + obstacle_delay[0];     bt[1][2] = bt[1][1] + obstacle_delay[1];
        bt[0][3] = bt[0][2] + obstacle_turn_t[0];    bt[1][3] = bt[1][2] + obstacle_turn_t[1];
        bt[0][4] = bt[0][3] + obstacle_delay_out[0]; bt[1][4] = bt[1][3] + obstacle_delay_out[1];
        bt[0][5] = bt[0][4] + obstacle_turn_t[0];    bt[1][5] = bt[1][4] + obstacle_turn_t[1];
        bt[0][6] = bt[0][5] + obstacle_delay[0]-20;  bt[1][6] = bt[1][5] + obstacle_delay[1]-50;
        bt[0][7] = bt[0][6] + obstacle_turn_t[0];    bt[1][7] = bt[1][6] + obstacle_turn_t[1];
    }
    if(flag.obstacle == 2)
    {
        flag.buzz = 4;
        speed_rate = car_speed_now / target_speed[Balance_mode];
        if(speed_rate > 1.2f) speed_rate = 1.2f;
        else if(speed_rate < 0.6f) speed_rate = 0.6f;
        speed_k = -2*speed_rate/3 + 1.7;
        obstacle_time++;
        if(obstacle_time > bt[Balance_mode][0] && obstacle_time < bt[Balance_mode][1])
            error_offset =  obstacle_turn_dir[obstacle_cnt] * circle_dir
                * obstacle_turn_k[Balance_mode] * speed_k;
        else if(obstacle_time > bt[Balance_mode][2] && obstacle_time < bt[Balance_mode][3]
            || obstacle_time > bt[Balance_mode][4] && obstacle_time < bt[Balance_mode][5])
            error_offset = -obstacle_turn_dir[obstacle_cnt] * circle_dir
                * obstacle_turn_k[Balance_mode] * speed_k;
        else if(obstacle_time > bt[Balance_mode][6] && obstacle_time < bt[Balance_mode][7])
        {
            pid_dir[Balance_mode].error = E_error;
            pid_dir[Balance_mode].deriv = pid_dir[Balance_mode].error - pid_dir[Balance_mode].preError[2];
            pid_dir[Balance_mode].preError[2] = pid_dir[Balance_mode].preError[1];
            pid_dir[Balance_mode].preError[1] = pid_dir[Balance_mode].preError[0];
            pid_dir[Balance_mode].preError[0] = pid_dir[Balance_mode].error;
            pid_dir[Balance_mode].output = pid_dir[Balance_mode].p * pid_dir[Balance_mode].error
                + pid_dir[Balance_mode].d * pid_dir[Balance_mode].deriv;
            error_offset = obstacle_turn_dir[obstacle_cnt] * circle_dir
                * obstacle_turn_k[Balance_mode] * speed_k
                + 0.2f * pid_dir[Balance_mode].output;
        }
        else
            error_offset = 0;
        if(obstacle_time > bt[Balance_mode][7])
        {
            flag.obstacle = 0;
            obstacle_cnt++;
            if(obstacle_cnt > 2)
                obstacle_cnt = 0;
        }
        //角速度内环---------
        pid_yaw[Balance_mode].error = CarAttitudeRate.Yaw + error_offset;
        pid_yaw[Balance_mode].deriv = pid_yaw[Balance_mode].error - pid_yaw[Balance_mode].preError[1];
        pid_yaw[Balance_mode].preError[1] = pid_yaw[Balance_mode].preError[0];
        pid_yaw[Balance_mode].preError[0] = pid_yaw[Balance_mode].error;
        pid_yaw[Balance_mode].output = pid_yaw[Balance_mode].p * pid_yaw[Balance_mode].error + pid_yaw[Balance_mode].d * pid_yaw[Balance_mode].deriv;
        dir_out = (int)pid_yaw[Balance_mode].output;
    }
    else
    {
        obstacle_time=0;
    //方向pid--------------------
        //偏差外环---------
        pid_dir[Balance_mode].error = E_error;
        pid_dir[Balance_mode].deriv = pid_dir[Balance_mode].error - pid_dir[Balance_mode].preError[4];
        pid_dir[Balance_mode].preError[4] = pid_dir[Balance_mode].preError[3];
        pid_dir[Balance_mode].preError[3] = pid_dir[Balance_mode].preError[2];
        pid_dir[Balance_mode].preError[2] = pid_dir[Balance_mode].preError[1];
        pid_dir[Balance_mode].preError[1] = pid_dir[Balance_mode].preError[0];
        pid_dir[Balance_mode].preError[0] = pid_dir[Balance_mode].error;
        pid_dir[Balance_mode].output = pid_dir[Balance_mode].p * pid_dir[Balance_mode].error + pid_dir[Balance_mode].d * pid_dir[Balance_mode].deriv;

        //角速度内环---------
        if(Balance_mode == 1 && line_cy!=0 && line_width!=0
           && line_width < 75 && car_speed_now > 280)
            pid_yaw[Balance_mode].error = k_ei*pid_dir[Balance_mode].output
                +(1-k_ei)*pid_img[Balance_mode].output + CarAttitudeRate.Yaw;
        else
            pid_yaw[Balance_mode].error = pid_dir[Balance_mode].output + CarAttitudeRate.Yaw;
        pid_yaw[Balance_mode].deriv = pid_yaw[Balance_mode].error - pid_yaw[Balance_mode].preError[2];
        pid_yaw[Balance_mode].preError[2] = pid_yaw[Balance_mode].preError[1];
        pid_yaw[Balance_mode].preError[1] = pid_yaw[Balance_mode].preError[0];
        pid_yaw[Balance_mode].preError[0] = pid_yaw[Balance_mode].error;
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
        if(ad_data_now[LH]+ad_data_now[MD]+ad_data_now[RH]<120 && flag.En_dir == 1 && flag.mode != MODE_DEBUG && flag.start>0 && flag.obstacle==0)
        {
            flag.lost = 1;
            printLog("Signal lost");
        }
#endif
    }
    dir_out = limit(dir_out, (Balance_mode?600:400));

    /*if( (dir_out-dir_out_last) > testPar[1] )
        dir_out=dir_out_last+testPar[1];
    else if( (dir_out-dir_out_last) < -testPar[1] )
        dir_out=dir_out_last-testPar[1];*/

    dir_out = (int)(dir_out_last * 0.2f + dir_out * 0.8f);
    //ftestVal[1] = dir_out;
    dir_out_last = dir_out;
    //ftestVal[0] = E_error;    ////////////////////////////
    //ftestVal[1] = img_err;    ////////////////////////////
    //ftestVal[2] = pid_yaw[Balance_mode].error;

    ////////////////////////////
    return dir_out;
}
//---------------方向控制-以上--------------------


//---------------速度控制-------------------------
int16 target_speed[2] = {305, 310}, target_speed_max[2] = {305, 310};
uint16 spd_acc = 9;
float speed_k_limit = 1, car_speed_now = 0, car_speed_now3 = 0;
double path_length = 0;
uint8 swich_mode = 1;

int SpeedControl(void)
{
    static int16    left_spd_sum=0, right_spd_sum=0;
    static float    speed_out=0;
    static float    speed_last = 0;
    static float    Speedold[5]={0};
    static uint8    speed_cnt=0,spd_error = 0;
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
        car_speed_now3 = car_speed_now;
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
        car_speed_now3 = flimit_ab(car_speed_now, -200, 450);//当前速度限幅
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
    if(swich_mode == 0)
    {
        if(flag.mode_switch == 1)
            target_speed[1] = 0;
        else
            target_speed[1] = target_speed_max[1];
    }
    else
    {
        if(flag.mode_switch == 1)
            spd_en = 0;
        else
            spd_en = 1;
    }
    if(flag.stop == 1)
        target_speed[Balance_mode] = 0;
    else if(flag.slow_down == 1 || time_count > tim.slow_a*500 && time_count < tim.slow_b*500
            || time_count > tim.slow_c*500 && time_count < tim.slow_d*500
            || time_count > tim.slow_e*500 && time_count < tim.slow_f*500)
        target_speed[Balance_mode] = (int16)(0.7f * target_speed_max[Balance_mode]);
    else if(flag.circle == 1 || flag.circle == 3)
        target_speed[Balance_mode] = 270;
    else if(flag.circle == 2 && circle_size[crcl_cnt2] > 2
       && myfabs(yaw_integ) > 30 && myfabs(yaw_integ) < 200+circle_size[crcl_cnt2]*10)
        target_speed[Balance_mode] = (int16)(1.2f * target_speed_max[Balance_mode]);
    else if(Balance_mode == 1 && line_cy!=0 && line_width!=0
        && line_width < 75 && myfabs(pid_img[Balance_mode].error) < 20 && myfabs(pid_dir[Balance_mode].error) < 20
        && !flag.slow_down && !flag.circle && !flag.obstacle && !flag.broken_road)
        target_speed[Balance_mode] = (int16)(1.1f * target_speed_max[Balance_mode]);
    else if(flag.ramp > 0 && Balance_mode == 1)
    {
        target_speed[Balance_mode] = 230;
        flag.buzz = 1;
    }
    else if(flag.broken_road > 0)
    {
        target_speed[Balance_mode] = 260;
        flag.buzz = 1;
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
    }
    else if(Balance_mode == 1)//三轮
    {
        if(speed_cnt % 5 == 0)//10ms
        {
            if(flag.obstacle > 0)
            {
                target_speed[Balance_mode] = 270;
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
                pid_speed[Balance_mode].error = target_speed[Balance_mode] - myfabs(0.8f*pid_dir[Balance_mode].error)
                    - flimit(8*(myfabs(pid_dir[Balance_mode].error)-myfabs(pid_dir[Balance_mode].preError[4])), 30) - car_speed_now3;
            else
                pid_speed[Balance_mode].error = target_speed[Balance_mode] - car_speed_now3;
            pid_speed[Balance_mode].deriv = pid_speed[Balance_mode].error - pid_speed[Balance_mode].preError[0];
            pid_speed[Balance_mode].preError[0] = pid_speed[Balance_mode].error;
            if(myfabs(pid_speed[Balance_mode].error) > pid_speed[Balance_mode].errlimit && time_count < 500)
                pid_speed[Balance_mode].output = target_speed[Balance_mode]
                    +0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error
                        +pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv;
            else
            {
                pid_speed[Balance_mode].integ += pid_speed[Balance_mode].error * pid_speed[Balance_mode].i*0.01f;
                pid_speed[Balance_mode].integ = flimit(pid_speed[Balance_mode].integ, (int16)(pid_speed[Balance_mode].intlimit));
                pid_speed[Balance_mode].output = target_speed[Balance_mode]
                    +0.1f*pid_speed[Balance_mode].p*pid_speed[Balance_mode].error
                        +pid_speed[Balance_mode].d*pid_speed[Balance_mode].deriv
                            +pid_speed[Balance_mode].integ;
            }
            //---------------------------
            speed_out = pid_speed[Balance_mode].output;
            if(speed_out - speed_out_pre > spd_acc)
                speed_out = speed_out_pre + spd_acc;
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
    speed_out = 0.5f*speed_out_pre + 0.5f*speed_out;//低通滤波
    speed_out_pre = speed_out;
    //ftestVal[5] = car_speed_now;                /////////////////////////////////////
    //ftestVal[4] = target_speed[Balance_mode];   /////////////////////////////////////
    //ftestVal[2] = pid_speed[Balance_mode].integ;/////////////////////////////////////
    //ftestVal[3] = speed_out;                    //////////////////////////
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
    if(cnt > 10 && cnt < 400)
    {
        AngleSpeedSum = speed_out;
        AngleSpeedSum = limit_ab(AngleSpeedSum, (Balance_mode ? -200 : -400), 200);
    }
    else if(cnt > 400)
    {
        AngleSpeedSum = 0;
        dir_out = 0;
    }
    else
    {
        if(Balance_mode == 0)//两轮
        {
            AngleSpeedSum = angle_out-speed_out;
            AngleSpeedSum = limit_ab(AngleSpeedSum, -700, 900);
        }
        else if(Balance_mode == 1)//三轮
        {
            AngleSpeedSum = angle_out+speed_out;
            AngleSpeedSum = limit_ab(AngleSpeedSum, (flag.broken_road==1 ? -200 : 0), 800);
        }
    }
    //ftestVal[6] = AngleSpeedSum;/////////////////////
    if(myfabs(AngleSpeedSum/(myfabs(car_speed_now)+1)) > 50 && myabs(AngleSpeedSum)> 100 && time_count>1000) error++;
    else error = 0;
    if(error > 500 && flag.mode!=MODE_DEBUG)
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
