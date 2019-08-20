#include "headfile.h"

void system_init(void);
void test(void);
void servoControl(void);
void ObstacleDetection(void);

uint16 distance = 0;
uint16 servo_duty = 550;
uint8 obstacle_pix2 = 40, obstacle_pix3 = 25;
uint8 obstacle_detection_cnt = 2;
uint8 obstacle_turn_mode = 0;
float k_servo = 0;

int main(void)
{
    system_init();
    flag.buzz = 1;

    while(1)
    {
        servoControl();
        ObstacleDetection();
        if(flag.mode == MODE_DEBUG)
        {
            displayDebug();
        }
    }
}

void system_init(void)
{
    get_clk();
    test();
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_LP, 17000, 0);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_LN, 17000, 0);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_RP, 17000, 0);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_RN, 17000, 0);
    ftm_pwm_init(SERVO_FTM, SERVO_CH, 330, 430); //三轮230  两轮590
    systick_delay_ms(100);
    FLASH_Init();
    systick_delay_ms(10);
    OLED_Init();
    pidInit();
    gpio_init(BUZZER_PIN, GPO, 0);
    gpio_init(HALL_PIN, GPI, 1);
    speed_encoder_init();
    IIC_init();
    IIC_init2();
    systick_delay_ms(10);
    BMX055_init();
    systick_delay_ms(10);
    ADC_init();
    communicate_init();
    displayUI();

    bt[0][0] = 0;                                bt[1][0] = 0;
    bt[0][1] = obstacle_turn_t[0];               bt[1][1] = obstacle_turn_t[1];
    bt[0][2] = bt[0][1] + obstacle_delay1[0];    bt[1][2] = bt[1][1] + obstacle_delay1[1];
    bt[0][3] = bt[0][2] + obstacle_turn_t[0];    bt[1][3] = bt[1][2] + obstacle_turn_t[1];
    bt[0][4] = bt[0][3] + obstacle_delay2[0];    bt[1][4] = bt[1][3] + obstacle_delay2[1];
    bt[0][5] = bt[0][4] + obstacle_turn_t[0];    bt[1][5] = bt[1][4] + obstacle_turn_t[1];
    bt[0][6] = bt[0][5] + obstacle_delay3[0];    bt[1][6] = bt[1][5] + obstacle_delay3[1];
    bt[0][7] = bt[0][6] + obstacle_turn_t[0];    bt[1][7] = bt[1][6] + obstacle_turn_t[1];

    pit_init_ms(pit0,2)
    set_irq_priority(PIT0_IRQn,1);
    enable_irq(PIT0_IRQn);
    EnableInterrupts;
}

void ObstacleDetection(void)
{
    uint8 buf[2];
    static uint8 cnt = 0;
    static uint32 t = 0;
    if(Balance_mode == 0)
    {
        if(CarAttitude.Pitch > -12)
        {
            simiic_read_buf2(0xA0>>1, 0x00, IIC, buf, 2);//两轮
            distance = (buf[0]<<8) | buf[1];
        }
        else
        {
            simiic_read_buf2(0xC0>>1, 0x00, IIC, buf, 2);//两轮
            distance = (buf[0]<<8) | buf[1];
        }
    }
    else if(Balance_mode == 1)
    {
        simiic_read_buf2(0xB0>>1, 0x00, IIC, buf, 2);//三轮
        distance = (buf[0]<<8) | buf[1];
    }
    if(distance > 1300)
        distance = 1300;

    if(((distance > 100 && distance < 900
       && obstacle_pix > (Balance_mode?obstacle_pix3:obstacle_pix2)
       && flag.ob_detection == 1) ||
       (obstacle_pix > (Balance_mode?obstacle_pix3:obstacle_pix2)
       && flag.ob_detection == 2) ||
       (distance > 100 && distance < 900
       && flag.ob_detection == 3))
       && myfabs(pid_dir[Balance_mode].error) < (Balance_mode ? 25 : 20)
       && (time_count-t > 500*2 || t == 0)
       && (time_count > tim.obstacle_a*500 && time_count < tim.obstacle_b*500
           || time_count > tim.obstacle_c*500 && time_count < tim.obstacle_d*500)//区间检测
       && flag.obstacle < 2
       && flag.circle < 2
       && flag.ramp != 1
           )
    {
        cnt++;
        flag.obstacle = 1;
        if(cnt >= obstacle_detection_cnt
           && (distance < 800 || obstacle_pix > (Balance_mode ? 30 : 70)))
        {
            if(obstacle_turn_mode == 0 && Balance_mode)
            {
                flag.obstacle = 3;
                if(car_speed_now > 250)
                    obstacle_step = 1;
                else
                    obstacle_step = -1;
            }
            else if(obstacle_cnt2 == 1 && Balance_mode == 0)
            {
                flag.obstacle = 2;
                obstacle_step = 1;
            }
            else
            {
                flag.obstacle = 4;
                obstacle_step = 1;
            }
            obstacle_time = 0;
            t = time_count;
            flag.buzz = 2;///////////////////////////////
        }
    }
    else if(cnt > 0)
        cnt--;
    else if(cnt == 0 && flag.obstacle == 1)
        flag.obstacle = 0;

}

void servoControl(void)
{
    uint16 servo = 540;
    static uint16 servo_last = 540;
    if(Balance_mode == 0 || flag.mode_switch == 1)
    {
        if(flag.ramp > 0)
            speed_output = -5;
        servo = limit_ab((int)(servo_duty-k_servo*0.01f*speed_output), 470, 560);
        if(servo - servo_last > 20) servo = servo_last + 20;
        else if(servo - servo_last < -20) servo = servo_last - 20;
        ftm_pwm_duty(SERVO_FTM, SERVO_CH, servo);
        servo_last = servo;
    }
    else
    {
        ftm_pwm_duty(SERVO_FTM, SERVO_CH, 228);
    }
}

void testServo(void)
{
    uint16 duty = 500;
    uint8 key=0;
    systick_delay_ms(100);
    ftm_pwm_init(SERVO_FTM, SERVO_CH, 330, duty);
    gpio_init(KEY_PIN_U, GPI, 0); //down
    gpio_init(KEY_PIN_D, GPI, 0); //right
    gpio_init(KEY_PIN_L, GPI, 0); //left
    gpio_init(KEY_PIN_R, GPI, 0); //up
    gpio_init(KEY_PIN_M, GPI, 0); //mid
    OLED_Init();
    while(1)
    {
        key = readKey();
        if(key == 1)
        {
            duty += 5;
        }
        else if(key == 3)
        {
            duty -= 5;
        }
        ftm_pwm_duty(SERVO_FTM, SERVO_CH, duty);
        OLED_Print_uint16(0, 0, duty, 1, 1);
    }
}

void testUART(void)
{
    uint8 i = 0;
    communicate_init();
    while(1)
    {
        i++;
        uart_putchar(COM_UART, i);
        systick_delay_ms(5);
    }
}

void testPWM(void)
{
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_LP, 17000, 200);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_LN, 17000, 400);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_RP, 17000, 600);
    ftm_pwm_init(MOTOR_FTM, MOTOR_CH_RN, 17000, 800);
    while(1);
}

void test(void)
{
}

//void remote(void)
//{
//    uint8 ch = 0;
//    int16 ll,rr;
//    uart_getchar(DEBUG_UART, &ch);
//    ll=0;rr=0;
//    if(ch&0x01 == 1)
//    {
//        rr-=120;
//        ll+=120;
//    }
//    if(ch>>1&0x01 == 1)
//    {
//        rr-=200;
//        ll-=200;
//    }
//    if(ch>>2&0x01 == 1)
//    {
//        rr+=120;
//        ll-=120;
//    }
//    if(ch>>3&0x01 == 1)
//    {
//        ll+=300;
//        rr+=300;
//    }
//    if(ch>>4 > 0)
//    {
//        ll=0;
//        rr=0;
//    }
//    ll=(int16)(ll*1.3f);
//    if(ll>0)
//    {
//        if(ll>500)
//            ll = 500;
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,(int)ll);
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,0);
//    }
//    else
//    {
//        ll = -ll;
//        if(ll>500)
//            ll = 500;
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,(int)ll);
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);
//    }
//    if(rr>0)
//    {
//        if(rr>500)
//            rr = 500;
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,(int)rr);
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,0);
//    }
//    else
//    {
//        rr = -rr;
//        if(rr>500)
//            rr = 500;
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,(int)rr);
//        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);
//    }
//}

//
//void play(uint8 cmd)
//{
//    uint8 sum = 0xAA, i;
//    uint8 ch[7][4] = { {0x13, 0x01, 0x1E, 0x00},//vol+++
//                        {0x07, 0x02, 0x00, 0x01},//start
//                        {0x07, 0x02, 0x00, 0x02},//ob
//                        {0x07, 0x02, 0x00, 0x03},//br
//                        {0x07, 0x02, 0x00, 0x04},//rh
//                        {0x07, 0x02, 0x00, 0x05},//ch
//                        {0x07, 0x02, 0x00, 0x06}};//pd
//    for(i = 0; i < 4; i++)
//        sum += ch[cmd][i];
//    uart_putchar(DEBUG_UART, 0xAA);
//    if(cmd == 0)
//        uart_putbuff(DEBUG_UART, ch[cmd], 3);
//    else
//        uart_putbuff(DEBUG_UART, ch[cmd], 4);
//    uart_putchar(DEBUG_UART, sum);
//}
