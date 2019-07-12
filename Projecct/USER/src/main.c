#include "headfile.h"

void system_init(void);
void test(void);
void remote(void);

uint16 distance = 0;
uint16 servo_duty = 520;

int main(void)
{
    system_init();
    uint8 buf[2];
    uint8 cnt = 0;
    flag.buzz = 1;
    uint32 t = 0;

    while(1)
    {
        if(Balance_mode == 0 || flag.mode_switch == 1)
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, servo_duty);
        else
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, 235);
        if(flag.mode != MODE_START)
        {
            if(flag.mode == MODE_DEBUG)
            {
                displayDebug();
            }
        }
        if(Balance_mode == 0)
        {
            simiic_read_buf2(0xA0>>1, 0x00, IIC, buf, 2);//两轮
            distance = (buf[0]<<8) | buf[1];
        }
        else if(Balance_mode == 1)
        {
            simiic_read_buf2(0xB0>>1, 0x00, IIC, buf, 2);//三轮
            distance = (buf[0]<<8) | buf[1];
        }
        if(distance > 2000)
            distance = 0;
        if((distance > 100 && distance < 900 && obstacle_pix > (Balance_mode?25:40))
           && myfabs(pid_dir[Balance_mode].error) < (Balance_mode ? 20 : 15)
           && (time_count-t > 500*3 || t == 0)
           && (time_count > tim.obstacle_a*500 && time_count < tim.obstacle_b*500
               || time_count > tim.obstacle_c*500 && time_count < tim.obstacle_d*500)//区间检测
           && flag.obstacle < 2
           && flag.circle < 2)
        {
            cnt++;
            flag.obstacle = 1;
            if(cnt >= 2 && (distance < 800 || obstacle_pix > (Balance_mode ? 30 : 90)))
            {
                flag.obstacle = 2;
                t = time_count;
            }
        }
        else if(cnt > 0)
            cnt--;
        else if(cnt == 0 && flag.obstacle == 1)
            flag.obstacle = 0;
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
    pit_init_ms(pit0,2)
    set_irq_priority(PIT0_IRQn,1);
    enable_irq(PIT0_IRQn);
    EnableInterrupts;
}

void remote(void)
{
    uint8 ch = 0;
    int16 ll,rr;
    uart_getchar(DEBUG_UART, &ch);
    ll=0;rr=0;
    if(ch&0x01 == 1)
    {
        rr-=120;
        ll+=120;
    }
    if(ch>>1&0x01 == 1)
    {
        rr-=200;
        ll-=200;
    }
    if(ch>>2&0x01 == 1)
    {
        rr+=120;
        ll-=120;
    }
    if(ch>>3&0x01 == 1)
    {
        ll+=300;
        rr+=300;
    }
    if(ch>>4 > 0)
    {
        ll=0;
        rr=0;
    }
    ll=(int16)(ll*1.3f);
    if(ll>0)
    {
        if(ll>500)
            ll = 500;
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,(int)ll);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,0);
    }
    else
    {
        ll = -ll;
        if(ll>500)
            ll = 500;
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LN,(int)ll);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_LP,0);
    }
    if(rr>0)
    {
        if(rr>500)
            rr = 500;
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,(int)rr);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,0);
    }
    else
    {
        rr = -rr;
        if(rr>500)
            rr = 500;
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RN,(int)rr);
        ftm_pwm_duty(MOTOR_FTM,MOTOR_CH_RP,0);
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

void play(uint8 cmd)
{
    uint8 sum = 0xAA, i;
    uint8 ch[7][4] = { {0x13, 0x01, 0x1E, 0x00},//vol+++
                        {0x07, 0x02, 0x00, 0x01},//start
                        {0x07, 0x02, 0x00, 0x02},//ob
                        {0x07, 0x02, 0x00, 0x03},//br
                        {0x07, 0x02, 0x00, 0x04},//rh
                        {0x07, 0x02, 0x00, 0x05},//ch
                        {0x07, 0x02, 0x00, 0x06}};//pd
    for(i = 0; i < 4; i++)
        sum += ch[cmd][i];
    uart_putchar(DEBUG_UART, 0xAA);
    if(cmd == 0)
        uart_putbuff(DEBUG_UART, ch[cmd], 3);
    else
        uart_putbuff(DEBUG_UART, ch[cmd], 4);
    uart_putchar(DEBUG_UART, sum);
}

void test(void)
{

}