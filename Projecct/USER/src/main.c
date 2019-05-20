#include "headfile.h"

void system_init(void);
void test(void);

vuint16 distance = 0;

int main(void)
{
    system_init();
    //uint16 distance;
    //uint8 buf[2];
    //uint8 a = 1, b = 10;
    //uint8 cnt = 0;
    flag.buzz = 1;
    //uint32 t = 0;

    while(1)
    {
        if(flag.mode != MODE_START)
        {
            if(flag.mode == MODE_DEBUG)
            {
                displayDebug();
            }
        }
        if(Balance_mode == 0)
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, 560);
        else
            ftm_pwm_duty(SERVO_FTM, SERVO_CH, 235);
        /*if(Balance_mode == 1)
        {
            simiic_read_buf2(0xA0>>1, 0x00, IIC, buf, 2);//三轮
            distance = (buf[0]<<8) | buf[1];
        }
        else if(Balance_mode == 0)
        {
            simiic_read_buf2(0xB0>>1, 0x00, IIC, buf, 2);//三轮
            distance = (buf[0]<<8) | buf[1];
        }
        if(distance > 400 && distance < 900
           && myfabs(image_error[0]) < 15 && time_count-t > 1500
           && time_count > a*500 && time_count < b*500
           && flag.circle == 0)
        {
            cnt++;
            if(distance < 700 && cnt >= 3)
            {
                flag.barrier = 1;
                t = time_count;
            }
        }
        else
            cnt = 0;*/
        if(!gpio_get(SWICH_PIN) && time_count>500 && flag.lost == 0)
        {
            flag.lost = 1;
            printLog("Remote stop");
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
    ftm_pwm_init(SERVO_FTM, SERVO_CH, 330, 450); //三轮230  两轮590
    systick_delay_ms(300);
    FLASH_Init();
    systick_delay_ms(5);
    OLED_Init();
    pidInit();
    gpio_init(BUZZER_PIN, GPO, 0);
    gpio_init(SWICH_PIN, GPI, 0);
    speed_encoder_init();
    IIC_init();
    IIC_init2();
    BMX055_init();
    systick_delay_ms(5);
    ADC_init();
    //communicate_init();
    systick_delay_ms(10);
    displayUI();
    pit_init_ms(pit0,2)
    set_irq_priority(PIT0_IRQn,1);
    enable_irq(PIT0_IRQn);
    EnableInterrupts;
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
    while(1)
    {
        uart_putstr(DEBUG_UART,"abcd567sffgxb000123456789\r\n");
        systick_delay_ms(20);
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