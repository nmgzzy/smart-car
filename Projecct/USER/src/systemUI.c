//此文件包含OLED调试的相关函数
//注意不要在中断函数中使用OLED显示
#include "systemUI.h"
#define PAGE_MAX 13
#define WORDS_MAX 15
#define DEFAULT_PAR 0

//page0 选择参数
//page1 主菜单
//page2 设置模式
//page3 设置参数
//page4 设置参数2

//添加参数：1.print_menu打印列表 2.adj_parameter增加判断调整
//          3.displayUI-按下增加参数选择 4.other.c增加flash存取

//private
static uint16 voltage = 0;
static uint8 flag_updown=0;
static int8 flag_plus_minus=0;
static double show_parameter;


//更新屏幕
static void print_menu(uint8 page, uint8 choice)
{
    uint8 menu[PAGE_MAX][8][WORDS_MAX] =
    {
        //page0-select param
        {
            "y0",
            "y1",
            "y2",
            "y3",
            "n0",
            "n1",
            "n2",
            "n3"
        },
        //page1-main
        {
            "GO!!!",
            "set",
            "show",
            "Param1",
            "Param2",
            "Param3",
            "Param4",
            "Param5"
        },
        //page2-set
        {
            "6s",
            "17s",
            "50s",
            "Debug mode",
            "flag_std",
            "flag_spd",
            "flag_dir",
            "PWM test"
        },
        //page3-show
        {
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            ""
        },
        //page4-param1
        {
            "setTime",
            "tarAng2",
            "tarAng 3",
            "tarSpd2",
            "tarSpd 3",
            "spd_acc",
            "k_adc",
            "BalanceMode"
        },
        //page5-param2
        {
            "angKp2",
            "angKd2",
            "angKp 3",
            "angKd 3",
            "spdKp2",
            "spdKi2",
            "spdKp 3",
            "spdKi 3",
        },
        //page6
        {
            "dirKp2",
            "dirKd2",
            "dirKp 3",
            "dirKd 3",
            "yawKp2",
            "yawKd2",
            "yawKp 3",
            "yawKd 3"
        },
        //page7
        {
            "spdKd2",
            "spdKd 3",
            "k_hv2",
            "k_hv 3",
            "k_md2",
            "k_md 3",
            "",
            ""
        },
        //page8
        {
            "bar_t2",
            "bar_k2",
            "bar_dly_o2",
            "bar_dir",
            "ke2",
            "ke3",
            "cl_num",
            "cl_time_in"
        },
        //page9
        {
            "Param6",
            "Param7",
            "Param8",
            "",
            "",
            "",
            "",
            ""
        },
        //page10
        {
            "k_c1",
            "c_o1",
            "k_c2",
            "c_o2",
            "k_c3",
            "c_o3",
            "k_c4",
            "c_o4"
        },
        //page11
        {
            "k_c5",
            "c_o5",
            "",
            "",
            "",
            "",
            "bar_dly2",
            "bar_dly 3"
        },
        //page12
        {
            "bar_t 3",
            "bar_k 3",
            "bar_dly_o 3",
            "",
            "",
            "Par1",
            "Par2",
            "Par3"
        }
    };
    static uint8 last_page = 0;
    uint8 i;
    float error2, error3;
    if(last_page != page)
    {
        OLED_Fill(0);
        OLED_P6x8Str(0,choice-1,"*");
    }
    switch(page)
    {
        case 0:         //首页
            OLED_P8x16Str(20,0,"read data n?");
            for(i=0; i<8; i++)
                OLED_P6x8Str(7,i,menu[page][i]);
            break;
        case 1:         //主菜单
            for(i=0; i<8; i++)
                OLED_P6x8Str(10,i,menu[page][i]);
            OLED_P8x16Str(100,0,"NNU");
            OLED_P8x16Str(100,2,"ZZY");
            voltage = adc_once(VBAT_PIN,ADC_12bit);
            voltage = (uint16)(limit_ab(voltage - 2926, 0 , 448)/4.48f);
            OLED_Print_uint16(80, 7, voltage, 0, 0);
            break;
        case 2:         //set
            for(i=0; i<8; i++)
                OLED_P6x8Str(7,i,menu[page][i]);
            OLED_Print_uint16(80, 4, flag.En_std, 0, 0);
            OLED_Print_uint16(80, 5, flag.En_spd, 0, 0);
            OLED_Print_uint16(80, 6, flag.En_dir, 0, 0);
            break;
        case 3:         //show
            //error2 = ErrorCalculate(2);
            //error3 = ErrorCalculate(3);
            ad_data_now[LH] = adc_once(ADC_LH,ADC_12bit);
            ad_data_now[LV] = adc_once(ADC_LV,ADC_12bit);
            ad_data_now[LX] = adc_once(ADC_LX,ADC_12bit);
            ad_data_now[MD] = adc_once(ADC_MD,ADC_12bit);
            ad_data_now[RX] = adc_once(ADC_RX,ADC_12bit);
            ad_data_now[RV] = adc_once(ADC_RV,ADC_12bit);
            ad_data_now[RH] = adc_once(ADC_RH,ADC_12bit);
            OLED_Print_uint16(10, 0, ad_data_now[LH], 1, 1);
            OLED_Print_uint16(68, 0, ad_data_now[RH], 1, 1);
            OLED_Print_uint16( 0, 2, ad_data_now[LV], 1, 1);
            OLED_Print_uint16(78, 2, ad_data_now[RV], 1, 1);
            OLED_Print_uint16( 5, 4, ad_data_now[LX], 1, 1);
            OLED_Print_uint16(73, 4, ad_data_now[RX], 1, 1);
            OLED_Print_uint16(38, 6, ad_data_now[MD], 1, 1);
            /*error2 = k_ke2*(k_hv2*(ad_data_now[LH] - ad_data_now[RH])
                 + (1-k_hv2)*(ad_data_now[LV] - ad_data_now[RV]))
                / (ad_data_now[LH] + ad_data_now[RH] + k_md2*ad_data_now[MD] + 1);
            error3 = k_ke3*(k_hv3*(ad_data_now[LH] - ad_data_now[RH])
                 + (1-k_hv3)*(ad_data_now[LV] - ad_data_now[RV]))
                / (ad_data_now[LH] + ad_data_now[RH] + 1);*/


            //OLED_Print_float(5, 6, error2, 3, 1);
            //OLED_Print_float(70, 6, error3, 3, 1);
            //OLED_Print_uint16(0, 4, ad_data_now[LH]+ad_data_now[RH], 1, 0);
            //OLED_Print_uint16(0, 5, ad_data_now[LV]+ad_data_now[RV], 1, 0);
            //OLED_Print_int16(64, 4, ad_data_now[LH]-ad_data_now[RH], 1, 0);
            //OLED_Print_int16(64, 5, ad_data_now[LV]-ad_data_now[RV], 1, 0);
            /*uint16 distance1, distance2;
            uint8 buf[2];
            simiic_read_buf2(0xA0>>1, 0x00, IIC, buf, 2);//三轮
            distance1 = (buf[0]<<8) | buf[1];
            simiic_read_buf2(0xB0>>1, 0x00, IIC, buf, 2);//两轮
            distance2 = (buf[0]<<8) | buf[1];
            OLED_Print_uint16(0, 7, distance1, 1, 0);
            OLED_Print_uint16(64, 7, distance2, 1, 0);*/
            systick_delay_ms(10);

            break;  //show
        default:         //parameter
            for(i=0; i<8; i++)
                OLED_P6x8Str(7,i,menu[page][i]);
            OLED_Print_float(64, 0, show_parameter, 4, 3);
            break;
    }
    last_page = page;
}


//读取五向开关
uint8 readKey(void)
{
    uint8 i, key = 0;
    static uint8 j = 0, k = 0;
    static uint8 keysta[5]={1,1,1,1,1},keybk[5]={1,1,1,1,1};
    keysta[0] = gpio_get(KEY_PIN_U);
    keysta[1] = gpio_get(KEY_PIN_L);
    keysta[2] = gpio_get(KEY_PIN_D);
    keysta[3] = gpio_get(KEY_PIN_R);
    keysta[4] = gpio_get(KEY_PIN_M);

    for(i=0; i<5; i++)
    {
        if(keysta[i] != keybk[i])
        {
            if(keybk[i] == 0)
            {
                key = i+1;
            }
            keybk[i]=keysta[i];
            k = 0;
        }
        else if(keysta[i] == 0 && keybk[i] == 0 && (i==0||i==2) && flag_updown == 1)
        {
            j++;
            if(j > (k?20:150))
            {
                j = 0;
                key = i+1;
                k++;
                if(k>250) k = 1;
            }
        }
    }
    return key;
}

static void adj_f(float* k, float step)
{
    show_parameter = *k;
    if(flag_plus_minus != 0)
    {
        *k += flag_plus_minus*step;
        flag_plus_minus = 0;
    }
}
static void adj_u8(uint8* k, uint8 step)
{
    show_parameter = *k;
    if(flag_plus_minus == 1)
    {
        flag_plus_minus = 0;
        *k += step;
    }
    else if(flag_plus_minus == -1 && *k >= step)
    {
        flag_plus_minus = 0;
        *k -= step;
    }
}
static void adj_i8(int8* k, uint8 step)
{
    show_parameter = *k;
    if(flag_plus_minus != 0)
    {
        *k += flag_plus_minus*step;
        flag_plus_minus = 0;
    }
}
static void adj_u16(uint16* k, uint8 step)
{
    show_parameter = *k;
    if(flag_plus_minus == 1)
    {
        flag_plus_minus = 0;
        *k += step;
    }
    else if(flag_plus_minus == -1 && *k >= step)
    {
        flag_plus_minus = 0;
        *k -= step;
    }
}
static void adj_i16(int16* k, uint8 step)
{
    show_parameter = *k;
    if(flag_plus_minus != 0)
    {
        *k += flag_plus_minus*step;
        flag_plus_minus = 0;
    }
}

//调整参数
static void adj_parameter(uint8 flag_parameters)
{
    ////////////////////////////
    if     (flag_parameters == 1)     adj_u16(&set_time, 1);
    else if(flag_parameters == 2)     adj_f(&target_angle[0], 0.5);
    else if(flag_parameters == 3)     adj_f(&target_angle[1], 0.5);
    else if(flag_parameters == 4)     adj_i16(&target_speed_max[0], 5);
    else if(flag_parameters == 5)     adj_i16(&target_speed_max[1], 5);
    else if(flag_parameters == 6)     adj_u16(&spd_acc, 1);
    else if(flag_parameters == 7)     adj_f(&k_adc, 0.01);
    else if(flag_parameters == 8)     adj_u8(&Balance_mode, 1);
    ////////////////////////////
    else if(flag_parameters == 9)     adj_f(&pid_angle[0].p, 1);
    else if(flag_parameters == 10)    adj_f(&pid_angle[0].d, 0.1);
    else if(flag_parameters == 11)    adj_f(&pid_angle[1].p, 1);
    else if(flag_parameters == 12)    adj_f(&pid_angle[1].d, 0.1);
    else if(flag_parameters == 13)    adj_f(&pid_speed[0].p, 0.2);
    else if(flag_parameters == 14)    adj_f(&pid_speed[0].i, 0.01);
    else if(flag_parameters == 15)    adj_f(&pid_speed[1].p, 1);
    else if(flag_parameters == 16)    adj_f(&pid_speed[1].i, 0.01);
    ////////////////////////////
    else if(flag_parameters == 17)    adj_f(&pid_dir_pset[0], 0.5);
    else if(flag_parameters == 18)    adj_f(&pid_dir[0].d, 0.5);
    else if(flag_parameters == 19)    adj_f(&pid_dir_pset[1], 0.5);
    else if(flag_parameters == 20)    adj_f(&pid_dir[1].d, 0.5);
    else if(flag_parameters == 21)    adj_f(&pid_yaw[0].p, 0.1);
    else if(flag_parameters == 22)    adj_f(&pid_yaw[0].d, 0.1);
    else if(flag_parameters == 23)    adj_f(&pid_yaw[1].p, 0.1);
    else if(flag_parameters == 24)    adj_f(&pid_yaw[1].d, 0.1);
    ////////////////////////////
    else if(flag_parameters == 25)    adj_f(&pid_speed[0].d, 0.01);
    else if(flag_parameters == 26)    adj_f(&pid_speed[1].d, 0.5);
    else if(flag_parameters == 27)    adj_f(&k_hv[0], 0.5);
    else if(flag_parameters == 28)    adj_f(&k_hv[1], 0.5);
    else if(flag_parameters == 29)    adj_f(&k_md[0], 0.05);
    else if(flag_parameters == 30)    adj_f(&k_md[1], 0.05);
    else if(flag_parameters == 31)    ;//adj_f(&k_ei[0], 0.1);
    else if(flag_parameters == 32)    ;//adj_f(&k_ei[1], 0.1);
    ////////////////////////////
    else if(flag_parameters == 33)    adj_u16(&barrier_turn_t[0], 5);
    else if(flag_parameters == 34)    adj_u16(&barrier_turn_k[0], 5);
    else if(flag_parameters == 35)    adj_u16(&barrier_delay_out[0], 10);
    else if(flag_parameters == 36)    adj_i8(&barrier_turn_dir, 1);
    else if(flag_parameters == 37)    adj_f(&k_ke[0], 0.2);
    else if(flag_parameters == 38)    adj_f(&k_ke[1], 0.2);
    else if(flag_parameters == 39)    adj_u8(&cl_num, 1);
    else if(flag_parameters == 40)    adj_u16(&cl_time, 5);
    ////////////////////////////
    else if(flag_parameters == 41)    adj_f(&k_circle[0], 0.2);
    else if(flag_parameters == 42)    adj_i8(&circle_offset[0], 1);
    else if(flag_parameters == 43)    adj_f(&k_circle[1], 0.2);
    else if(flag_parameters == 44)    adj_i8(&circle_offset[1], 1);
    else if(flag_parameters == 45)    adj_f(&k_circle[2], 0.2);
    else if(flag_parameters == 46)    adj_i8(&circle_offset[2], 1);
    else if(flag_parameters == 47)    adj_f(&k_circle[3], 0.2);
    else if(flag_parameters == 48)    adj_i8(&circle_offset[3], 1);
    ////////////////////////////
    else if(flag_parameters == 49)    adj_f(&k_circle[4], 0.2);
    else if(flag_parameters == 50)    adj_i8(&circle_offset[4], 1);
    else if(flag_parameters == 51)    ;//adj_f(&pid_dire_pset[0], 0.5);
    else if(flag_parameters == 52)    ;//adj_f(&pid_dire_pset[1], 0.5);
    else if(flag_parameters == 53)    ;//adj_f(&pid_dire[0].d, 0.5);
    else if(flag_parameters == 54)    ;//adj_f(&pid_dire[1].d, 0.5);
    else if(flag_parameters == 55)    adj_u16(&barrier_delay[0], 5);
    else if(flag_parameters == 56)    adj_u16(&barrier_delay[1], 5);
    ////////////////////////////
    else if(flag_parameters == 57)    adj_u16(&barrier_turn_t[1], 5);
    else if(flag_parameters == 58)    adj_u16(&barrier_turn_k[1], 5);
    else if(flag_parameters == 59)    adj_u16(&barrier_delay_out[1], 10);

    else if(flag_parameters == 60)    ;//adj_u16(&barrier_turn_t[1], 5);
    else if(flag_parameters == 61)    ;//adj_u16(&barrier_turn_k[1], 5);
    else if(flag_parameters == 62)    adj_f(&testPar[0], 0.1f);
    else if(flag_parameters == 63)    adj_f(&testPar[1], 1);
    else if(flag_parameters == 64)    adj_f(&testPar[2], 10);
    ////////////////////////////
}

//oled调试界面
void displayUI(void)
{
    uint8 key=0, page = 0, choice = 0, n=100;
    uint8 flag_parameters=0;    //1-target_angle 2-target_speed 3-set_time 4-K_hv
    //uint8 i;
    OLED_Fill(0X00);
    gpio_init(KEY_PIN_U, GPI, 0); //down
    gpio_init(KEY_PIN_D, GPI, 0); //right
    gpio_init(KEY_PIN_L, GPI, 0); //left
    gpio_init(KEY_PIN_R, GPI, 0); //up
    gpio_init(KEY_PIN_M, GPI, 0); //mid
    while(1)
    {
        if(flag.mode != 0) break;
        key = readKey();

        //------上键--------------------------------------
        if(key == 1)
        {
            if(flag_updown == 0)
            {
                if(choice > 1)
                {
                    OLED_fill(0,6,choice-1,0);
                    choice--;
                }
                else if(choice == 1)
                {
                    if(page == 5 || page == 6 || page == 7 || page == 8 || page == 11 || page == 12 )
                    {
                        page--;
                        choice = 8;
                    }
                    else if(page == 10)
                    {
                        page = 8;
                        choice = 8;
                    }
                    else if(page == 9)
                    {
                        page = 1;
                        choice = 8;
                    }
                }
                OLED_P6x8Str(0,choice-1,"*");
            }
            else if(flag_updown == 1)
            {
                flag_plus_minus = 1;
            }
        }

        //------下键---------------------------------------
        else if(key == 3)
        {
            if(flag_updown == 0)
            {
                if(choice < 8)
                {
                    OLED_fill(0,6,choice-1,0);
                    choice++;
                }
                else if(choice == 8)
                {
                    if(page == 4 || page == 5 || page == 6 || page == 7 || page == 10 || page == 11 )
                    {
                        page++;
                        choice = 1;
                    }
                    else if(page == 8)
                    {
                        page = 10;
                        choice = 1;
                    }
                    else if(page == 1)
                    {
                        page = 9;
                        choice = 1;
                    }
                }
                OLED_P6x8Str(0,choice-1,"*");
            }
            else if(flag_updown == 1)
            {
                flag_plus_minus = -1;
            }
        }

        //-----左键--------------------------------------
        else if(key == 2)
        {
            if(flag_updown)
                flag_updown = 0;
            else
            {
                if(page >= 2 && page <= 8)
                {
                    page = 1;
                }
                else if(page > 9)
                {
                    page = 9;
                }
            }
        }

        //-----右键--------------------------------------
        else if(key == 4)
        {
            if(page == 1)
            {
                if(choice != 1)
                    page = choice;
            }
            else if(page == 9)
            {
                page = choice+9;
            }
        }

        //-----按下--------------------------------------
        else if(key == 5)
        {
            if(page == 0)//read param
            {
                switch(choice)
                {
                    case 0:data_read(DEFAULT_PAR);n=DEFAULT_PAR;break;
                    case 1:data_read(0);n=0;break;
                    case 2:data_read(1);n=1;break;
                    case 3:data_read(2);n=2;break;
                    case 4:data_read(3);n=3;break;
                    case 5:data_read(n);n=0;break;
                    case 6:data_read(n);n=1;break;
                    case 7:data_read(n);n=2;break;
                    case 8:data_read(n);n=3;break;
                }
                page = 1;
                choice = 1;
            }
            else if(page == 1 && choice == 1)//main
            {
                flag.mode = MODE_START;
            }
            else if(page == 2)//set
            {
                switch(choice)
                {
                    case 1:set_time = 6;break;
                    case 2:set_time = 17;break;
                    case 3:set_time = 50;break;
                    case 4:flag.mode = MODE_DEBUG; break;
                    case 5:flag.En_std = flag.En_std?0:1;break;
                    case 6:flag.En_spd = flag.En_spd?0:1;break;
                    case 7:flag.En_dir = flag.En_dir?0:1;break;
                    case 8:flag.mode = MODE_PWM_TEST; break;
                }
            }
            else if(page == 4)//param1
            {
                flag_updown = 1;
                flag_parameters = choice;
            }
            else if(page == 5)//param2
            {
                flag_updown = 1;
                flag_parameters = choice+8;
            }
            else if(page == 6)//param3
            {
                flag_updown = 1;
                flag_parameters = choice+16;
            }
            else if(page == 7)//param4
            {
                flag_updown = 1;
                flag_parameters = choice+24;
            }
            else if(page == 8)//param5
            {
                flag_updown = 1;
                flag_parameters = choice+32;
            }
            else if(page == 10)//param6
            {
                flag_updown = 1;
                flag_parameters = choice+40;
            }
            else if(page == 11)//param7
            {
                flag_updown = 1;
                flag_parameters = choice+48;
            }
            else if(page == 12)//param8
            {
                flag_updown = 1;
                flag_parameters = choice+56;
            }
        }
        print_menu(page, choice);
        adj_parameter(flag_parameters);
    }
    OLED_Fill(0X00);
    data_save(n);
}

void displayDebug(void)
{
    int8 buff[20];
    sprintf(buff, "Pch:%3.1f \0", CarAttitude.Pitch);
    OLED_P6x8Str(0, 0, (uint8*)buff);
    sprintf(buff, "Prt:%3.1f \0", CarAttitudeRate.Pitch);
    OLED_P6x8Str(0, 1, (uint8*)buff);
    sprintf(buff, "Yrt:%3.1f \0", CarAttitudeRate.Yaw);
    OLED_P6x8Str(0, 2, (uint8*)buff);
    //OLED_Print_float(0 , 1, CarAttitude.Yaw,3,1);
    ///OLED_Print_float(0 , 2, CarAttitudeRate.Pitch,3,1);
    //OLED_Print_float(0 , 3, CarAttitudeRate.Yaw,3,1);
    sprintf(buff, "spd:%3d \0", (int16)car_speed_now);
    OLED_P6x8Str(0, 7, (uint8*)buff);

    OLED_Print_float(0 , 4, ftestVal[0],3,2);
    OLED_Print_float(0 , 5, itestVal[1],3,1);
    OLED_Print_float(0 , 6, ftestVal[2],3,1);

}

void printLog(int8 message[20])
{
    uint8 i;
    static uint8 log[8][20] = {0};
    for(i=0; i<7; i++)
    {
        strcpy((char*)log[i],(char*)log[i+1]);
    }
    strcpy((char*)log[7],(char*)message);
    OLED_P6x8Str1(0,0,log[0], 20);
    OLED_P6x8Str1(0,1,log[1], 20);
    OLED_P6x8Str1(0,2,log[2], 20);
    OLED_P6x8Str1(0,3,log[3], 20);
    OLED_P6x8Str1(0,4,log[4], 20);
    OLED_P6x8Str1(0,5,log[5], 20);
    OLED_P6x8Str1(0,6,log[6], 20);
    OLED_P6x8Str1(0,7,log[7], 20);
}
