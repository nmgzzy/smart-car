/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		MK60_port_cfg
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN and MK60FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/



#ifndef _MK60_port_cfg_h
#define _MK60_port_cfg_h

#include "common.h"

//-----------------------------------  UART  -----------------------------------
#define DEBUG_UART  uart1
#define DEBUG_BAUD  115200

#define TEST_UART  uart0
#define TEST_BAUD  115200

//      模块通道        端口          可选范围                建议
#define UART0_RX_PIN    B16        //A1、A15、B16、D6          A1不要用（与Jtag冲突）
#define UART0_TX_PIN    B17        //A2、A14、B17、D7          A2不要用（与Jtag冲突）

#define UART1_RX_PIN    E1        //C3、E1
#define UART1_TX_PIN    E0        //C4、E0

#define UART2_RX_PIN    D2        //D2
#define UART2_TX_PIN    D3        //D3

#define UART3_RX_PIN    C16       //B10、C16、E5
#define UART3_TX_PIN    C17       //B11、C17、E4

#define UART4_RX_PIN    C14       //C14、E25
#define UART4_TX_PIN    C15       //C15、E24

#define UART5_RX_PIN    E9        //D8、E9
#define UART5_TX_PIN    E8        //D9、E8

//-----------------------------------  FTM  -----------------------------------

//      模块通道        端口      可选范围                建议
#define FTM0_CH0_PIN    C1        //C1、A3                A3不要用（与Jtag、SWD冲突）
#define FTM0_CH1_PIN    C2        //C2、A4
#define FTM0_CH2_PIN    C3        //C3、A5
#define FTM0_CH3_PIN    C4        //C4、A6
#define FTM0_CH4_PIN    D4        //D4、A7
#define FTM0_CH5_PIN    D5        //D5、A0                A0不要用（与Jtag、SWD冲突）
#define FTM0_CH6_PIN    D6        //D6、A1                A1不要用（与Jtag冲突）
#define FTM0_CH7_PIN    D7        //D7、A2                A2不要用（与Jtag冲突）


//      模块通道        端口      可选范围                建议
#define FTM1_CH0_PIN    A12       //A8、A12、B0
#define FTM1_CH1_PIN    A13       //A9、A13、B1

//      模块通道        端口      可选范围                建议
#define FTM2_CH0_PIN    B18       //A10、B18
#define FTM2_CH1_PIN    B19       //A11、B19

//      模块通道        端口      可选范围                建议
#define FTM3_CH0_PIN    E5        //D0
#define FTM3_CH1_PIN    E6        //D1
#define FTM3_CH2_PIN    E7        //D2
#define FTM3_CH3_PIN    E8        //D3
#define FTM3_CH4_PIN    E9        //C8
#define FTM3_CH5_PIN    E10       //C9
#define FTM3_CH6_PIN    C10       //E11
#define FTM3_CH7_PIN    E12       //C11


//正交解码模块通道      端口      可选范围                建议
#define FTM1_QDPHA_PIN  A12       //A8、A12、B0
#define FTM1_QDPHB_PIN  A13       //A9、A13、B1

#define FTM2_QDPHA_PIN  B18       //A10、B18
#define FTM2_QDPHB_PIN  B19       //A11、B19


//-----------------------------------  I2C  -----------------------------------

//      模块通道        端口      可选范围                建议
#define I2C0_SCL_PIN    B2        // B0、B2、D8
#define I2C0_SDA_PIN    B3        // B1、B3、D9

#define I2C1_SCL_PIN    C10       // E1、C10
#define I2C1_SDA_PIN    C11       // E0、C11

#define SEEKFREE_SCL    B21                           //定义SCL引脚
#define SEEKFREE_SDA    B22                           //定义SDA引脚

#define SEEKFREE_SCL2   C16                           //定义SCL引脚
#define SEEKFREE_SDA2   C17                           //定义SDA引脚


//-----------------------------------  SPI  -----------------------------------
//如果不使用片选接口的话且需要片选接口用于做其他的时候需要注释掉，就不会进行初始化对应的管脚
//      模块通道        端口      可选范围              建议

#define SPI0_SCK_PIN    D1       // A15、C5、D1        全部都是 ALT2
#define SPI0_SOUT_PIN   D2       // A16、C6、D2        全部都是 ALT2
#define SPI0_SIN_PIN    D3       // A17、C7、D3        全部都是 ALT2

#define SPI0_PCS0_PIN   A14       // A14、C4、D0、      全部都是 ALT2
#define SPI0_PCS1_PIN   D4        // C3、D4             全部都是 ALT2
#define SPI0_PCS2_PIN   C2        // C2、D5             全部都是 ALT2
#define SPI0_PCS3_PIN   C1        // C1、D6             全部都是 ALT2
#define SPI0_PCS4_PIN   C0        // C0、               全部都是 ALT2
#define SPI0_PCS5_PIN   B23       // B23                ALT3


#define SPI1_SCK_PIN    B11       // E2、B11、          全部都是 ALT2
#define SPI1_SOUT_PIN   B16       // E1、B16、          全部都是 ALT2
#define SPI1_SIN_PIN    B17       // E3、B17、          全部都是 ALT2

#define SPI1_PCS0_PIN   B10       // E4、B10、          全部都是 ALT2
#define SPI1_PCS1_PIN   E0        // E0、B9、           全部都是 ALT2
#define SPI1_PCS2_PIN   E5        // E5、               全部都是 ALT2
#define SPI1_PCS3_PIN   E6        // E6、               全部都是 ALT2


#define SPI2_SCK_PIN    B21       // B21、D12           全部都是 ALT2
#define SPI2_SOUT_PIN   B22       // B22、D13           全部都是 ALT2
#define SPI2_SIN_PIN    B23       // B23、D14           全部都是 ALT2
#define SPI2_PCS0_PIN   B20       // B20、D11           全部都是 ALT2
#define SPI2_PCS1_PIN   D15       // D15                全部都是 ALT2


//-----------------------------------  CAN  -----------------------------------
#define CAN0_TX_PIN     A12       //A12、B18            全部都是 ALT2
#define CAN0_RX_PIN     A13       //A13、B19            全部都是 ALT2

#define CAN1_TX_PIN     E24       //E24、C17            全部都是 ALT2
#define CAN1_RX_PIN     E25       //E25、C16            全部都是 ALT2


#define MOTOR_FTM ftm0
#define MOTOR_CH_LP ftm_ch2
#define MOTOR_CH_LN ftm_ch1
#define MOTOR_CH_RP ftm_ch0
#define MOTOR_CH_RN ftm_ch3

#define SERVO_FTM ftm3
#define SERVO_CH ftm_ch6

#define KEY_PIN_U   E5
#define KEY_PIN_L   E6
#define KEY_PIN_D   E7
#define KEY_PIN_M   E4
#define KEY_PIN_R   E3

//  U   M
//  L   R
//  D   gnd

#define BUZZER_PIN  E10
#define SWICH_PIN  E11

//??????????
#define ADC_LH   ADC1_SE12
#define ADC_LV   ADC1_SE13
#define ADC_LX   ADC1_SE11
#define ADC_MD   ADC1_SE10
#define ADC_RH   ADC0_SE12
#define ADC_RV   ADC0_SE9
#define ADC_RX   ADC0_SE13
#define OTHERAD  ADC0_SE8
//??????????

#define VBAT_PIN    ADC0_SE14

#define ENCODER_CLKL  B9
#define ENCODER_CSnL  B8
#define ENCODER_DATEL B10
#define ENCODER_CLKR  E24
#define ENCODER_CSnR  E25
#define ENCODER_DATER E26


//----OLED引脚----
#define  OLED_SCL_PIN	D1
#define  OLED_SDA_PIN	D2
#define  OLED_RST_PIN	D0
#define  OLED_DC_PIN	C19
#define  OLED_CS_PIN	C18



#endif


