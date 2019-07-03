/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * All rights reserved.
 *中南大学黄竞辉
 *
 * @file       		BMX055
 * @author     		Alex
 * @version    		v1.0
 * @Software 		IAR 8.1
 * @date       		2017-11-9
 ********************************************************************************************************************/

#include "BMX055.h"

BMX055Data_t Q_raw;

bool BMX055_init(void)
{
    /************************************/
    /*加速度配置*/
    /************************************/
    uint8 ErrCount = 0;
    while(simiic_read_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_ID, IIC) != 0xFA)   /////确认芯片ID
    {
        ErrCount++;
        if(ErrCount > 5)
            return false;
    }
    simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMURANGE, 0x03);//      3-2G  5-4G  8-8G
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMUBW, 0x0F);     ///1000HZ-0x0F     500Hz-0x0E
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_ACC_ADR, BMX055_ACC_PMULPM, 0x00);   ///Normal MODE
    systick_delay_ms(10);

    /************************************/
    /*陀螺仪配置*/
    /************************************/
    ErrCount = 0;
    while(simiic_read_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_ID, IIC) != 0x0F)   /////确认芯片ID
    {
        ErrCount++;
        if(ErrCount > 5)
            return false;
    }
    simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RANGE, 0x01);   ///+-1000
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_BW, 0x02);     ///1000HZ
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_LPM, 0x00);   ///Normal MODE
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_GYRO_ADR, BMX055_GYRO_RATEHBW, 0x08);   ///高通滤波 ，可以不要
    systick_delay_ms(10);

    /************************************/
    /*磁力计配置*/
    /************************************/
    ErrCount = 0;
    simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_POM, 0x81);
    systick_delay_ms(10);
    while(simiic_read_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_ID, IIC) != 0x32)   /////确认芯片ID
    {
        ErrCount++;
        if(ErrCount > 5)
            return false;
    }
    simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_DATARATE, 0x38);   ///输出速率30HZ
    systick_delay_ms(10);
    simiic_write_reg(IIC_BMX055_MAG_ADR, BMX055_MAG_INTEN, 0x00);      ///不使能中断
    systick_delay_ms(10);
    return true;
}

void BMX055_DataRead(BMX055Data_t *Q, uint8 readMAG)
{
    uint8 datatemp[6] = {0};
    int16 MagX,MagY,MagZ;
    simiic_read_buf(IIC_BMX055_GYRO_ADR, BMX055_GYRO_XDATALSB, IIC, datatemp, 6);
    Q->GyroX = (float)((int16)((datatemp[1] << 8) | datatemp[0]));
    Q->GyroY = (float)((int16)((datatemp[3] << 8) | datatemp[2]));
    Q->GyroZ = (float)((int16)((datatemp[5] << 8) | datatemp[4]));

    simiic_read_buf(IIC_BMX055_ACC_ADR, BMX055_ACC_XDATALSB, IIC, datatemp, 6);
    Q->AccX = (float)((int16)((datatemp[1] << 8) | datatemp[0]) >> 4);
    Q->AccY = (float)((int16)((datatemp[3] << 8) | datatemp[2]) >> 4);
    Q->AccZ = (float)((int16)((datatemp[5] << 8) | datatemp[4]) >> 4);

    ///--------磁力计数据读取----------
    if(readMAG)
    {
        simiic_read_buf(IIC_BMX055_MAG_ADR, BMX055_MAG_XDATALSB, IIC, datatemp, 6);
        MagX = (int16)((datatemp[1] << 8) | datatemp[0]) >> 3;
        MagY = (int16)((datatemp[3] << 8) | datatemp[2]) >> 3;
        MagZ = (int16)((datatemp[5] << 8) | datatemp[4]) >> 3;
        Q->Mag = (uint16)((myabs(MagX) + myabs(MagY) + myabs(MagZ))/3);
    }
}

void GyroOffset_init(void)      /////////陀螺仪零飘初始化
{
    uint16 i;
    BMX055Data_t GyroOffset;
    gpio_set(BUZZER_PIN, 1);
    systick_delay_ms(100);
    gpio_set(BUZZER_PIN, 0);
    systick_delay_ms(900);
    float GX=0, GY=0, GZ=0;
    for(i=0; i<1000; i++)
    {
        BMX055_DataRead(&GyroOffset, 0);
        GX += GyroOffset.GyroX;
        GY += GyroOffset.GyroY;
        GZ += GyroOffset.GyroZ;
        systick_delay_ms(2);
    }
    GX /= 1000;
    GY /= 1000;
    GZ /= 1000;
    //SAVE()-------------
    gpio_set(BUZZER_PIN, 1);
    systick_delay_ms(100);
    gpio_set(BUZZER_PIN, 0);
}