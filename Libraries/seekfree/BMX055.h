﻿#ifndef __BMX055_H_
#define __BMX055_H_

#include "headfile.h"

#define IIC_BMX055_ACC_ADR    0x18
#define IIC_BMX055_GYRO_ADR   0x68
#define IIC_BMX055_MAG_ADR    0x10

#define BMX055_ACC_XDATALSB   0x02
#define BMX055_ACC_ID         0x00
#define BMX055_ACC_PMURANGE   0x0F
#define BMX055_ACC_PMUBW      0x10
#define BMX055_ACC_PMULPM     0x11


#define BMX055_GYRO_XDATALSB  0x02
#define BMX055_GYRO_ID        0x00
#define BMX055_GYRO_RANGE     0x0F
#define BMX055_GYRO_BW        0x10
#define BMX055_GYRO_LPM       0x11
#define BMX055_GYRO_RATEHBW   0x13

#define BMX055_MAG_XDATALSB   0x42
#define BMX055_MAG_ID         0x40
#define BMX055_MAG_POM        0x4B
#define BMX055_MAG_DATARATE   0x4C
#define BMX055_MAG_INTEN      0x4E

typedef struct
{
  float GyroX;
  float GyroY;
  float GyroZ;
  float AccX;
  float AccY;
  float AccZ;
  uint16 Mag;
}BMX055Data_t;

extern BMX055Data_t Q_raw;

bool BMX055_init(void);
void BMX055_DataRead(BMX055Data_t *Q, uint8 readMAG);
void GyroOffset_init(void);
#endif