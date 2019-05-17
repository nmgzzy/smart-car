//图像处理
//基于逐飞k60库
#ifndef _IMG_PROCESS_H
#define _IMG_PROCESS_H

#include "headfile.h"


//u8 two_peak_threshold(void);   //双峰法计算阈值  //返回阈值 返回0失败
u8 otsu_threshold(u8 *image);   //大津法计算阈值
void image_binaryzation(u8 *image_bin);  //图像二值化
void show_histogram(void);
void MedianBlur_bin(u8* img_bin_src, u8* img_bin_dst);

#endif