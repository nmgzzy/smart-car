//图像处理
//基于逐飞k60库
#include "imgProcess.h"

u8 otsu_threshold(u8 *image)//大津法计算阈值
{
    #define GrayScale 128
    u16 i, j;
    int pixelCount[GrayScale]={0};
    float pixelPro[GrayScale]={0};
    int pixelSum = COL * ROW;
    u8 threshold = 0;
    u8* data = image;  //指向像素数据的指针

    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < pixelSum; i++)
    {
        pixelCount[(data[i]>>1)]++;  //将像素值作为计数数组的下标
    }

    //计算每个像素在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }

    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = thresholdMin>>1; i < thresholdMax>>1; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i<<1;
        }
    }
    return threshold;
}

void image_binaryzation(u8 *image_bin)  //图像二值化
{
    u16 i, j;
    u8* img_src;
	for(i=0;i<ROW;i++)
    {
        img_src = image[i];
        for(j=0;j<COL;j++)
        {
            if( *(img_src+j) >= imgThreshold + Threshold_add[i*3/ROW][j*3/COL])
                *(image_bin+i*COL+j) = 1;
            else
                *(image_bin+i*COL+j) = 0;
        }
    }
}

void show_histogram(void)
{
	u8 i, j, t;
	u16 histogram_data[51]={0};
	u8 histogram_img[40][51]={0};
    for (i = 0; i < ROW; ++i)
    {
    	for (j = 0; j < COL; ++j)
    	{
    		t=image[i][j] / 5;
    		if(t>50) t=50;
    		histogram_data[t]++;
    	}
    }
    for (j = 0; j < 51; ++j)
    {
    	histogram_data[j] /= 100;
        if (histogram_data[j]>40)histogram_data[j]=40;
        else if (histogram_data[j]<1)histogram_data[j]=1;
    	for (i = 0; i < histogram_data[j]; ++i)
    	{
            if(j==imgThreshold/5)
                histogram_img[39-i][j] = 2;
            else
                histogram_img[39-i][j] = 1;
    	}
    }
    lcd_showimage_bin(histogram_img, 88, 0, 51, 40);
}

void MedianBlur_bin(u8* img_bin_src, u8* img_bin_dst)//图像中值滤波
{
    uint8 i, j;
    uint8 sum;
    uint16 c[3] = {0};
    for (i = 0; i < ROW; ++i)
    {
        c[1] = i*COL;
        if(i>0)
        {
            c[0] = c[1] - COL;
            c[2] = c[1] + COL;
        }
    	for (j = 0; j < COL; ++j)
    	{
    		if(i==0 || j==0 || i==ROW-1 || j==COL-1)
            {
                *(img_bin_dst + c[1] + j) = *(img_bin_src + c[1] + j);
            }
            else
            {
                sum =
                    *(img_bin_src + c[0] + j-1)+
                    *(img_bin_src + c[0] + j)+
                    *(img_bin_src + c[0] + j+1)+
                    *(img_bin_src + c[1] + j-1)+
                    *(img_bin_src + c[1] + j)+
                    *(img_bin_src + c[1] + j+1)+
                    *(img_bin_src + c[2] + j-1)+
                    *(img_bin_src + c[2] + j)+
                    *(img_bin_src + c[2] + j+1);
                if(sum > 4)
                    *(img_bin_dst + c[1] + j) = 1;
                else
                    *(img_bin_dst + c[1] + j) = 0;
            }
    	}
    }
}