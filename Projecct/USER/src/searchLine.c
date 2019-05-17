#include "searchLine.h"

static void image_clear(void)
{
    uint8 i;
    for (i = 0; i < ROW; i++)
    {
        left_border[i] = 0;             //左边界
        right_border[i] = COL - 1;      //右边界
        road_center[i] = COL / 2;       //中线路径
        road_width[i] = 0;              //赛道宽度
        is_left_fix[i] = 0;             //补线标志
        is_right_fix[i] = 0;            //补线标志
    }
}

static uint8 *pixPoint;
static uint8 Ysite = 0, Xsite = 0;
static const uint8 imgMid = COL / 2;
static uint8 DrawLinesFirst(void)
{
    uint8 BottomBorderLeft = 0, BottomBorderRight = COL-1;
    pixPoint = img_bin_pro[ROW-1];
    if (*(pixPoint + imgMid) == 0)                           //如果底边图像中点为黑
    {
        for (Xsite = 0; Xsite < imgMid-1; Xsite++)
        {
            if (*(pixPoint + imgMid - Xsite) != 0 && *(pixPoint + imgMid - Xsite -1) != 0)
                break;
            if (*(pixPoint + imgMid + Xsite) != 0 && *(pixPoint + imgMid + Xsite + 1) != 0)
                break;
        }
        if (Xsite >(imgMid - 30))return 0;                  //中间无白点返回
        if (*(pixPoint + imgMid - Xsite) != 0)              //如果白点在左边
        {
            BottomBorderRight = imgMid - Xsite + 1;         //记录右边界
            for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)//寻找左边界
            {
                if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite - 1) == 0)//连续找到两个黑点
                {
                    BottomBorderLeft = Xsite;
                    break;
                }
                else if (Xsite == 1)
                {
                    BottomBorderLeft = 0;
                    break;
                }
            }
        }
        else if (*(pixPoint + imgMid + Xsite) != 0)         //如果白点在右边
        {
            BottomBorderLeft = imgMid + Xsite - 1;
            for (Xsite = BottomBorderLeft; Xsite < COL-1; Xsite++)
            {
                if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite + 1) == 0)
                {
                    BottomBorderLeft = Xsite;
                    break;
                }
                else if (Xsite == COL - 2)
                {
                    BottomBorderLeft = COL - 1;
                    break;
                }
            }
        }
    }
    else                           //如果底边图像中点为白
    {
        for (Xsite = imgMid; Xsite < COL - 1; Xsite++)           //向右搜线
        {
            if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite + 1) == 0)
            {
                BottomBorderRight = Xsite;
                break;
            }
            else if (Xsite == COL - 2)
            {
                BottomBorderRight = COL - 1;
                break;
            }
        }
        for (Xsite = imgMid; Xsite > 0; Xsite--)
        {
            if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite - 1) == 0)
            {
                BottomBorderLeft = Xsite;
                break;
            }
            else if (Xsite == 1)
            {
                BottomBorderLeft = 0;
                break;
            }
        }
    }

    left_border[ROW-1] = BottomBorderLeft;
    right_border[ROW - 1] = BottomBorderRight;
    road_center[ROW - 1] = (BottomBorderLeft + BottomBorderRight) / 2;                       ////确定最底边
    road_width[ROW - 1] = BottomBorderRight - BottomBorderLeft;   /////存储宽度信息
    //IsLeftFind[ROW - 1] = 1;
    //IsRightFind[ROW - 1] = 1;
    for (Ysite = ROW-2; Ysite >= ROW - 5; Ysite--)                         ////由中间向两边确定底边五行
    {
        pixPoint = img_bin_pro[Ysite];
        for (Xsite = road_center[Ysite + 1]; Xsite < COL - 1; Xsite++)        //向右搜
        {
            if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite + 1) == 0)
            {
                right_border[Ysite] = Xsite;
                break;
            }
            else if (Xsite == COL - 2)
            {
                right_border[Ysite] = COL - 1;
                break;
            }
        }
        for (Xsite = road_center[Ysite + 1]; Xsite > 0; Xsite--)            //向左搜
        {
            if (*(pixPoint + Xsite) == 0 && *(pixPoint + Xsite - 1) == 0)
            {
                left_border[Ysite] = Xsite;
                break;
            }
            else if (Xsite == 1)
            {
                left_border[Ysite] = 0;
                break;
            }
        }
        //ImageDeal[Ysite].IsLeftFind = 'T';
        //ImageDeal[Ysite].IsRightFind = 'T';
        road_center[Ysite] = (right_border[Ysite] + left_border[Ysite]) / 2;   //存储中点
        road_width[Ysite] = right_border[Ysite] - left_border[Ysite];         //存储宽度
        if (road_width[Ysite] <= 20 && Ysite <= ROW-4)// && SystemSettings.StartCheck == 'T'
        {
            uint8 StartFlagCount = 0;
            for (Xsite = 1; Xsite <= COL-2; Xsite++)
            {
                if (*(pixPoint + Xsite) != *(pixPoint + Xsite + 1))
                    StartFlagCount++;
            }
            if (StartFlagCount >= 10)   ///////保证底边有超过10个跳变点，则认为进入了起跑线范围，开始执行停车
            {
                //RoadType = Start;//起跑线标志
                //return 'S';
                //printf("finish\n");
            }
        }
    }
    return 1;
}  /////最基本的要求，最近的五行首先不会受到干扰，这需要在安装的时候调整摄像头的视角


void show_search_line(void)
{
    image_clear();
    DrawLinesFirst();
    for (int i = 0; i < ROW; i++)
    {
        if (is_left_fix[i] == 1)
        {
            img_bin_pro[i][left_border[i]] = ORANGE;
            if(left_border[i] > 0)
                img_bin_pro[i][left_border[i] - 1] = ORANGE;
        }
        else
        {
            img_bin_pro[i][left_border[i]] = PINK;
            if(left_border[i] > 0)
            img_bin_pro[i][left_border[i] - 1] = PINK;
        }
        if (is_right_fix[i] == 1)
        {
            img_bin_pro[i][right_border[i]] = ORANGE;
            if(right_border[i] < COL - 1)
                img_bin_pro[i][right_border[i] + 1] = ORANGE;
        }
        else
        {
            img_bin_pro[i][right_border[i]] = BLUE;
            if(right_border[i] < COL - 1)
                img_bin_pro[i][right_border[i] + 1] = BLUE;
        }
        img_bin_pro[i][road_center[i] - 1] = RED;
        img_bin_pro[i][road_center[i]]     = RED;
        img_bin_pro[i][road_center[i] + 1] = RED;
    }
}
