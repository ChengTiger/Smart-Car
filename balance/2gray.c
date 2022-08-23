#include "zf_common_headfile.h"


//w0为背景像素点占整幅图像的比例
// 
//u0为w0平均灰度
// 
//w1为前景像素点占整幅图像的比例
// 
//u1为w1平均灰度
// 
//u为整幅图像的平均灰度
// 
//类间方差公式 g = w1 * w2 * (u1 - u2) ^ 2
 
int otsuThreshold(int *image, int col, int row)
{
    #define GrayScale 256
    int width = col;
    int height = row;
    int pixelCount[GrayScale] = {0}; //每个灰度值所占像素个数
    float pixelPro[GrayScale] = {0};//每个灰度值所占总像素比例
    int i, j, pixelSum = width * height;   //总像素
    int threshold = 0;
    int* data = image;  //指向像素数据的指针
 
 
    //统计灰度级中每个像素在整幅图像中的个数  
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
        }
    }
 
 
    //遍历灰度级[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分  
            {
                pixelPro[i] = (float)pixelCount[i] / pixelSum;   //计算每个像素在整幅图像中的比例  
                w0 += pixelPro[j];//背景像素点占整个图像的比例
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分  
            {
                pixelPro[i] = (float)pixelCount[i] / pixelSum;   //计算每个像素在整幅图像中的比例  
                w1 += pixelPro[j];//前景像素点占整个图像的比例
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;//背景平均灰度μ0
        u1 = u1tmp / w1;//前景平均灰度μ1
        deltaTmp = (float)(w0 *w1* pow((u0 - u1), 2)); //类间方差公式 g = w1 * w2 * (u1 - u2) ^ 2
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i;
        }
    }
 
    return threshold;
    
}