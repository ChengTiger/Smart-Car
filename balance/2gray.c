#include "zf_common_headfile.h"


//w0Ϊ�������ص�ռ����ͼ��ı���
// 
//u0Ϊw0ƽ���Ҷ�
// 
//w1Ϊǰ�����ص�ռ����ͼ��ı���
// 
//u1Ϊw1ƽ���Ҷ�
// 
//uΪ����ͼ���ƽ���Ҷ�
// 
//��䷽�ʽ g = w1 * w2 * (u1 - u2) ^ 2
 
int otsuThreshold(int *image, int col, int row)
{
    #define GrayScale 256
    int width = col;
    int height = row;
    int pixelCount[GrayScale] = {0}; //ÿ���Ҷ�ֵ��ռ���ظ���
    float pixelPro[GrayScale] = {0};//ÿ���Ҷ�ֵ��ռ�����ر���
    int i, j, pixelSum = width * height;   //������
    int threshold = 0;
    int* data = image;  //ָ���������ݵ�ָ��
 
 
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���  
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            pixelCount[(int)data[i * width + j]]++;  //������ֵ��Ϊ����������±�
        }
    }
 
 
    //�����Ҷȼ�[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++)     // i��Ϊ��ֵ
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //��������  
            {
                pixelPro[i] = (float)pixelCount[i] / pixelSum;   //����ÿ������������ͼ���еı���  
                w0 += pixelPro[j];//�������ص�ռ����ͼ��ı���
                u0tmp += j * pixelPro[j];
            }
            else   //ǰ������  
            {
                pixelPro[i] = (float)pixelCount[i] / pixelSum;   //����ÿ������������ͼ���еı���  
                w1 += pixelPro[j];//ǰ�����ص�ռ����ͼ��ı���
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;//����ƽ���ҶȦ�0
        u1 = u1tmp / w1;//ǰ��ƽ���ҶȦ�1
        deltaTmp = (float)(w0 *w1* pow((u0 - u1), 2)); //��䷽�ʽ g = w1 * w2 * (u1 - u2) ^ 2
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i;
        }
    }
 
    return threshold;
    
}