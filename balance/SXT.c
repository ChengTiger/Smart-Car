#include "zf_common_headfile.h"
#include "SXT.h"
#include "Control.h"

int TOP=80;          //设定亮度阈值
int SXT1[120];
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
int y2;//二级判定
int y;
int SXT2[188];
int Xend1=0; 
int Yend1=0;
int Pend1=0;

int PANDUAN[3][3];
int m=0,n=0;
/***************
    函数声明
***************/
void balance();//平衡函数

/*****************************
函数介绍：
函数功能：一级判定光点坐标值y
******************************/
void SXTpointE()
{
	int i=0,j=0;
	int Pend=0,Xend=0,Yend=0;
	if(mt9v03x_finish_flag)
	{	
		//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
	while(Pend<1)
	{
		if(mt9v03x_image[i][j]>TOP) 	
		{
			Pend=1;
			Xend=(2*j+1)/2;
			Yend=i;
		}
		else
		{
			//if(j+1<=187)
			if(j<=187)	
			{
				j++;
			}
			else
			{
				i++;
				j=0;
			}
		}
		ips200_show_int(0,180,Pend,5);
		ips200_show_int(50,180,Xend,5);
		ips200_show_int(100,180,Yend,5);
		ips200_show_int(0,210,i,5);
		ips200_show_int(50,210,j,5);
	}
	}
}

void SXTpointEEE()
{	
	int Pend=0,Xend=0,Yend=0;
	if(mt9v03x_finish_flag)
	{//ips200_show_gray_image(0,0,(const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 80);
	for(int i=0;i<=119;i++)
	{
		for(int j=0;j<=187;j++)
		{
			if(mt9v03x_image[i][j]>TOP)
			{
				Pend=1;
				Xend=j;
				Yend=i;
					PANDUAN[0][0]=0;PANDUAN[0][1]=0;PANDUAN[0][2]=0;
					PANDUAN[0][1]=0;PANDUAN[1][1]=0;PANDUAN[1][2]=0;
					PANDUAN[0][2]=0;PANDUAN[2][1]=0;PANDUAN[2][2]=0;
			}
					PANDUAN[m][n]=Pend;
					PANDUAN[m+1][n]=Xend;
					PANDUAN[m+2][n]=Yend;
				if(n==2)
				{
					if(PANDUAN[0][0]+PANDUAN[0][1]+PANDUAN[0][2]>=1)
					{
						Pend1=1;
						if(PANDUAN[1][0]){Xend1=PANDUAN[1][0];Yend1=PANDUAN[2][0];}
							else if(PANDUAN[1][1]){Xend1=PANDUAN[1][1];Yend1=PANDUAN[2][1];}
								else{Xend1=PANDUAN[1][2];Yend1=PANDUAN[2][2];}
					}
					m=0;n=0;
				}
			    else
				{n++;}
		}
		if(Pend1==1)
		{
			break;
		}
	}
	//Pend1=Pend;Xend1=Xend;Yend1=Yend;
	ips200_show_int(0,180,Pend1,5);
		ips200_show_int(50,180,Xend1,5);
		ips200_show_int(100,180,Yend1,5);
    }
}


void SXTpointEE()
{
	int Pend=0,Xend=0,Yend=0;
	
	int EE[2];
	
	if(mt9v03x_finish_flag)
	{	
	//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
	for(int i=0;i<=119;i++)
	{
		for(int j=0;j<=187;j++)
		{
			if(mt9v03x_image[i][j]>TOP)
			{
				Pend=1;
				Xend=j;
				Yend=i;
				break;
			}
		}
		if(Pend==1)
		{
			break;
		}
	
	}

	Pend1=Pend;Xend1=Xend;Yend1=Yend;
   
//	EE[0]=0;EE[1]=0;
//	if(EE[0]==0&&Pend==0)
//	{
//		Pend1=0;Xend1=Xend;
//		EE[0]=Pend;EE[1]=Xend;
//	}
//	if(EE[0]==0&&Pend==1)
//	{
//		Pend1=1;Xend1=Xend;
//		EE[0]=Pend;EE[1]=Xend;
//	}
//	if(EE[0]==1&&Pend==0)
//	{
//		Pend1=1;Xend1=EE[1];
//		EE[0]=1;EE[1]=EE[1];
//	}
//	if(EE[0]==1&&Pend==1)
//	{
//		Pend1=1;Xend1=Xend;
//		EE[0]=1;EE[1]=Xend;
//	}
		
	
	ips200_show_int(0,180,Pend1,5);
	ips200_show_int(50,180,Xend1,5);
	ips200_show_int(100,180,Yend1,5);
    }
}


int SXTpoint()//判断是否有灯，返回值为1，则有灯。0，则无灯。
{
	int Point;//光点判断标志位
	
	int W=0;//行亮点和
	int MAX=0;
	
	if(mt9v03x_finish_flag)
     {
		//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
		for(int i=0;i<121;i++)
		 {
			for(int j=0;j<189;j++)
			{
				if(mt9v03x_image[i][j]>TOP)//设定亮度阈值
				{
					mt9v03x_image[i][j]=1;
					W++;
				}
				else{mt9v03x_image[i][j]=0;}
				
		    }
			SXT1[i]=W;//行向求和
			if(MAX<SXT1[i])
			{	
			MAX=SXT1[i]; y=i;
			}
			W=0;//赋值清零
		 }
		if(MAX>1)
		{
			Point=1;
		}
		else{Point=0;}
	 }	
	 ips200_show_int(50,180,Point,5);
	 
	return Point;
}


/*****************************
函数介绍：
函数功能：一级判定光点坐标值x
******************************/

int SXTPP()//确定光点坐标位置
{
	int x=0;//光点横坐标位置
	int k=0,k1=0;//列个数
	int PX[189];
	int PK[189];
	int m=0,n=0;
	int max;
	for(int i=0;i<189;i++)
	{
		if(mt9v03x_image[y][i]==1)
		{
			x=x+i;//光点位置列总和
			k++;//列光点个数总和
		}
		if(k>=1)
		{
			if(mt9v03x_image[y][i]==0)
			{
					PX[m]=x;m++;
					PK[n]=k;n++;
					x=0;k=0;
			}
		}

	}
	for(int L=0;L<n;L++)
	{
		if(max<PK[L])
		{
			max=PK[L];
			k1=L;
		}
	}
	x=PX[k1]/PK[k1];//光点位置
	ips200_show_int(100,180,x,5);
	
	return x;//行坐标，第几列
}



/*****************************
函数介绍：
函数功能：二级判定光点坐标值y
******************************/

int SXTpoint2()//判断是否有灯，返回值为1，则有灯。0，则无灯。
{
	int Point;//光点判断标志位
	
	int W=0;//行亮点和
	int MAX=0;
	
		//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
		for(int i=0;i<188;i++)
		 {
			for(int j=0;j<120;j++)
			{
				if(mt9v03x_image[j][i]>TOP)//设定亮度阈值
				{
					mt9v03x_image[j][i]=1;
					W++;
				}
				else{mt9v03x_image[j][i]=0;}
				
		    }
			SXT2[i]=W;//行向求和
			if(MAX<SXT2[i])
			{	
			MAX=SXT2[i]; y2=i;//确定第几列
			}
			W=0;//赋值清零
		 }
		if(MAX>1)
		{
			Point=1;
		}
		else{Point=0;}
		
	// ips200_show_int(50,210,Point,5);
	 
	return Point;
}



/*****************************
函数介绍：
函数功能：二级判定光点坐标值x
******************************/

int SXTPP2()//确定光点坐标位置，二级
{
	int x=0;//光点横坐标位置
	int k=0,k1=0;//列个数
	int PX[120];
	int PK[120];
	int m=0,n=0;
	int max=0;
	for(int i=0;i<120;i++)
	{
		if(mt9v03x_image[i][y2]==1)
		{
			x=x+i;//光点位置列总和
			k++;//列光点个数总和
		}
		if(k>=1)
		{
			if(mt9v03x_image[i][y2]==0)
			{
					PX[m]=x;m++;
					PK[n]=k;n++;
					x=0;k=0;
			}
		}

	}
	for(int L=0;L<n;L++)
	{
		if(max<PK[L])
		{
			max=PK[L];
			k1=L;
		}
	}
	x=PX[k1]/PK[k1];//光点位置
	//ips200_show_int(100,210,x,5);
	
	return x;//列坐标，第几行
}

void ENDpoint()//有无光点最终判定，光点位置最终判定
{
//	int L;
//	if(SXTpoint()||SXTpoint2())
//	{
//		if(L<500)
//	  {
//		Xend=(SXTPP()+y2)/2;
//		Yend=(y+SXTPP2())/2;
//	  }
//		Pend=1;
//	}
//	else
//	{
//	    Pend=0;
//		Xend=0;
//		Yend=0;
//		
//	}   //判断光点
//	L=(SXTPP()-y2)*(SXTPP()-y2)+(y-SXTPP2())*(y-SXTPP2());//距离平方值
//	
//	ips200_show_int(0,150,Xend,5);
//	ips200_show_int(50,150,Yend,5);
//	ips200_show_int(100,150,Pend,5);
//	ips200_show_int(150,150,L,5);
}


void ENDPoint()
{
		SXTpoint();//判断红外灯是否存在
	if(SXTpoint()==1)//如果有红外灯，判断灯的位置
	{
		int PP;
		PP=SXTPP()-94;//位置偏差
		//判定灯的位置在左边还是右边
		if(GFP_abs(SXTPP()-94))
		{
			OUT();//方向调整函数
		}
		else if(PP==0)
		{
			balance();//直行函数
		}
	}
	else
	{
		           //如果没有灯，清除位置
		PANXuan();//原地盘旋找灯
	}
}


/***************
函数名称：盘旋函数
函数功能:当小车没有找到灯时，打开转向环，小车原地盘旋找灯
****************/
int PANXuan()
{
    extern float Turn_Kp;	
			Turn_Kp=0;//转向环系数是定值，小车以定值旋转
			balance();//小车直立运行函数
}

/****************
函数名称：方向调整输出函数
函数功能：当函数找到灯的位置时，动态调整车子的运行方向跑向灯
****************/
int OUT()
{
	        extern int PP;
	        extern float Turn_Kp;
			extern int Zhuanxiang_Kp;//转向比例系数（可以放在函数外）
			Turn_Kp=Zhuanxiang_Kp*PP+Turn_Kp;//计算转向环系数
			balance();//小车直立运行函数
}


