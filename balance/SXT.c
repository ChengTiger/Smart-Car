#include "zf_common_headfile.h"
#include "SXT.h"
#include "Control.h"

int TOP=80;          //�趨������ֵ
int SXT1[120];
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
int y2;//�����ж�
int y;
int SXT2[188];
int Xend1=0; 
int Yend1=0;
int Pend1=0;

int PANDUAN[3][3];
int m=0,n=0;
/***************
    ��������
***************/
void balance();//ƽ�⺯��

/*****************************
�������ܣ�
�������ܣ�һ���ж��������ֵy
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


int SXTpoint()//�ж��Ƿ��еƣ�����ֵΪ1�����еơ�0�����޵ơ�
{
	int Point;//����жϱ�־λ
	
	int W=0;//�������
	int MAX=0;
	
	if(mt9v03x_finish_flag)
     {
		//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
		for(int i=0;i<121;i++)
		 {
			for(int j=0;j<189;j++)
			{
				if(mt9v03x_image[i][j]>TOP)//�趨������ֵ
				{
					mt9v03x_image[i][j]=1;
					W++;
				}
				else{mt9v03x_image[i][j]=0;}
				
		    }
			SXT1[i]=W;//�������
			if(MAX<SXT1[i])
			{	
			MAX=SXT1[i]; y=i;
			}
			W=0;//��ֵ����
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
�������ܣ�
�������ܣ�һ���ж��������ֵx
******************************/

int SXTPP()//ȷ���������λ��
{
	int x=0;//��������λ��
	int k=0,k1=0;//�и���
	int PX[189];
	int PK[189];
	int m=0,n=0;
	int max;
	for(int i=0;i<189;i++)
	{
		if(mt9v03x_image[y][i]==1)
		{
			x=x+i;//���λ�����ܺ�
			k++;//�й������ܺ�
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
	x=PX[k1]/PK[k1];//���λ��
	ips200_show_int(100,180,x,5);
	
	return x;//�����꣬�ڼ���
}



/*****************************
�������ܣ�
�������ܣ������ж��������ֵy
******************************/

int SXTpoint2()//�ж��Ƿ��еƣ�����ֵΪ1�����еơ�0�����޵ơ�
{
	int Point;//����жϱ�־λ
	
	int W=0;//�������
	int MAX=0;
	
		//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
		for(int i=0;i<188;i++)
		 {
			for(int j=0;j<120;j++)
			{
				if(mt9v03x_image[j][i]>TOP)//�趨������ֵ
				{
					mt9v03x_image[j][i]=1;
					W++;
				}
				else{mt9v03x_image[j][i]=0;}
				
		    }
			SXT2[i]=W;//�������
			if(MAX<SXT2[i])
			{	
			MAX=SXT2[i]; y2=i;//ȷ���ڼ���
			}
			W=0;//��ֵ����
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
�������ܣ�
�������ܣ������ж��������ֵx
******************************/

int SXTPP2()//ȷ���������λ�ã�����
{
	int x=0;//��������λ��
	int k=0,k1=0;//�и���
	int PX[120];
	int PK[120];
	int m=0,n=0;
	int max=0;
	for(int i=0;i<120;i++)
	{
		if(mt9v03x_image[i][y2]==1)
		{
			x=x+i;//���λ�����ܺ�
			k++;//�й������ܺ�
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
	x=PX[k1]/PK[k1];//���λ��
	//ips200_show_int(100,210,x,5);
	
	return x;//�����꣬�ڼ���
}

void ENDpoint()//���޹�������ж������λ�������ж�
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
//	}   //�жϹ��
//	L=(SXTPP()-y2)*(SXTPP()-y2)+(y-SXTPP2())*(y-SXTPP2());//����ƽ��ֵ
//	
//	ips200_show_int(0,150,Xend,5);
//	ips200_show_int(50,150,Yend,5);
//	ips200_show_int(100,150,Pend,5);
//	ips200_show_int(150,150,L,5);
}


void ENDPoint()
{
		SXTpoint();//�жϺ�����Ƿ����
	if(SXTpoint()==1)//����к���ƣ��жϵƵ�λ��
	{
		int PP;
		PP=SXTPP()-94;//λ��ƫ��
		//�ж��Ƶ�λ������߻����ұ�
		if(GFP_abs(SXTPP()-94))
		{
			OUT();//�����������
		}
		else if(PP==0)
		{
			balance();//ֱ�к���
		}
	}
	else
	{
		           //���û�еƣ����λ��
		PANXuan();//ԭ�������ҵ�
	}
}


/***************
�������ƣ���������
��������:��С��û���ҵ���ʱ����ת�򻷣�С��ԭ�������ҵ�
****************/
int PANXuan()
{
    extern float Turn_Kp;	
			Turn_Kp=0;//ת��ϵ���Ƕ�ֵ��С���Զ�ֵ��ת
			balance();//С��ֱ�����к���
}

/****************
�������ƣ���������������
�������ܣ��������ҵ��Ƶ�λ��ʱ����̬�������ӵ����з��������
****************/
int OUT()
{
	        extern int PP;
	        extern float Turn_Kp;
			extern int Zhuanxiang_Kp;//ת�����ϵ�������Է��ں����⣩
			Turn_Kp=Zhuanxiang_Kp*PP+Turn_Kp;//����ת��ϵ��
			balance();//С��ֱ�����к���
}


