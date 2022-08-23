#include "control.h"
#include "zf_common_headfile.h"
#include "SXT.h"
#include "isr.h" 

/*****************
�������岿��
******************/

int PWM_out=0,S;//�������PWM

float Med_Angle=55.43;	//56.13//��е��ֵ��---�������޸���Ļ�е��ֵ���ɡ�
float Target_Speed=-700;	//400//�����ٶȡ�---���ο����ӿڣ����ڿ���С��ǰ�����˼����ٶȡ�
                   //-1500
float Target_SpeedE=-700;
float 
	Vertical_Kp=750,//ֱ����KP��KD           650    490     750   690
	Vertical_Kd=-3.3;                  //   -3.3    -1.5   -3.3   -2.1
float 
	Velocity_Kp=-100.0,//�ٶȻ�KP��KI    -0.9        -0.6  -100.0    -0.3
	Velocity_Ki=-0.5; //             -0.0045     -0.003    -0.5 -0.0015
float 
	SpeedUp=0,//�ٶ���������(��Ҫ���˰��ƣ�����0)
	L=0.5,//�ٶ�ͻ���޷����ޱ���
	SpeedUpKp=0.3;//�ٶ���������
float
	Turn_Kp=0,//ת��,��
	Turn_Kd=-0,
    Zhuanxiang_Kp=0;//ת����Ʊ���ϵ��
int TURN=0;//(�ǵò鿴Ŀǰ�ǲ�ȡ˫�֣����ǵ�����ת)
float TURN_Kp=-40;//�����ݵ��Խ���޸�TURN_Kp�ļ��ԣ�

int
	PP=0;//���λ��ƫ��

int Vertical_out,Velocity_out,Turn_out;//ֱ����&�ٶȻ�&ת�� ���������

int MOTO1=0,MOTO2=0;//���͵�����ϵ�PWMֵ

int PWM_MAX=7200,PWM_MIN=-7200;	//PWM�޷�����

int STOP=0;//ɲ��

//��������ȡ����ת�٣�������ת�٣�
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
//�����Ǽ��ٶȼƱ���
//icm_acc_x;
//icm_acc_y;
//icm_acc_z;
//���������ݱ���
//icm_gyro_x
//icm_gyro_y
//icm_gyro_z


extern int Pend1,Xend1,Yend1;
//�Ƕȴ������ 
float acc_x = 0, acc_z = 0,acc_y = 0; 
float gyro_y = 0,gyro_z = 0,angle_z = 0;          
float angle_Filtering = 90; //
float angle_acc = 0;        //
float angle_gyro = 0;       //
float err_angle = 0;        //
float anglespeed = 0;       //
float dtt = 0.001;          //
float K1 = 0.1900;           //0.325 0.39
float x1=0,x2=0,y1=0;       //
float Angle=0.0;//���嵱ǰ�Ƕ�ֵ
//��ת��
float angle_Filtering2 = 90; //
float angle_acc2 = 0;        //
float angle_gyro2 = 0;       //
float err_angle2 = 0;        //
float anglespeed2 = 0;       //
float K2 = 0.2;             //
float Angle1=0.0;//���嵱ǰ�Ƕ�ֵ


/******************
������������
*******************/

int Vertical(float Med,float Angle,float gyro_Y);//��������
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z); 
float ComplementaryFiltering();//�Ƕ���ȡ���� 
void pit_hanlder (void);//�����������������ݶ�ȡ����
int Limit(int *motoA,int *motoB);//�޷�����
int GFP_abs(int p);//����ֵ����
void FA(int Med_Angle,int Angle);
void balance();//ƽ�⺯��
int SpeedUP();//���ٺ���
void huizheng();//��������
void zhuanxiang();//ת����
void huizhengT();//��������2
void RUN();//ǰ������
float Complementaryhover();//��ת����ȡ����
int turn(int encoder_left, int encoder_right, int gyro);//ת��PD����
void SXTpointEE();//���Ѱ�Һ���

/************************************************************************************************************/
/******************
�жϺ�����TIM6
*******************/

void TIM6_IRQHandler (void)
{	
			int location=0;
            TIM6->SR &= ~TIM6->SR;                                                     // ����ж�״̬
			SXTpointEE();//���Ѱ��
			pit_hanlder ();
			ComplementaryFiltering();
			location=Xend1;//�ж��Ƶ�λ��
			TURN=TURN_Kp*(location-94);//�����ݵ��Խ���޸�TURN_Kp�ļ��ԣ�
			if(TURN>=2000){TURN=2000;}
			if(TURN<=-2000){TURN=-2000;}//��ת���޷������ݵ������Ѱ�Ҽ���ֵ��
/***************************************���Ⱥ�����û��**************************************/			
			if(Pend1)//�ж��Ƿ��е�
			{
				
//				RUN();
				//balance();
				if(Angle>=56)
				{
					//huizheng();//��������
					Target_Speed=Target_Speed+1100;
					balance();
					Target_Speed=-550;  //600
				}
				else
				{
					
					balance();
				}
					Target_Speed=Target_SpeedE;
			}
			else
			{
				Med_Angle=57;
				TURN=-1100;Target_Speed=-550; //550
				balance();
//				if(Angle<51.6)
//				{
//					///Target_Speed=-3000;
//					
//					balance();
////					huizhengT();//��������
//				}
//				if(Angle>=51.6&&Angle<60)
//				{
//					balance();
//				}
//				if(Angle>=60)
//				{
//					huizheng();//��������
//				}
				Target_Speed=Target_SpeedE;
			}
}

/**************************************************************************************************************/

/****************
�������ƣ�ǰ����������һ�棩
�������ܣ�С������ǰ��
*****************/
void RUN()
{
			if((encoder_data_1+encoder_data_2)/2>=-200&&(encoder_data_1+encoder_data_2)/2<1500)
			{	
				//Target_Speed=-2000;
				balance();//��ֱ��
			}
			else if((encoder_data_1+encoder_data_2)/2>=1500)//��ǰ�ٶ��޷�
			{
				Target_Speed=Target_Speed-100;
				huizheng();//��������ֹ����
			}
			else if(encoder_data_1<-200||encoder_data_2<-200)//����ٶ��޷�
			{
				Target_Speed=Target_Speed-100;
				huizheng();//����2����ֹ����
			}
			else
			{
				//Target_Speed=-2000;
				balance();//��ֱ��
			}
}




/****************
���ٺ���
*****************/
int SpeedUP()
{
     float speed,actualspeed,ICM_V,Encoder_V,SpeedD;
	Encoder_V=(encoder_data_1+encoder_data_2)/2;//ͨ����������ȡС���ٶ�
	ICM_V+=icm_acc_y;//���ݼ��ٶȻ��ֳ��ٶ�
	actualspeed=(1-SpeedUpKp)*Encoder_V+SpeedUpKp*ICM_V;//�����������ʵ�ٶ�
	SpeedD=actualspeed+Target_Speed;//�ٶȲ�ֵ
	SpeedUp+=SpeedUpKp*SpeedUpKp*SpeedD+SpeedUpKp*actualspeed;//����ٶ�����ֵ
	if(GFP_abs(SpeedUp)-GFP_abs(actualspeed)>GFP_abs(SpeedUp)*L)//�����޷�����ֹ�ٶ�ͻ��
	{
		SpeedUp=actualspeed;//ͻ�临λ
	}	
	if(SpeedUp<=Target_Speed)//�ﵽĿ���ٶ���ֹͣ����
	{
		SpeedUp=Target_Speed;
	}
	speed=SpeedUp;
	
	return speed;
}

/***************
�������ƣ�ƽ�⺯��
�������ܣ�ʹС��ֱ������
***************/
void balance()
{
			//2.������ѹ��ջ������У�����������������
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//�ٶȻ�
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//ֱ����
			//Turn_out=Turn(icm_gyro_z);	  //ת��																					//ת��
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out+Target_Speed;//�������
			//3.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
			MOTO1=PWM_out-Turn_out-TURN;//����
			MOTO2=PWM_out+Turn_out;//�ҵ��
//			if(MOTO1>0){MOTO1=MOTO1+150;}
//			if(MOTO1<0){MOTO1=MOTO1-200;}
//			if(MOTO2>0){MOTO2=MOTO2+150;}
//			if(MOTO2<0){MOTO2=MOTO2-200;}
			//ips200_show_int(100,180,MOTO1,5);
			Limit(&MOTO1,&MOTO2);	 //PWM�޷�	
			FA(Med_Angle,Angle);
}

/**************
�������ƣ���������
�������ã���С���ӽ�ʧ��ʱ����С��
***************/
void huizheng()
{
	        float Med_Angle=54.2;
			//2.������ѹ��ջ������У�����������������
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//�ٶȻ�
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//ֱ����
			//Turn_out=Turn(icm_gyro_z);	  //ת��																					//ת��
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out;//�������
			//3.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
			MOTO1=PWM_out-Turn_out-TURN;//����
			MOTO2=PWM_out+Turn_out+TURN;//�ҵ��
			Limit(&MOTO1,&MOTO2);	 //PWM�޷�	
			FA(Med_Angle,Angle);
			//Load(STOP,STOP);
}

void huizhengT()
{
	        float Med_Angle=56.4;
			//2.������ѹ��ջ������У�����������������
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//�ٶȻ�
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//ֱ����
			//Turn_out=Turn(icm_gyro_z);	  //ת��																					//ת��
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out;//�������
			//3.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
			MOTO1=PWM_out-Turn_out-TURN;//����
			MOTO2=PWM_out+Turn_out+TURN;//�ҵ��
			Limit(&MOTO1,&MOTO2);	 //PWM�޷�	
			FA(Med_Angle,Angle);
}

/***************
��������:ת����
�������ã�ת��
***************/
void zhuanxiang()
{
			Med_Angle=55.5,Turn_Kp=Turn_Kp+Zhuanxiang_Kp;//(����Kp�������߼�©��)
			//S=SpeedUP();
	        pit_hanlder ();//��ȡ���ٶȣ������ǣ��������ٶ�
			//pit_hanlderICM ();
			//balance();//ƽ�⺯��
	        //ENDPoint();//��ִ�к�������������ͷ��������ͷ��ȡ��û��������ʱ��δ���ƣ���
			ComplementaryFiltering();			        //�Ƕ�
				
			//2.������ѹ��ջ������У�����������������
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//�ٶȻ�
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//ֱ����
			Turn_out=Turn(icm_gyro_z);	  //ת��																					//ת��
			
			PWM_out=Vertical_out-Velocity_out;//�������
			//3.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
			MOTO1=PWM_out-Turn_out+TURN;//����
			MOTO2=PWM_out+Turn_out+100;//�ҵ��
			Limit(&MOTO1,&MOTO2);	 //PWM�޷�	
			FA(Med_Angle,Angle);
}



/*********************
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}


/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.�����ٶ�ƫ��
	Encoder_Err=((encoder_left+encoder_right)-Target);//��ȥ���--�ҵ���⣺�ܹ����ٶ�Ϊ"0"�ĽǶȣ����ǻ�е��ֵ��
	//2.���ٶ�ƫ����е�ͨ�˲�
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S+=EnC_Err_Lowout;
	//4.�����޷�
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	
	//5.�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}

/**************************************
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
��    �ߣ�������
**************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
    static float Encoder_Integral;
   //=============�ٶ�PI������=======================//  
    Encoder_Least =(encoder_left+encoder_right)-0;      
    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
    Encoder *= 0.7;          //===һ�׵�ͨ�˲���       
    Encoder += Encoder_Least*0.3;   //===һ�׵�ͨ�˲���    
    Encoder_Integral +=Encoder; //===���ֳ�λ�� ����ʱ�䣺10ms
    if(Encoder_Integral>10000)    Encoder_Integral=10000;   
    //===�����޷�
    if(Encoder_Integral<-10000)    Encoder_Integral=-10000;   
    //===�����޷�  
    Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;  
    //===�ٶȿ���  
    //if(Angle<-40||Angle>40)   Encoder_Integral=0;   
    //===����رպ��������
    return Velocity;
}



/*********************
ת�򻷣�ϵ��*Z����ٶ�
*********************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z;
	return PWM_out;
}

/*********************
ת�򻷣�ϵ��*Z����ٶ�PD����
*********************/
int turn(int encoder_left, int encoder_right, int gyro)
{
	static int bias;
	int Turn_Amplitude=50, turn, encoder_temp;
	
	encoder_temp = encoder_left - encoder_right;
	bias += encoder_temp; //�Խ��ٶȻ���
	
	//�޷�
	if(bias > Turn_Amplitude)  
    	bias = Turn_Amplitude;
	if(bias < -Turn_Amplitude) 
		bias = -Turn_Amplitude;
	
	turn = Turn_Kp * bias + Turn_Kd * gyro; //===���Z�������ǽ���PD����
	
	return turn;
}



/**********************
   �˲��㷨��ȡ�����Ƕ�
***********************/
float ComplementaryFiltering()               //�Ƕȷ���ֵ�Ǹ�����
{
	int angleControlOut,angle_balance;
    if (icm_acc_x >= 0)
        icm_acc_x = -1;                                
    acc_x = - icm_acc_x * 9.8 / 4096.0;                
    acc_z = icm_acc_z * 9.8 / 4096.0;                   
    gyro_y = icm_gyro_y / 16.4;                         

    anglespeed = - (gyro_y - 0.25);                     
    angle_acc = atan((acc_z) / acc_x) * 57.3;           
    angle_gyro = angle_Filtering + anglespeed * dtt;    
    angle_Filtering = K1 * angle_acc + (1 - K1) * angle_gyro;
    //err_angle = angle_Filtering - angle_balance;   //���սǶ�
    // angleControlOut = Vertical_Kp * err_angle + Vertical_Kd * anglespeed;
	Angle= angle_Filtering;
	ips200_show_float(5,260,Angle,5,3);
	return Angle;
}

/**********************
   �˲��㷨��ȡ��ת�Ƕ�
***********************/
float Complementaryhover()
{
	int angleControlOut,angle_balance;
    if (icm_acc_x >= 0)
        icm_acc_x = -1;                                
    acc_x = - icm_acc_x * 9.8 / 4096.0;                
    acc_y = icm_acc_y * 9.8 / 4096.0;                   
    gyro_z = icm_gyro_z / 16.4;                         

    anglespeed2 = - (gyro_z - 0.25);                     
    angle_acc2 = atan((acc_y) / acc_x) * 57.3;           
    angle_gyro2 = angle_Filtering + anglespeed * dtt;    
    angle_Filtering2 = K2 * angle_acc + (1 - K2) * angle_gyro;
    //err_angle = angle_Filtering - angle_balance;   //���սǶ�
    // angleControlOut = Vertical_Kp * err_angle + Vertical_Kd * anglespeed;
	Angle1= angle_Filtering2;
	ips200_show_int(50,260,Angle1,5);
	return Angle;
}




void pit_hanlder (void)
{
	//�����������Ѿ�ȡ������������һ��
    encoder_data_1 = encoder_get_count(ENCODER_1);                              // ��ȡ����������
    encoder_clear_count(ENCODER_1);                                             // ��ձ���������

    encoder_data_2 = -encoder_get_count(ENCODER_2);                              // ��ȡ����������
    encoder_clear_count(ENCODER_2);                                          	// ��ձ���������	
	
	icm20602_get_acc();                                                         // ��ȡICM20602�ļ��ٶȲ�����ֵ
    icm20602_get_gyro();                                                        // ��ȡICM20602�Ľ��ٶȲ�����ֵ	

}

void pit_hanlderICM (void)
{
    icm20602_get_acc();                                                         // ��ȡICM20602�ļ��ٶȲ�����ֵ
    icm20602_get_gyro();                                                        // ��ȡICM20602�Ľ��ٶȲ�����ֵ	
}


/*�޷�����*/
int Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*************************
����ֵ����
**************************/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}	


/***********************
��ֵ����
��ڲ�����PID������ɺ������PWMֵ
***********************/
void Load(int moto1,int moto2)
{
	//1.�о������ţ���Ӧ����ת
	if(moto1>0) {gpio_set(DIR_L, GPIO_LOW),
				pwm_set_duty(PWM_L, moto1);
				}//��ת
				
	else        {gpio_set(DIR_L, GPIO_HIGH),
				pwm_set_duty(PWM_L, -moto1);
				}//��ת
				
	//2.�о�PWMֵ
	
//	ips200_show_int(50,260,moto1,5);
	
	if(moto2>0) {gpio_set(DIR_R, GPIO_LOW),
				pwm_set_duty(PWM_R, moto2);
				}//��ת(���)
	
	else        {gpio_set(DIR_R, GPIO_HIGH),
				pwm_set_duty(PWM_R, -moto2);
				}//��ת
	
//	ips200_show_int(100,260,moto2,5);
}

float PWM_Zero=0,stop=0;
void Stop(float Med_Jiaodu,float Jiaodu)
{
	if(GFP_abs(Jiaodu-Med_Jiaodu)>20)Load(PWM_Zero,PWM_Zero);
}

/*****************************
�������ܣ���ֹ����ʧ��
��ڲ�����Med_Angle;Angle
******************************/
void FA(int Med_Angle,int Angle)
{
	int a=0;
	if(GFP_abs(Angle-Med_Angle)<50)
	{
		Load(MOTO1,MOTO2);		 //���ص�����ϡ�
	}
	if(GFP_abs(Angle-Med_Angle)>=50)
	{
		Load(a,a);		 //���ص�����ϡ�
	}
}



