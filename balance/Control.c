#include "control.h"
#include "zf_common_headfile.h"
#include "SXT.h"
#include "isr.h" 

/*****************
变量定义部分
******************/

int PWM_out=0,S;//最终输出PWM

float Med_Angle=55.43;	//56.13//机械中值。---在这里修改你的机械中值即可。
float Target_Speed=-700;	//400//期望速度。---二次开发接口，用于控制小车前进后退及其速度。
                   //-1500
float Target_SpeedE=-700;
float 
	Vertical_Kp=750,//直立环KP、KD           650    490     750   690
	Vertical_Kd=-3.3;                  //   -3.3    -1.5   -3.3   -2.1
float 
	Velocity_Kp=-100.0,//速度环KP、KI    -0.9        -0.6  -100.0    -0.3
	Velocity_Ki=-0.5; //             -0.0045     -0.003    -0.5 -0.0015
float 
	SpeedUp=0,//速度增长变量(不要改了阿云，就是0)
	L=0.5,//速度突变限幅极限比例
	SpeedUpKp=0.3;//速度增长比例
float
	Turn_Kp=0,//转向环,正
	Turn_Kd=-0,
    Zhuanxiang_Kp=0;//转向控制比例系数
int TURN=0;//(记得查看目前是采取双轮，还是单轮旋转)
float TURN_Kp=-40;//（根据调试结果修改TURN_Kp的极性）

int
	PP=0;//光点位置偏差

int Vertical_out,Velocity_out,Turn_out;//直立环&速度环&转向环 的输出变量

int MOTO1=0,MOTO2=0;//输送到电机上的PWM值

int PWM_MAX=7200,PWM_MIN=-7200;	//PWM限幅变量

int STOP=0;//刹车

//编码器读取到的转速（编码器转速）
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
//陀螺仪加速度计变量
//icm_acc_x;
//icm_acc_y;
//icm_acc_z;
//陀螺仪数据变量
//icm_gyro_x
//icm_gyro_y
//icm_gyro_z


extern int Pend1,Xend1,Yend1;
//角度处理相关 
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
float Angle=0.0;//定义当前角度值
//旋转角
float angle_Filtering2 = 90; //
float angle_acc2 = 0;        //
float angle_gyro2 = 0;       //
float err_angle2 = 0;        //
float anglespeed2 = 0;       //
float K2 = 0.2;             //
float Angle1=0.0;//定义当前角度值


/******************
函数声明部分
*******************/

int Vertical(float Med,float Angle,float gyro_Y);//函数声明
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z); 
float ComplementaryFiltering();//角度求取函数 
void pit_hanlder (void);//编码器，陀螺仪数据读取函数
int Limit(int *motoA,int *motoB);//限幅函数
int GFP_abs(int p);//绝对值函数
void FA(int Med_Angle,int Angle);
void balance();//平衡函数
int SpeedUP();//增速函数
void huizheng();//回正函数
void zhuanxiang();//转向函数
void huizhengT();//回正函数2
void RUN();//前进函数
float Complementaryhover();//旋转角求取函数
int turn(int encoder_left, int encoder_right, int gyro);//转向环PD控制
void SXTpointEE();//光点寻找函数

/************************************************************************************************************/
/******************
中断函数：TIM6
*******************/

void TIM6_IRQHandler (void)
{	
			int location=0;
            TIM6->SR &= ~TIM6->SR;                                                     // 清空中断状态
			SXTpointEE();//光点寻找
			pit_hanlder ();
			ComplementaryFiltering();
			location=Xend1;//判定灯的位置
			TURN=TURN_Kp*(location-94);//（根据调试结果修改TURN_Kp的极性）
			if(TURN>=2000){TURN=2000;}
			if(TURN<=-2000){TURN=-2000;}//做转向限幅（根据调试情况寻找极限值）
/***************************************补救函数还没加**************************************/			
			if(Pend1)//判定是否有灯
			{
				
//				RUN();
				//balance();
				if(Angle>=56)
				{
					//huizheng();//回正函数
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
////					huizhengT();//回正函数
//				}
//				if(Angle>=51.6&&Angle<60)
//				{
//					balance();
//				}
//				if(Angle>=60)
//				{
//					huizheng();//回正函数
//				}
				Target_Speed=Target_SpeedE;
			}
}

/**************************************************************************************************************/

/****************
函数名称：前进函数（第一版）
函数功能：小车匀速前进
*****************/
void RUN()
{
			if((encoder_data_1+encoder_data_2)/2>=-200&&(encoder_data_1+encoder_data_2)/2<1500)
			{	
				//Target_Speed=-2000;
				balance();//走直线
			}
			else if((encoder_data_1+encoder_data_2)/2>=1500)//向前速度限幅
			{
				Target_Speed=Target_Speed-100;
				huizheng();//回正，防止超速
			}
			else if(encoder_data_1<-200||encoder_data_2<-200)//向后速度限幅
			{
				Target_Speed=Target_Speed-100;
				huizheng();//回正2，防止后倾
			}
			else
			{
				//Target_Speed=-2000;
				balance();//走直线
			}
}




/****************
增速函数
*****************/
int SpeedUP()
{
     float speed,actualspeed,ICM_V,Encoder_V,SpeedD;
	Encoder_V=(encoder_data_1+encoder_data_2)/2;//通过编码器提取小车速度
	ICM_V+=icm_acc_y;//根据加速度积分出速度
	actualspeed=(1-SpeedUpKp)*Encoder_V+SpeedUpKp*ICM_V;//比例器求出真实速度
	SpeedD=actualspeed+Target_Speed;//速度差值
	SpeedUp+=SpeedUpKp*SpeedUpKp*SpeedD+SpeedUpKp*actualspeed;//求出速度增长值
	if(GFP_abs(SpeedUp)-GFP_abs(actualspeed)>GFP_abs(SpeedUp)*L)//加速限幅，防止速度突变
	{
		SpeedUp=actualspeed;//突变复位
	}	
	if(SpeedUp<=Target_Speed)//达到目标速度则停止加速
	{
		SpeedUp=Target_Speed;
	}
	speed=SpeedUp;
	
	return speed;
}

/***************
函数名称：平衡函数
函数功能：使小车直立运行
***************/
void balance()
{
			//2.将数据压入闭环控制中，计算出控制输出量。
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//速度环
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//直立环
			//Turn_out=Turn(icm_gyro_z);	  //转向环																					//转向环
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out+Target_Speed;//最终输出
			//3.把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-Turn_out-TURN;//左电机
			MOTO2=PWM_out+Turn_out;//右电机
//			if(MOTO1>0){MOTO1=MOTO1+150;}
//			if(MOTO1<0){MOTO1=MOTO1-200;}
//			if(MOTO2>0){MOTO2=MOTO2+150;}
//			if(MOTO2<0){MOTO2=MOTO2-200;}
			//ips200_show_int(100,180,MOTO1,5);
			Limit(&MOTO1,&MOTO2);	 //PWM限幅	
			FA(Med_Angle,Angle);
}

/**************
函数名称：回正函数
函数作用：当小车接近失控时回正小车
***************/
void huizheng()
{
	        float Med_Angle=54.2;
			//2.将数据压入闭环控制中，计算出控制输出量。
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//速度环
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//直立环
			//Turn_out=Turn(icm_gyro_z);	  //转向环																					//转向环
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out;//最终输出
			//3.把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-Turn_out-TURN;//左电机
			MOTO2=PWM_out+Turn_out+TURN;//右电机
			Limit(&MOTO1,&MOTO2);	 //PWM限幅	
			FA(Med_Angle,Angle);
			//Load(STOP,STOP);
}

void huizhengT()
{
	        float Med_Angle=56.4;
			//2.将数据压入闭环控制中，计算出控制输出量。
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//速度环
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//直立环
			//Turn_out=Turn(icm_gyro_z);	  //转向环																					//转向环
			Turn_out=turn(encoder_data_1, encoder_data_2, icm_gyro_z);
	
			PWM_out=Vertical_out-Velocity_out;//最终输出
			//3.把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-Turn_out-TURN;//左电机
			MOTO2=PWM_out+Turn_out+TURN;//右电机
			Limit(&MOTO1,&MOTO2);	 //PWM限幅	
			FA(Med_Angle,Angle);
}

/***************
函数名称:转向函数
函数作用：转向
***************/
void zhuanxiang()
{
			Med_Angle=55.5,Turn_Kp=Turn_Kp+Zhuanxiang_Kp;//(这里Kp可能有逻辑漏洞)
			//S=SpeedUP();
	        pit_hanlder ();//获取加速度，陀螺仪，编码器速度
			//pit_hanlderICM ();
			//balance();//平衡函数
	        //ENDPoint();//总执行函数（包含摄像头）（摄像头提取还没结束（定时器未控制））
			ComplementaryFiltering();			        //角度
				
			//2.将数据压入闭环控制中，计算出控制输出量。
			Velocity(Target_Speed,encoder_data_1,encoder_data_2);
			//Velocity_out=velocity(encoder_data_1,encoder_data_2);	//速度环
			Vertical_out=Vertical(Med_Angle,Angle,icm_gyro_y);			//直立环
			Turn_out=Turn(icm_gyro_z);	  //转向环																					//转向环
			
			PWM_out=Vertical_out-Velocity_out;//最终输出
			//3.把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-Turn_out+TURN;//左电机
			MOTO2=PWM_out+Turn_out+100;//右电机
			Limit(&MOTO1,&MOTO2);	 //PWM限幅	
			FA(Med_Angle,Angle);
}



/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
	return PWM_out;
}


/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.计算速度偏差
	Encoder_Err=((encoder_left+encoder_right)-Target);//舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	//2.对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}

/**************************************
入口参数：电机编码器的值
返回  值：速度控制PWM
作    者：张巧龙
**************************************/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
    static float Encoder_Integral;
   //=============速度PI控制器=======================//  
    Encoder_Least =(encoder_left+encoder_right)-0;      
    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
    Encoder *= 0.7;          //===一阶低通滤波器       
    Encoder += Encoder_Least*0.3;   //===一阶低通滤波器    
    Encoder_Integral +=Encoder; //===积分出位移 积分时间：10ms
    if(Encoder_Integral>10000)    Encoder_Integral=10000;   
    //===积分限幅
    if(Encoder_Integral<-10000)    Encoder_Integral=-10000;   
    //===积分限幅  
    Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;  
    //===速度控制  
    //if(Angle<-40||Angle>40)   Encoder_Integral=0;   
    //===电机关闭后清除积分
    return Velocity;
}



/*********************
转向环：系数*Z轴角速度
*********************/
int Turn(int gyro_Z)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z;
	return PWM_out;
}

/*********************
转向环：系数*Z轴角速度PD控制
*********************/
int turn(int encoder_left, int encoder_right, int gyro)
{
	static int bias;
	int Turn_Amplitude=50, turn, encoder_temp;
	
	encoder_temp = encoder_left - encoder_right;
	bias += encoder_temp; //对角速度积分
	
	//限幅
	if(bias > Turn_Amplitude)  
    	bias = Turn_Amplitude;
	if(bias < -Turn_Amplitude) 
		bias = -Turn_Amplitude;
	
	turn = Turn_Kp * bias + Turn_Kd * gyro; //===结合Z轴陀螺仪进行PD控制
	
	return turn;
}



/**********************
   滤波算法求取俯仰角度
***********************/
float ComplementaryFiltering()               //角度返回值是浮点型
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
    //err_angle = angle_Filtering - angle_balance;   //最终角度
    // angleControlOut = Vertical_Kp * err_angle + Vertical_Kd * anglespeed;
	Angle= angle_Filtering;
	ips200_show_float(5,260,Angle,5,3);
	return Angle;
}

/**********************
   滤波算法求取旋转角度
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
    //err_angle = angle_Filtering - angle_balance;   //最终角度
    // angleControlOut = Vertical_Kp * err_angle + Vertical_Kd * anglespeed;
	Angle1= angle_Filtering2;
	ips200_show_int(50,260,Angle1,5);
	return Angle;
}




void pit_hanlder (void)
{
	//编码器数据已经取反完成输出极性一致
    encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
    encoder_clear_count(ENCODER_1);                                             // 清空编码器计数

    encoder_data_2 = -encoder_get_count(ENCODER_2);                              // 获取编码器计数
    encoder_clear_count(ENCODER_2);                                          	// 清空编码器计数	
	
	icm20602_get_acc();                                                         // 获取ICM20602的加速度测量数值
    icm20602_get_gyro();                                                        // 获取ICM20602的角速度测量数值	

}

void pit_hanlderICM (void)
{
    icm20602_get_acc();                                                         // 获取ICM20602的加速度测量数值
    icm20602_get_gyro();                                                        // 获取ICM20602的角速度测量数值	
}


/*限幅函数*/
int Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*************************
绝对值函数
**************************/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}	


/***********************
赋值函数
入口参数：PID运算完成后的最终PWM值
***********************/
void Load(int moto1,int moto2)
{
	//1.研究正负号，对应正反转
	if(moto1>0) {gpio_set(DIR_L, GPIO_LOW),
				pwm_set_duty(PWM_L, moto1);
				}//正转
				
	else        {gpio_set(DIR_L, GPIO_HIGH),
				pwm_set_duty(PWM_L, -moto1);
				}//反转
				
	//2.研究PWM值
	
//	ips200_show_int(50,260,moto1,5);
	
	if(moto2>0) {gpio_set(DIR_R, GPIO_LOW),
				pwm_set_duty(PWM_R, moto2);
				}//正转(左边)
	
	else        {gpio_set(DIR_R, GPIO_HIGH),
				pwm_set_duty(PWM_R, -moto2);
				}//反转
	
//	ips200_show_int(100,260,moto2,5);
}

float PWM_Zero=0,stop=0;
void Stop(float Med_Jiaodu,float Jiaodu)
{
	if(GFP_abs(Jiaodu-Med_Jiaodu)>20)Load(PWM_Zero,PWM_Zero);
}

/*****************************
函数功能：防止车子失控
入口参数：Med_Angle;Angle
******************************/
void FA(int Med_Angle,int Angle)
{
	int a=0;
	if(GFP_abs(Angle-Med_Angle)<50)
	{
		Load(MOTO1,MOTO2);		 //加载到电机上。
	}
	if(GFP_abs(Angle-Med_Angle)>=50)
	{
		Load(a,a);		 //加载到电机上。
	}
}



