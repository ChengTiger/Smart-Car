#ifndef  _CONTROL_H
#define  _CONTROL_H


int Vertical(float Med,float Angle,float gyro_Y);//直立环
int Velocity(int Target,int encoder_left,int encoder_right);//速度环
int Turn(int gyro_Z);//转向环
void EXTI9_5_IRQHandler1(void);//中断函数
float ComplementaryFiltering();//但钱角度求取函数
void pit_hanlder (void);//传感器读取函数
int Limit(int *motoA,int *motoB);//限幅函数
int GFP_abs(int p);//绝对值函数
void Load(int moto1,int moto2);//使能函数
void Stop(float Med_Jiaodu,float Jiaodu);
void EXTI9_5_IRQHandler2(void);//摄像头中断执行函数
void TIM6_IRQHandler (void);
void FA(int Med_Angle,int Angle);//防止失控
int velocity(int encoder_left,int encoder_right);
void balance();//平衡函数
void pit_hanlderICM (void);//获取陀螺仪数据
int SpeedUP();//增速函数
void huizheng();//回正函数
void zhuanxiang();//转向函数
void huizhengT();//回正函数2
void RUN();//前进函数
float Complementaryhover();//旋转角求取函数
int turn(int encoder_left, int encoder_right, int gyro);//转向环PD控制



//编码器引脚宏定义
#define ENCODER_1                   TIM3_ENCOEDER
#define ENCODER_1_A                 TIM3_ENCOEDER_CH1_B4
#define ENCODER_1_B                 TIM3_ENCOEDER_CH2_B5

#define ENCODER_2		            TIM4_ENCOEDER
#define ENCODER_2_A                 TIM4_ENCOEDER_CH1_B6
#define ENCODER_2_B                 TIM4_ENCOEDER_CH2_B7

#define SXT		        E8

//电机驱动引脚宏定义
#define DIR_L		        A0
#define PWM_L               TIM5_PWM_CH2_A1

#define DIR_R		        A2
#define PWM_R               TIM5_PWM_CH4_A3

//H2号LED灯引脚宏定义
#define LED1        	H2

#endif

