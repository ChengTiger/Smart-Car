#ifndef  _CONTROL_H
#define  _CONTROL_H


int Vertical(float Med,float Angle,float gyro_Y);//ֱ����
int Velocity(int Target,int encoder_left,int encoder_right);//�ٶȻ�
int Turn(int gyro_Z);//ת��
void EXTI9_5_IRQHandler1(void);//�жϺ���
float ComplementaryFiltering();//��Ǯ�Ƕ���ȡ����
void pit_hanlder (void);//��������ȡ����
int Limit(int *motoA,int *motoB);//�޷�����
int GFP_abs(int p);//����ֵ����
void Load(int moto1,int moto2);//ʹ�ܺ���
void Stop(float Med_Jiaodu,float Jiaodu);
void EXTI9_5_IRQHandler2(void);//����ͷ�ж�ִ�к���
void TIM6_IRQHandler (void);
void FA(int Med_Angle,int Angle);//��ֹʧ��
int velocity(int encoder_left,int encoder_right);
void balance();//ƽ�⺯��
void pit_hanlderICM (void);//��ȡ����������
int SpeedUP();//���ٺ���
void huizheng();//��������
void zhuanxiang();//ת����
void huizhengT();//��������2
void RUN();//ǰ������
float Complementaryhover();//��ת����ȡ����
int turn(int encoder_left, int encoder_right, int gyro);//ת��PD����



//���������ź궨��
#define ENCODER_1                   TIM3_ENCOEDER
#define ENCODER_1_A                 TIM3_ENCOEDER_CH1_B4
#define ENCODER_1_B                 TIM3_ENCOEDER_CH2_B5

#define ENCODER_2		            TIM4_ENCOEDER
#define ENCODER_2_A                 TIM4_ENCOEDER_CH1_B6
#define ENCODER_2_B                 TIM4_ENCOEDER_CH2_B7

#define SXT		        E8

//����������ź궨��
#define DIR_L		        A0
#define PWM_L               TIM5_PWM_CH2_A1

#define DIR_R		        A2
#define PWM_R               TIM5_PWM_CH4_A3

//H2��LED�����ź궨��
#define LED1        	H2

#endif

