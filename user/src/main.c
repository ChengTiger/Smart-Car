/*********************************************************
* 以下所有主函数代码版权均属安徽大学所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留安徽大学的版权声明。
*
* @file             main
* @company          安徽大学
* @author           王默然   郭彩云   张晨晨
* @Software         Keil5
* @Target core      MM32F3277G9P1
* @date             2022-5-25
*********************************************************/

#include "zf_common_headfile.h"
#include "Control.h"
#include "SXT.h"

//*********************宏定义****************************//

#define BEEP   D12

/*****************************
  变量定义
***************************/
int T=0;//定时器计数
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern int Target_Speed;
extern int16 encoder_data_1;
extern int16 encoder_data_2;

/*******************
//函数声明
*******************/

void pit_hanlder (void);   //采集编码器速度数据和陀螺仪位置数据和加速度数据
int Turn(int gory_Z);  //转向环
int GFP_abs(int p);    //绝对值
int Vertical(float Med,float Angle,float gyro_Y); //直立环
int Velocity(int Target,int encoder_left,int encoder_right);//速度环
void PID(void); //PID运算执行
void Load(int moto1,int moto2); //PID运算完成后的最终PWM值
float ComplementaryFiltering();//角度算法
void ComplementaryFiltering2();//二阶角度滤波函数
int GFP_abs(int p);//绝对值函数
int Limit(int *motoA,int *motoB);//限幅函数
void    exti_init     (gpio_pin_enum pin, exti_trigger_enum trigger);//中断初始化
void    exti_enable   (gpio_pin_enum pin);//中断使能
void EXTI9_5_IRQHandler1(void);//中断函数
void timer_clock_enable (timer_index_enum index);//定时器使能
void timer_start  (timer_index_enum index, timer_mode_enum mode);//定时器开始计数
void clock_init (uint32 clock);//核心时钟初始化
void EXTI9_5_IRQHandler2(void);//摄像头中断执行函数
uint16 timer_get (timer_index_enum index);//读取定时器计数
void timer_clear (timer_index_enum index);//清除定时器计数
void RCC_EnableAPB1Periphs(uint32_t apb1_periphs, bool enable);
void TIM5_IRQHandler1 (void);
uint8 timer_funciton_check (timer_index_enum index,timer_function_enum mode);//定时器模式
void pit_init (pit_index_enum pit_n, uint32 period);
int SXTpoint();//判断有无光点，返回值为Point，1为有光点，0为无光点
int SXTPP();//确定光点坐标位置,返回值为x（光点位置平均值）
void ENDpoint();//有无光点最终判定，光点位置最终判定
void tsl1401_init (void);
void First();
void balance();//平衡函数

// ****************************主函数****************************
int main (void)
{
/**********************************************************************
	                        初始化部分
***********************************************************************/
	
    encoder_init_quad(ENCODER_1, ENCODER_1_A, ENCODER_1_B);    // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_init_quad(ENCODER_2, ENCODER_2_A, ENCODER_2_B);    // 初始化编码器模块与引脚 正交解码编码器模式
	clock_init(SYSTEM_CLOCK_120M);                          // 初始化芯片时钟 工作频率为 120MHz
    debug_init(); 	// 初始化默认 Debug UART
	tsl1401_init ();
	//轮子驱动初始化	
	gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);        // GPIO 初始化为输出 默认上拉输出高
	pwm_init(PWM_L, 17000, 0);								// PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);        // GPIO 初始化为输出 默认上拉输出高
	pwm_init(PWM_R, 17000, 0);							    // PWM 通道初始化频率 17KHz 占空比初始为 0
	ips200_debug_init ();                                    //显示屏Debug初始化
	ips200_init(IPS200_TYPE_PARALLEL8);                      //显示屏模式设定
//	gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);	          //上拉LED1
//	gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);
	//clock_init(SYSTEM_CLOCK_120M);//初始化系统时钟频率
	interrupt_set_priority(TIM6_IRQn, 0);//中断优先级0级
//	timer_clock_enable(TIM_6);
//	timer_start(TIM_6, 1);//定时器6开始计时，满载10ms
//	timer_get (TIM_1);
	
//	RCC_EnableAPB1Periphs(1,ENABLE);//使能APB1总线
	
	
//摄像头初始化		
	while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0,230,"mt9v03x reinit.");
        else
            break;
        system_delay_ms(1);                                                   // 短延时快速闪灯表示异常
    }	
	    
//陀螺仪初始化	
	while(1)
    {
        if(icm20602_init())
            printf("\r\nICM20602 init error.");                                 // ICM20602 初始化失败
        else
            break;
        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
	//ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
//	  pit_init_ms(TIM7_PIT, 10);//编码器定时器中断开启
//	  system_delay_ms(1);//设置固定延时错开数据提取
	  //First();
      pit_init_ms(TIM8_PIT, 10);//陀螺仪定时器中断开启
	  //pit_init_ms(TIM7_PIT, 20);
//      system_delay_ms(1);//设置固定延时错开数据提取	
	  pit_init_ms(TIM6_PIT, 20);//主程序定时器中断开启
	
//************************主循环********************//
	while(1)
    {
		
//摄像头捕获		
//		if(mt9v03x_finish_flag)
//      {
//            ips200_displayimage032((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
////			ips200_displayimage7725((const uint8 *)mt9v03x_image,MT9V03X_W, MT9V03X_H);
//		}
////        SXTpoint();//判断有无光点
////		SXTPP();//计算光点位置
		//ENDpoint();

		//ENDPoint();
		//SXTpointEE();//光点寻找
    }

}

// **************************** 代码区域 ****************************

