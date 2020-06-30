#include "motor.h"

extern struct PID Control_left;//左轮PID参数，适于新电机4096
extern struct PID Control_right;//右轮PID参数，适于新电机4096


void Motor_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//输出频率50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);

}

void LeftMovingSpeedW(unsigned int val)//左轮方向和速度控制函数
{     
    if(val>10000)
    {  
        //GPIO_SetBits(GPIOC, GPIO_Pin_0);	
        //GPIO_ResetBits(GPIOC, GPIO_Pin_1);	
        	IN1H;
	        IN2L;
        Control_left.OwenValue=(val-10000);//PID调节的目标编码数			
    }
    else if(val<10000)
    {  
        
			  //GPIO_SetBits(GPIOC, GPIO_Pin_1);	
        //(GPIOC, GPIO_Pin_0);	
        IN1L;
	      IN2H;
        Control_left.OwenValue=(10000-val);//PID调节的目标编码数	 
    }	
    else
    {
         //GPIO_SetBits(GPIOC, GPIO_Pin_0);	
         //GPIO_SetBits(GPIOC, GPIO_Pin_1);
         	IN1H;
	        IN2H;
          Control_left.OwenValue=0;//PID调节的目标编码数
    }					
}

void RightMovingSpeedW(unsigned int val2)//右轮方向和速度控制函数
{    
    if(val2>10000)
    {  
        /* motor A 正转*/
        GPIO_SetBits(GPIOC, GPIO_Pin_2);	
        GPIO_ResetBits(GPIOC, GPIO_Pin_3); 
        //IN3H;
	      //IN4L;
        Control_right.OwenValue=(val2-10000);//PID调节的目标编码数
    }
    else if(val2<10000)
    {  
        /* motor A 反转*/
         GPIO_SetBits(GPIOC, GPIO_Pin_3);	
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);	
        //IN3L;
	      //IN4H;
        Control_right.OwenValue=(10000-val2);//PID调节的目标编码数	 
    }	
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_2);	
        GPIO_SetBits(GPIOC, GPIO_Pin_3);
        //IN3H;
	      //IN4H;
        Control_right.OwenValue=0;//PID调节的目标编码数
    }												
}

void car_control(float rightspeed,float leftspeed)//小车速度转化和控制函数
{
    float k2=17.179;         //速度转换比例,转/分钟	
    
    //将从串口接收到的速度转换成实际控制小车的速度？还是PWM？
    int right_speed=(int)k2*rightspeed;
    int left_speed=(int)k2*leftspeed;          //为什么要做强制数据类型转换?
    
    RightMovingSpeedW(right_speed+10000);
    LeftMovingSpeedW(left_speed+10000);
}

//void Contact_Init(void)//左右轮方向和速度初始化
//{
//	LeftMovingSpeedW(12000); //电机B
//	RightMovingSpeedW(12000);//电机A	
//}



void M1_STOP(void)//电机1停止转动
 {
	IN1H;
	IN2H;
 }
 void M2_STOP(void)//电机2停止转动
 {
	IN3H;
	IN4H;
 }

// void M3_STOP(void)//电机3停止转动
// {
//	IN5H;
//	IN6H;
// }
// void M4_STOP(void)//电机4停止转动
// {
//	IN7H;
//	IN8H;
// }
 
 void M1_Forward(void)//电机1正转
 {
	IN1H;
	IN2L;
 }
 void M2_Forward(void)//电机2正转
 {
	IN3H;
	IN4L;
 }
// void M3_Forward(void)//电机3正转
// {
//	IN5H;
//	IN6L;
// }
// void M4_Forward(void)//电机4正转
// {
//	IN7H;
//	IN8L;
// }
 
 void M1_Reverse(void)//电机1反转
 {
	 IN1L;
	 IN2H;
 }
 void M2_Reverse(void)//电机2反转
 {
	IN3L;
	IN4H;
 }
// void M3_Reverse(void)//电机3反转
// {
//	IN5L;
//	IN6H;
// }
// void M4_Reverse(void)//电机4反转
// {
//	IN7L;
//	IN8H;
// }

 void Car_WalkStraight(void)//小车直走
 {
	M1_Forward();
	M2_Forward();
//	M3_Reverse();
//	M4_Reverse();
	
 }
 void Car_WalkBack(void)//小车后退
 {
	M1_Reverse();
	M2_Reverse();
//	M3_Forward();
//	M4_Forward();
 }
 
 void Car_TurnLeft90(void)//小车正转
 {
	M1_Reverse();
	M2_Reverse();
//	M3_Forward();
//	M4_Forward(); 
 
 }
 void Car_TurnRight90(void)//小车反转
 {
	M1_Forward();
	M2_Forward();
//	M3_Reverse();
//	M4_Reverse();
 }
 
