#include "motor.h"

/*********************************************** 外部引用 **************************************************/
extern struct PID Control_right;                                               //右前轮PID参数
extern struct PID Control_left;                                                //左前轮PID参数
extern struct PID Control_left1;                                               //左后轮PID参数
extern struct PID Control_right1;                                              //右后轮PID参数


/************************************************** 函数 ***************************************************/

/************************* 电机初始化 ***************************/
void Motor_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*************** 使能引脚PC0、PC1、PC2 ***************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //输出频率50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);

	/*************** 使能引脚PC3、PC4、PC5 ***************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //输出频率50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOC,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	
	/******************* 使能引脚PD6、PD7 ****************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //输出频率50MHZ
	GPIO_Init(GPIOD,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOD,GPIO_Pin_6|GPIO_Pin_7);
}

/****************** 右前轮方向和速度控制函数 *******************/
//已经考虑到电机安装位置不同导致转向不同，需注意电机正负极安装顺序，红前黑后
//电机顺时针转动是电机正转，电机逆时针转动是电机反转
void RightMovingSpeedW(unsigned int val)
{    
    if(val>10000)
    {   
        IN1H;
	      IN2L;
        Control_right.OwenValue=(val-10000);                                   //PID调节的目标编码数
    }
    else if(val<10000)
    {	
        IN1L;
	      IN2H;
        Control_right.OwenValue=(10000-val);                                   //PID调节的目标编码数	 
    }	
    else
    {
        IN1H;
	      IN2H;
        Control_right.OwenValue=0;                                             //PID调节的目标编码数
    }												
}

/****************** 左前轮方向和速度控制函数 *******************/
void LeftMovingSpeedW(unsigned int val)                                        //左前轮方向和速度控制函数
{     
    if(val>10000)
    {  	
        IN3L;
	      IN4H;
        Control_left.OwenValue=(val-10000);                                    //PID调节的目标编码数			
    }
    else if(val<10000)
    {  
        IN3H;
	      IN4L;
        Control_left.OwenValue=(10000-val);                                    //PID调节的目标编码数	 
    }	
    else
    {
        IN3H;
	      IN4H;
        Control_left.OwenValue=0;                                              //PID调节的目标编码数
    }					
}

/****************** 左后轮方向和速度控制函数 *******************/
void LeftMovingSpeedW1(unsigned int val)                                       //左后轮方向和速度控制函数
{     
    if(val>10000)
    {
        IN5L;
	      IN6H;
        Control_left1.OwenValue=(val-10000);                                   //PID调节的目标编码数			
    }
    else if(val<10000)
    {
        IN5H;
	      IN6L;
        Control_left1.OwenValue=(10000-val);                                   //PID调节的目标编码数	 
    }	
    else
    {
        IN5H;
	      IN6H;
        Control_left1.OwenValue=0;                                             //PID调节的目标编码数
    }					
}

/****************** 右后轮方向和速度控制函数 *******************/
void RightMovingSpeedW1(unsigned int val)
{    
    if(val>10000)
    {  
        IN7H;
	      IN8L;
        Control_right1.OwenValue=(val-10000);                                  //PID调节的目标编码数
    }
    else if(val<10000)
    {	
        IN7L;
	      IN8H;
        Control_right1.OwenValue=(10000-val);                                  //PID调节的目标编码数	 
    }	
    else
    {
        IN7H;
	      IN8H;
        Control_right1.OwenValue=0;                                            //PID调节的目标编码数
    }												
}

/****************** 小车速度转化和控制函数 *******************/
void Car_Control(float rightspeed,float leftspeed,float leftspeed1,float rightspeed1)
{
    float k2=17.179;                                                           //速度转换比例,转/分钟	                                                                                 
    int right_speed=(int)k2*rightspeed;                                        //将从串口接收到的是预期的每个轮子的线速度
    int left_speed=(int)k2*leftspeed;																			  	 //为什么要做强制数据类型转换? 
	  int left_speed1=(int)k2*leftspeed1;
	  int right_speed1=(int)k2*rightspeed1; 
	
		RightMovingSpeedW(right_speed+10000);                                      //电机A
    LeftMovingSpeedW(left_speed+10000);                                        //电机B
	  LeftMovingSpeedW1(left_speed1+10000);                                      //电机C
	  RightMovingSpeedW1(right_speed1+10000);                                    //电机D
}
