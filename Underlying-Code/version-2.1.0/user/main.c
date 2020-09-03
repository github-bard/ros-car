#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "motor.h"
#include "key.h"
#include "pwm.h"
#include "encoder.h"
#include "nvic.h"
#include "usart.h"
#include "odometer.h"
#include "pid.h"

/*********************************************** 外部变量引用 ******************************************************************/
extern union recieve_data                             
{
	float recieve_float;                                    
	unsigned char recieve_char[4];
}leftdata,rightdata,leftdata1,rightdata1;


/****************************************************** 主函数 ******************************************************************/
int main()
{
	  /************************ 初始化 **********************/
		delay_init();	
	  PWM_Init();	
		USART3_Init();
		Motor_Init();	  
	  NVIC_Config_Init();  	
	  Encoder_Init();
	  Odometer_Pub_Init(); 
		//KEY_Init();	
	
	  /************************* 主循环 *********************/
		while(1)
		{		
				/************ 串口接收处理函数 **********/
				USART3_Handle();
			
			  /************ 小车四轮控制函数 **********/
				Car_Control(rightdata.recieve_float,leftdata.recieve_float,leftdata1.recieve_float,rightdata1.recieve_float);	      //将接收到的四轮速度赋给小车
		}
}







