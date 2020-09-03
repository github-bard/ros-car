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

/*********************************************** �ⲿ�������� ******************************************************************/
extern union recieve_data                             
{
	float recieve_float;                                    
	unsigned char recieve_char[4];
}leftdata,rightdata,leftdata1,rightdata1;


/****************************************************** ������ ******************************************************************/
int main()
{
	  /************************ ��ʼ�� **********************/
		delay_init();	
	  PWM_Init();	
		USART3_Init();
		Motor_Init();	  
	  NVIC_Config_Init();  	
	  Encoder_Init();
	  Odometer_Pub_Init(); 
		//KEY_Init();	
	
	  /************************* ��ѭ�� *********************/
		while(1)
		{		
				/************ ���ڽ��մ����� **********/
				USART3_Handle();
			
			  /************ С�����ֿ��ƺ��� **********/
				Car_Control(rightdata.recieve_float,leftdata.recieve_float,leftdata1.recieve_float,rightdata1.recieve_float);	      //�����յ��������ٶȸ���С��
		}
}







