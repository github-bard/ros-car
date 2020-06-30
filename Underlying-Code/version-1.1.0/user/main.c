#include "stm32f10x.h"
#include "motor.h"
#include "key.h"
#include "pwm.h"
#include "encoder.h"
#include "nvic.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "odometry.h"
#include "pid.h"
/***********************************************  ���  *****************************************************************/

char odometry_data[21]={0};   //���͸����ڵ���̼���������

float odometry_right=0,odometry_left=0;//���ڵõ����������ٶ�

/***********************************************  ����  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //����õ�����̼���ֵ

extern u8 USART_RX_BUF[USART_REC_LEN];     //���ڽ��ջ���,���USART_REC_LEN���ֽ�.
extern u16 USART_RX_STA;                   //���ڽ���״̬���	

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dtʱ���ڵ��������ٶ�,������̼Ƽ���

/***********************************************  ����  *****************************************************************/

u8 main_sta=0; //������������������if��ȥ�������flag��1��ӡ��̼ƣ���2���ü�����̼����ݺ�������3���ڽ��ճɹ�����4���ڽ���ʧ�ܣ�

union recieveData  //���յ�������
{
	float d;    //�������ٶ�
	unsigned char data[4];
}leftdata,rightdata;       //���յ�����������

union odometry  //��̼����ݹ�����
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //Ҫ��������̼����ݣ��ֱ�Ϊ��X��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�


int main()
{

	

	  TIM_PWM_Init();	
		USART3_Config();
	  delay_init();	
		Motor_Init();
		//KEY_Init();	  
	  NVIC_Config_Init();
  	
	  Encoder_Init();
	  TIM7_Config_Init();
	
	  u8 i=0,j=0,m=0,t=0;
		//u16 pwmval0=200;      //��¼ʱ�����´�hex�ļ�    
		//u16 pwmval1=200;

		//Car_WalkStraight();
	  //int count=1;

		while(1)
		{
			//printf("qwqeq");
					
				//GPIO_SetBits(GPIOB,GPIO_Pin_5);//�ߵ�ƽ 5
				//GPIO_SetBits(GPIOE,GPIO_Pin_5);//E 5
			
			//if(count)
				//{
				//		car_control(0 ,0);
				//	  count=0;
				//}
				//else
			//	{
				//		car_control(200,200);
				//}
				//printf("main_sta = %d \r\n",main_sta);
				//delay_ms(10);
				//TIM_SetCompare3(TIM4,pwmval0);
				//TIM_SetCompare4(TIM4,200);

				//if(KEY0==1)				  //��������bug���ѽ��
				//{
				//	delay_ms(10000);
				//	if(KEY0==1)
				//		pwmval1+=50;
				//}
//			else if(WK_UP==1&&pwmval1>0) 
//			{
//				delay_ms(10);	
//				if(WK_UP==0)
//					pwmval1-=50;
//			}
				//printf("pwmval1=%d\r\n",pwmval1);
			
				if(main_sta&0x01)//ִ�з�����̼����ݲ���
				{
            //��̼����ݻ�ȡ
						x_data.odoemtry_float=position_x;//��λmm
						y_data.odoemtry_float=position_y;//��λmm
						theta_data.odoemtry_float=oriention;//��λrad
						vel_linear.odoemtry_float=velocity_linear;//��λmm/s
						vel_angular.odoemtry_float=velocity_angular;//��λrad/s
            
            //��������̼����ݴ浽Ҫ���͵�����
						for(j=0;j<4;j++)
						{
								odometry_data[j]=x_data.odometry_char[j];
								odometry_data[j+4]=y_data.odometry_char[j];
								odometry_data[j+8]=theta_data.odometry_char[j];
								odometry_data[j+12]=vel_linear.odometry_char[j];
								odometry_data[j+16]=vel_angular.odometry_char[j];
						}      
						odometry_data[20]='\n';//��ӽ�����
						//printf("odometry_data: %c\r\n",odometry_data[2]);      
						//��������Ҫ����
						for(i=0;i<21;i++)
						{
								USART_ClearFlag(USART3,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
								USART_SendData(USART3,odometry_data[i]);//����һ���ֽڵ�����	
								while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	//�ȴ����ͽ���			
						}
            
						main_sta&=0xFE;//ִ�м�����̼����ݲ���
				}
				
				if(main_sta&0x02)//ִ�м�����̼����ݲ���
				{
						odometry(Milemeter_R_Motor,Milemeter_L_Motor);//������̼�        
						main_sta&=0xFD;//ִ�з�����̼����ݲ���
				} 
				
				if(main_sta&0x08)        //������ָ��û����ȷ����ʱ
				{
						USART_ClearFlag(USART3,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����
            for(m=0;m<3;m++)
            {
                USART_SendData(USART3,0x00);	
                while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            }		
            USART_SendData(USART3,'\n');	
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
            main_sta&=0xF7;
				}
				
				if(USART_RX_STA&0x8000)  // ����1���պ���
				{			
            //�����������ٶ�
            for(t=0;t<4;t++)
            {
                rightdata.data[t]=USART_RX_BUF[t];
                leftdata.data[t]=USART_RX_BUF[t+4];
            }
            
            //�����������ٶ�
						rightdata.d=rightdata.data[0]*100+rightdata.data[1]*10+rightdata.data[2]+rightdata.data[3]*0.1;
						leftdata.d=leftdata.data[0]*100+leftdata.data[1]*10+leftdata.data[2]+leftdata.data[3]*0.1;;
            odometry_right=rightdata.d;//��λmm/s
            odometry_left=leftdata.d;//��λmm/s            
						USART_RX_STA=0;//������ձ�־λ
					  
				}
			//car_control(200,200);
    car_control(rightdata.d,leftdata.d,rightdata.d,leftdata.d);	 //�����յ����������ٶȸ���С��	rightdata.d  leftdata.d
		}//end_while
}//end main
/*********************************************END OF FILE**************************************************/

