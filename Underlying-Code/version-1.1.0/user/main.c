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
/***********************************************  输出  *****************************************************************/

char odometry_data[21]={0};   //发送给串口的里程计数据数组

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度

/***********************************************  输入  *****************************************************************/

extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值

extern u8 USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲,最大USART_REC_LEN个字节.
extern u16 USART_RX_STA;                   //串口接收状态标记	

extern float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮速度,用于里程计计算

/***********************************************  变量  *****************************************************************/

u8 main_sta=0; //用作处理主函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）

union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

union odometry  //里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度


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
		//u16 pwmval0=200;      //烧录时需重新打开hex文件    
		//u16 pwmval1=200;

		//Car_WalkStraight();
	  //int count=1;

		while(1)
		{
			//printf("qwqeq");
					
				//GPIO_SetBits(GPIOB,GPIO_Pin_5);//高电平 5
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

				//if(KEY0==1)				  //按键出现bug？已解决
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
			
				if(main_sta&0x01)//执行发送里程计数据步骤
				{
            //里程计数据获取
						x_data.odoemtry_float=position_x;//单位mm
						y_data.odoemtry_float=position_y;//单位mm
						theta_data.odoemtry_float=oriention;//单位rad
						vel_linear.odoemtry_float=velocity_linear;//单位mm/s
						vel_angular.odoemtry_float=velocity_angular;//单位rad/s
            
            //将所有里程计数据存到要发送的数组
						for(j=0;j<4;j++)
						{
								odometry_data[j]=x_data.odometry_char[j];
								odometry_data[j+4]=y_data.odometry_char[j];
								odometry_data[j+8]=theta_data.odometry_char[j];
								odometry_data[j+12]=vel_linear.odometry_char[j];
								odometry_data[j+16]=vel_angular.odometry_char[j];
						}      
						odometry_data[20]='\n';//添加结束符
						//printf("odometry_data: %c\r\n",odometry_data[2]);      
						//发送数据要串口
						for(i=0;i<21;i++)
						{
								USART_ClearFlag(USART3,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
								USART_SendData(USART3,odometry_data[i]);//发送一个字节到串口	
								while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	//等待发送结束			
						}
            
						main_sta&=0xFE;//执行计算里程计数据步骤
				}
				
				if(main_sta&0x02)//执行计算里程计数据步骤
				{
						odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程计        
						main_sta&=0xFD;//执行发送里程计数据步骤
				} 
				
				if(main_sta&0x08)        //当发送指令没有正确接收时
				{
						USART_ClearFlag(USART3,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
            for(m=0;m<3;m++)
            {
                USART_SendData(USART3,0x00);	
                while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            }		
            USART_SendData(USART3,'\n');	
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
            main_sta&=0xF7;
				}
				
				if(USART_RX_STA&0x8000)  // 串口1接收函数
				{			
            //接收左右轮速度
            for(t=0;t<4;t++)
            {
                rightdata.data[t]=USART_RX_BUF[t];
                leftdata.data[t]=USART_RX_BUF[t+4];
            }
            
            //储存左右轮速度
						rightdata.d=rightdata.data[0]*100+rightdata.data[1]*10+rightdata.data[2]+rightdata.data[3]*0.1;
						leftdata.d=leftdata.data[0]*100+leftdata.data[1]*10+leftdata.data[2]+leftdata.data[3]*0.1;;
            odometry_right=rightdata.d;//单位mm/s
            odometry_left=leftdata.d;//单位mm/s            
						USART_RX_STA=0;//清楚接收标志位
					  
				}
			//car_control(200,200);
    car_control(rightdata.d,leftdata.d,rightdata.d,leftdata.d);	 //将接收到的左右轮速度赋给小车	rightdata.d  leftdata.d
		}//end_while
}//end main
/*********************************************END OF FILE**************************************************/

