#include "usart.h"
#include "odometer.h"

/**************************************** 串口专用配置 ********************************************************/

/**************** 标准库需要的支持函数 ****************/
#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 

}; 
FILE __stdout;

/******* 定义_sys_exit()以避免使用半主机模式 **********/   
void _sys_exit(int x) 
{ 
	x = x; 
} 

/****************** 重定义fputc函数 *******************/
int fputc(int ch, FILE *f)
{      
	while((USART3->SR&0X40)==0){};                                                   //循环发送,直到发送完毕   
    USART3->DR = (u8) ch;      
	return ch;
}
#endif

/********************************************************** 外部引用 ************************************************************/

/**************** 里程计数据 *****************/
//odometer.c中定义
extern float position_x;
extern float position_y;
extern float oriention;
extern float velocity_linear;
extern float velocity_angular;

/** 一定时间间隔内脉冲数，用于传递给里程计 ***/
//encoder.c中定义，赋值
extern float Odometer_L_CNT;
extern float Odometer_R_CNT; 
extern float Odometer_L1_CNT;
extern float Odometer_R1_CNT;


/********************************************************** 变量 ***************************************************************/

/*********** 发送给串口的里程计数组 **********/
char odometer_data[21]={0};

/********** 校正里程计用 *********/
float odometer_right=0;
float odometer_left=0;
float odometer_right1=0;
float odometer_left1=0;

/************** 执行标志位 ******************/
u8 main_sta=0;       //用作处理USART3_handle函数各种if，去掉多余的flag（1打印里程计）（2调用计算里程计数据函数）（3串口接收成功）（4串口接收失败）

/************ 串口接收相关变量 **************/
u8  USART_RX_BUF[USART_REC_LEN];                                                    //接收缓冲,最大USART_REC_LEN 200个字节
u16 USART_RX_STA=0;                                                                 //接收状态标记位	
u8  serial_rec=0x31;                                                                //接收串口数据变量

/******************************************************** 共用体 ***************************************************************/

/************ 从串口接收四轮速度 ************/
//需要经过处理，传递给odometer_**
union recieve_data                             
{
	float recieve_float;                                    
	unsigned char recieve_char[4];
}leftdata,rightdata,leftdata1,rightdata1; 

/********* 临时存取里程计数据共用体 *********/
//中转变量，接收里程计数据，处理后传递给odometer_data[21]
union odometer
{
	float odometer_float;
	unsigned char odometer_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为X，Y方向移动的距离，当前角度，线速度，角速度


/************************************************** 函数 *****************************************************/

/******************************* 串口3初始化 ***************************/
//使能PB10、PB11
void USART3_Init(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;                                         //串口端口配置结构体变量
	USART_InitTypeDef    USART_InitStructure;                                        //串口参数配置结构体变量

	/*********** 第1步：打开GPIO和USART部件的时钟 ***********/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	                         //打开串口3时钟
	USART_DeInit(USART3);                                                            //复位串口3

	/* 第2步：将USART3 Tx（发送脚）的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			                                 //串口3发送脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                               //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                               //输出速度50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				                                   //初始化GPIOB
														  
	/* 第3步：将USART Rx（接收脚）的GPIO配置为浮空输入模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			                                 //串口3接收脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	                                   //上拉输入            
	GPIO_Init(GPIOB, &GPIO_InitStructure);				                                   //初始化GPIOB

	/************** 第4步：配置USART3参数 *******************/
	USART_InitStructure.USART_BaudRate           = 115200;							             //波特率设置：115200
	USART_InitStructure.USART_WordLength         = USART_WordLength_8b;			         //数据位数设置：8位
	USART_InitStructure.USART_StopBits           = USART_StopBits_1;				         //停止位设置：1位
	USART_InitStructure.USART_Parity             = USART_Parity_No;				           //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;   //硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode               = USART_Mode_Rx | USART_Mode_Tx;	   //接收与发送都使能
	USART_Init(USART3, &USART_InitStructure);										                     //初始化USART3

  /*********** 打开发送中断和接收中断(如果需要中断) ******/
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);                                  //使能指定的USART3发送中断
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                   //使能指定的USART3接收中断

	/************ 第5步：使能 USART3， 配置完毕 ************/
	USART_Cmd(USART3, ENABLE); 

  /***** 如下语句解决第1个字节无法正确发送出去的问题 *****/
  USART_ClearFlag(USART3, USART_FLAG_TC);                                          //清空串口3发送标志位
}

/******************************** 串口3中断函数 **************************/
//通过串口接收标志位以及数据标志位来判断接收是否完成，同时改变main_sta值执行不同步骤
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                            //判断是否接受到数据
  {
		serial_rec =USART_ReceiveData(USART3);                                         //读取接收到的数据		
		if((USART_RX_STA&0x8000)==0)                                                   //接收未完成
    {
			if(USART_RX_STA&0x4000)                                                      //接收到了0x0D
      {
				if(serial_rec==0x0a)
        {
					if((USART_RX_STA&0x3f)==16)                                              //接收有效字节数16
          {				
            USART_RX_STA|=0x8000;	                                                 //接收完成了                       
						main_sta|=0x04;
						main_sta&=0xF7;
          }
          else
          {
            main_sta|=0x08;
            main_sta&=0xFB;
            USART_RX_STA=0;                                                        //接收错误,重新开始
          }
        }
        else 
        {
          main_sta|=0x08;
					USART_RX_STA=0;                                                          //接收错误,重新开始
        }
      }
      else                                                                         //还没收到0X0D
      {
				if(serial_rec==0x0d)
				{
					USART_RX_STA|=0x4000;
				}
        else
        {
					USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
          USART_RX_STA++;									
          if(USART_RX_STA>(USART_REC_LEN-1))
          { 
						main_sta|=0x08;
						USART_RX_STA=0;                                                        //接收数据错误,重新开始接收
          }							
        }		 
      }
   }   		 
 }
}

/******************************** 串口3接收处理函数 *************************/
void USART3_Handle(void)
{
		/************ 变量 ************/
	  u8 i=0;
		/************ 1、执行发送里程计数据步骤 *****************/
		if(main_sta&0x01)                                          
		{
        /************** 里程计数据获取 *************/
				x_data.odometer_float      = position_x;                 //单位mm
				y_data.odometer_float      = position_y;                 //单位mm
				theta_data.odometer_float  = oriention;                  //单位rad
				vel_linear.odometer_float  = velocity_linear;            //单位mm/s
				vel_angular.odometer_float = velocity_angular;           //单位rad/s
        
		   	//printf("posintion_x:%f\r\n",position_x);
		  	//printf("x_data.odometer_float:%f\r\n",x_data.odometer_float);
			
        /* 将获取的里程计数据以字符格式存到要发送的数组 */
				for(i=0;i<4;i++)
				{
						odometer_data[i]    = x_data.odometer_char[i];
						odometer_data[i+4]  = y_data.odometer_char[i];
						odometer_data[i+8]  = theta_data.odometer_char[i];
						odometer_data[i+12] = vel_linear.odometer_char[i];
						odometer_data[i+16] = vel_angular.odometer_char[i];
				}      
				odometer_data[20]='\n';                                  //添加结束符
				
		    /************** 发送数据到串口 *************/
				for(i=0;i<21;i++)
				{
						USART_ClearFlag(USART3,USART_FLAG_TC);               //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
						USART_SendData(USART3,odometer_data[i]);             //发送一个字节到串口	
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	                           //等待发送结束			
				}           
				main_sta&=0xFE;                                          //将标志位清零
		}
		
		/************** 2、执行计算里程计数据步骤 ***************/
		if(main_sta&0x02)
		{
				odometer(Odometer_R_CNT,Odometer_L_CNT,Odometer_L1_CNT,Odometer_R1_CNT);       //计算里程计        
				main_sta&=0xFD;                                          //将标志位清零
		} 
		
		/************** 3、处理发送指令没有正确接收 ************/
		if(main_sta&0x08)        
		{
				USART_ClearFlag(USART3,USART_FLAG_TC);                   //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题
        for(i=0;i<3;i++)
        {
            USART_SendData(USART3,0x00);	
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        }		
        USART_SendData(USART3,'\n');	
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
        main_sta&=0xF7;                                           //标志位清零
		}
		
		/***************** 4、串口接收函数 **********************/
		if(USART_RX_STA&0x8000)
		{			
        for(i=0;i<4;i++)
        {
				/************** 接收四轮速度 *************/
            rightdata.recieve_char[i]  = USART_RX_BUF[i];
            leftdata.recieve_char[i]   = USART_RX_BUF[i+4];
						leftdata1.recieve_char[i]  = USART_RX_BUF[i+4];
						rightdata1.recieve_char[i] = USART_RX_BUF[i];
        }
        
				/************** 储存四轮速度 *************/                        // ???字符直接乘？
				rightdata.recieve_float  = rightdata.recieve_char[0]*100+rightdata.recieve_char[1]*10+rightdata.recieve_char[2]+rightdata.recieve_char[3]*0.1;
				leftdata.recieve_float   = leftdata.recieve_char[0]*100+leftdata.recieve_char[1]*10+leftdata.recieve_char[2]+leftdata.recieve_char[3]*0.1;
				leftdata1.recieve_float  = leftdata1.recieve_char[0]*100+leftdata1.recieve_char[1]*10+leftdata1.recieve_char[2]+leftdata1.recieve_char[3]*0.1;
				rightdata1.recieve_float = rightdata1.recieve_char[0]*100+rightdata1.recieve_char[1]*10+rightdata1.recieve_char[2]+rightdata1.recieve_char[3]*0.1;
				
        odometer_right  = rightdata.recieve_float;                  //校正里程计用
        odometer_left   = leftdata.recieve_float;         	                   	 
        odometer_left1  = leftdata1.recieve_float;                              
				odometer_right1 = rightdata1.recieve_float;                             
				
				USART_RX_STA=0;                                             //清除接收标志位					  
		}
}












