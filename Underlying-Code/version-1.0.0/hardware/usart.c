#include "usart.h"

extern u8 main_sta;//主函数步骤执行标志位

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){};//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif

 /**
  * @brief  USART1 GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */
	//复用？上拉输入？
void USART1_Config(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;   //串口端口配置结构体变量
	USART_InitTypeDef    USART_InitStructure;  //串口参数配置结构体变量

	//第1步：打开GPIO和USART部件的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //打开GPIOA时钟和    复用时钟 没啥影响    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//打开串口复用时钟
	USART_DeInit(USART1);  //复位串口1

	//第2步：将USART1 Tx（发送脚）的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			   //串口1发送脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //输出速度50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   //初始化GPIOA
														  
	//第3步：将USART Rx（接收脚）的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			   //串口1接收脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入            
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   //初始化GPIOA

	//第4步：配置USART1参数
	USART_InitStructure.USART_BaudRate             = 115200;							 //波特率设置：115200
	USART_InitStructure.USART_WordLength           = USART_WordLength_8b;			 //数据位数设置：8位
	USART_InitStructure.USART_StopBits             = USART_StopBits_1;				 //停止位设置：1位
	USART_InitStructure.USART_Parity               = USART_Parity_No;				 //是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl  = USART_HardwareFlowControl_None; //硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;	 //接收与发送都使能
	USART_Init(USART1, &USART_InitStructure);										 //初始化USART1

    //打开发送中断和接收中断(如果需要中断)
	 // USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  // 使能指定的USART1发送中断 ；
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能指定的USART1接收中断 ；

	//第5步：使能 USART1， 配置完毕
	USART_Cmd(USART1, ENABLE);							   //使能 USART1

    //如下语句解决第1个字节无法正确发送出去的问题
    USART_ClearFlag(USART1, USART_FLAG_TC);                //清串口1发送标志
}


u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA=0;   //接收状态标记	
u8 serial_rec=0x31;   //接收串口数据变量

void USART1_IRQHandler(void)//串口中断函数
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //是否接受到数据
    {
		//printf("yes1\r\n");
		serial_rec =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
			//printf("serial_rec:%c\r\n",serial_rec);			
		if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            //printf("yes2\r\n");
					if(USART_RX_STA&0x4000)//接收到了0x0d
            {//printf("yes3\r\n");
							  printf("serial_rec=%d\r\n",serial_rec);
                if(serial_rec==0x0a)
                {//printf("yes4\r\n");
                    if((USART_RX_STA&0x3f)==8)
                    {			//printf("yes5\r\n");				
                        USART_RX_STA|=0x8000;	//接收完成了                       
											  main_sta|=0x04;
                        main_sta&=0xF7;
											  //printf("finish");
                    }
                    else
                    {//printf("yes6\r\n");
                        main_sta|=0x08;
                        main_sta&=0xFB;
                        USART_RX_STA=0;//接收错误,重新开始
                    }
                }
                else 
                {//printf("yes7\r\n");
                    main_sta|=0x08;
                    USART_RX_STA=0;//接收错误,重新开始
                }
            }
            else //还没收到0X0D
            {//	printf("yes8\r\n");
                if(serial_rec==0x0d)
								{
									//printf("yes9\r\n");
									USART_RX_STA|=0x4000;
								}
                else
                {//printf("yes10\r\n");
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
                    USART_RX_STA++;
									//printf("USART_RX_STA=%d\r\n",USART_RX_STA);
									
                    if(USART_RX_STA>(USART_REC_LEN-1))
                    {//printf("yes11\r\n");
											  
                        main_sta|=0x08;
                        USART_RX_STA=0;//接收数据错误,重新开始接收
                    }							
                }		 
            }
        }   		 
    }
}
 

/*********************************************END OF FILE**********************/
