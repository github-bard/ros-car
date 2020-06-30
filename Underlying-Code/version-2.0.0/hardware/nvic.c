#include "nvic.h"

/*********************************************************** 函数 ********************************************************/
void NVIC_Config_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;	
    //1、flash  2、优先级分组
  	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);                //将中断矢量放到Flash的0地址
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                //设置优先级配置的模式,第1组:抢占优先级0(0:7),抢占优先级1(0:7),

    /**************************** 使能串口3中断，串口通信用 ***********************/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;					     //USART3全局中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	           //子占优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  		 			     //使能中断
    NVIC_Init(&NVIC_InitStructure);	     	
		  	
	  /**************************** 使能TIM2中断，编码器接收信号用 ***********************/
	  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;                 //电机A码盘中断函数，在encoder.c中定义使用
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	  /**************************** 使能TIM3中断，编码器接收信号用 ***********************/
    NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;                 //电机B码盘中断函数，在encoder.c中定义使用
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
		/************************** 使能TIM1更新中断，编码器接收信号用 *********************/
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	           //C码盘中断函数，在encoder.c中定义使用
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		/************************* 使能TIM8更新中断，编码器接收信号用 **********************/
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;	           //D码盘中断函数，在encoder.c中定义使用
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
		/**************************** 使能TIM6中断，定时读取编码器 ***********************/
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
		/************************** 使能TIM7中断，定时发布里程计信息 **********************/
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		

		
}	

