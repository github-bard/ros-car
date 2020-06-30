#include "pwm.h"

/********************************************************* 函数 ************************************************/

/******************************* PWM输出初始化 ****************************/
void PWM_Init(void)
{
	  TIM4_PWM_Init();
    TIM5_PWM_Init();
}

/******************************** TIM4 PWM模式设置 *************************/
//使能引脚PB8、PB9
void TIM4_PWM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);    
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
	  /**************** 引脚使能 *****************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;                //只用到了定时器的第3、4路输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /**************** 周期设置 *****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                             //当定时器从0计数到899，即为900次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                          //设置预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	          //设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
	  /************ 通道3 PWM模式设置 ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //设置通道3的电平跳变值，输出另外一个占空比的PWM  
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	                          //使能通道3
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

		/************ 通道4 PWM模式设置 ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //设置通道4的电平跳变值，输出另外一个占空比的PWM 
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /************ 定时器4 使能 *****************/
		TIM_ARRPreloadConfig(TIM4, ENABLE);			                            //使能TIM4重载寄存器ARR
    TIM_Cmd(TIM4, ENABLE);
}	

/******************************** TIM5 PWM模式设置 *************************/
//使能引脚PA2、PA3
void TIM5_PWM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
		/**************** 引脚使能 *****************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                 //只用到了定时器的第3、4路输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                   //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /**************** 周期设置 *****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                             //当定时器从0计数到899，即为900次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                          //设置预分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	          //设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

		/************ 通道3 PWM模式设置 ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //设置通道3的电平跳变值，输出另外一个占空比的PWM  
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);	                          //使能通道3
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/************ 通道4 PWM模式设置 ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //设置通道4的电平跳变值，输出另外一个占空比的PWM 
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/************** 定时器5 使能 ***************/
    TIM_ARRPreloadConfig(TIM5, ENABLE);			                            //使能TIM5重载寄存器ARR
    TIM_Cmd(TIM5, ENABLE);                                              //使能定时器5
}










