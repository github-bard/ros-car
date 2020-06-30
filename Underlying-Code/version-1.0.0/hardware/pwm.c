#include "pwm.h"

void TIM4_PWM_Init(void)//PWM输出初始化
{
	  GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_9;      // 只用到了定时器的第2、1路输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /****************信号周期0.5ms=(3599+1)/72Mhz*****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                   //当定时器从0计数到3599，即为3600次，为一个定时周期
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                 //设置预分频7，即为9MHz：不预0分频，即为72MHz；
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	 //设置时钟分频系数：不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                     //设置通道3的电平跳变值，输出另外一个占空比的PWM  最低500
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	                 //使能通道3 PA0
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                     //设置通道4的电平跳变值，输出另外一个占空比的PWM  最低500
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);			                   // 使能TIM2重载寄存器ARR
    TIM_Cmd(TIM4, ENABLE);                                     //使能定时器2
}

