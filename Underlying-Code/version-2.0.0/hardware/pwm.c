#include "pwm.h"

/********************************************************* ���� ************************************************/

/******************************* PWM�����ʼ�� ****************************/
void PWM_Init(void)
{
	  TIM4_PWM_Init();
    TIM5_PWM_Init();
}

/******************************** TIM4 PWMģʽ���� *************************/
//ʹ������PB8��PB9
void TIM4_PWM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);    
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
	  /**************** ����ʹ�� *****************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;                //ֻ�õ��˶�ʱ���ĵ�3��4·���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /**************** �������� *****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                             //����ʱ����0������899����Ϊ900�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                          //����Ԥ��Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	          //����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
	  /************ ͨ��3 PWMģʽ���� ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM  
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	                          //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

		/************ ͨ��4 PWMģʽ���� ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM 
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /************ ��ʱ��4 ʹ�� *****************/
		TIM_ARRPreloadConfig(TIM4, ENABLE);			                            //ʹ��TIM4���ؼĴ���ARR
    TIM_Cmd(TIM4, ENABLE);
}	

/******************************** TIM5 PWMģʽ���� *************************/
//ʹ������PA2��PA3
void TIM5_PWM_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);    
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
		/**************** ����ʹ�� *****************/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                 //ֻ�õ��˶�ʱ���ĵ�3��4·���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                   //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /**************** �������� *****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                             //����ʱ����0������899����Ϊ900�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                          //����Ԥ��Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	          //����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

		/************ ͨ��3 PWMģʽ���� ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM  
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);	                          //ʹ��ͨ��3
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/************ ͨ��4 PWMģʽ���� ************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	                  //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                              //����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM 
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

		/************** ��ʱ��5 ʹ�� ***************/
    TIM_ARRPreloadConfig(TIM5, ENABLE);			                            //ʹ��TIM5���ؼĴ���ARR
    TIM_Cmd(TIM5, ENABLE);                                              //ʹ�ܶ�ʱ��5
}










