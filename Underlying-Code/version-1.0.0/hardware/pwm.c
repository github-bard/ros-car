#include "pwm.h"

void TIM4_PWM_Init(void)//PWM�����ʼ��
{
	  GPIO_InitTypeDef GPIO_InitStructure;      
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_9;      // ֻ�õ��˶�ʱ���ĵ�2��1·���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /****************�ź�����0.5ms=(3599+1)/72Mhz*****************/    	
    TIM_TimeBaseStructure.TIM_Period = 899;                   //����ʱ����0������3599����Ϊ3600�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	                 //����Ԥ��Ƶ7����Ϊ9MHz����Ԥ0��Ƶ����Ϊ72MHz��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	 //����ʱ�ӷ�Ƶϵ��������Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                     //����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM  ���500
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	                 //ʹ��ͨ��3 PA0
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	         //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 100;	                     //����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM  ���500
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);			                   // ʹ��TIM2���ؼĴ���ARR
    TIM_Cmd(TIM4, ENABLE);                                     //ʹ�ܶ�ʱ��2
}

