#include "nvic.h"

/*********************************************************** ���� ********************************************************/
void NVIC_Config_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;	
    //1��flash  2�����ȼ�����
  	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);                //���ж�ʸ���ŵ�Flash��0��ַ
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                //�������ȼ����õ�ģʽ,��1��:��ռ���ȼ�0(0:7),��ռ���ȼ�1(0:7),

    /**************************** ʹ�ܴ���3�жϣ�����ͨ���� ***********************/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;					     //USART3ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	     //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	           //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  		 			     //ʹ���ж�
    NVIC_Init(&NVIC_InitStructure);	     	
		  	
	  /**************************** ʹ��TIM2�жϣ������������ź��� ***********************/
	  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;                 //���A�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	  /**************************** ʹ��TIM3�жϣ������������ź��� ***********************/
    NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;                 //���B�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
		/************************** ʹ��TIM1�����жϣ������������ź��� *********************/
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	           //C�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		/************************* ʹ��TIM8�����жϣ������������ź��� **********************/
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;	           //D�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
		/**************************** ʹ��TIM6�жϣ���ʱ��ȡ������ ***********************/
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
		/************************** ʹ��TIM7�жϣ���ʱ������̼���Ϣ **********************/
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		

		
}	
