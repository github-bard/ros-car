#include "nvic.h"

void NVIC_Config_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;	
    //1��flash  2�����ȼ�����
  	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);               //���ж�ʸ���ŵ�Flash��0��ַ
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);               //�������ȼ����õ�ģʽ,��1��:��ռ���ȼ�0(0:7),��ռ���ȼ�1(0:7),

    /****************************ʹ�ܴ���1�жϣ����������ȼ�***********************/
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;					    //USART1ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	    //��ռ���ȼ� 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	          //��ռ���ȼ� 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  		 			    //ʹ���ж�
    NVIC_Init(&NVIC_InitStructure);	     	
		  	
	
	  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;                //B(��)�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;                //A(��)�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
                                                        
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	              //��ȡ������
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	 
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;	              //������̼�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;	          //C�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;	           //D�����жϺ�������encoder.c�ж���ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}	

