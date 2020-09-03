#include "led.h"
#include "stm32f10x.h"
void LED_Init(void)
{
//	 GPIO_InitTypeDef GPIO_Inits ;
//	
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
//	
//	 GPIO_Inits.GPIO_Mode=GPIO_Mode_Out_PP;
//	 GPIO_Inits.GPIO_Pin=GPIO_Pin_5;//5
//	 GPIO_Inits.GPIO_Speed=GPIO_Speed_50MHz;
//	 GPIO_Init(GPIOB,&GPIO_Inits);	 
//	 GPIO_SetBits(GPIOB,GPIO_Pin_5);//5
//	
//	 GPIO_Inits.GPIO_Mode=GPIO_Mode_Out_PP;
//	 GPIO_Inits.GPIO_Pin=GPIO_Pin_5;//5
//	 GPIO_Inits.GPIO_Speed=GPIO_Speed_50MHz;
//	 GPIO_Init(GPIOE,&GPIO_Inits);	 
//	 GPIO_SetBits(GPIOE,GPIO_Pin_5);//E 5
	
		GPIO_InitTypeDef GPIO_InitStructure; 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		         
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOG,GPIO_Pin_10);


	


}
