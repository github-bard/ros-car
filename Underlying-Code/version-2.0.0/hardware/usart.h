#ifndef __USART3_H
#define	__USART3_H

#include "stm32f10x.h"
#include <stdio.h>

#define USART_REC_LEN  			200  	//�����������ֽ��� 200

void USART3_Init(void);
void USART3_Handle(void);

#endif
