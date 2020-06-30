#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"

#define KEY0 GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)
#define WK_UP GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
#define KEY0_PRES 	1
#define WKUP_PRES   2
#define ON 1
#define OFF 0
void KEY_Init(void);
u8 KEY_Scan(u8);

#endif
