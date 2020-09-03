#ifndef __ODOMETER_H
#define __ODOMETER_H

#include "stm32f10x.h"
#include "usart.h"
#include "encoder.h"
#include <math.h>
#include <stdlib.h>

void odometer(float R_CNT,float L_CNT,float L1_CNT,float R1_CNT);
void TIM7_Config_Init(void);
void Odometer_Pub_Init(void);

#endif
