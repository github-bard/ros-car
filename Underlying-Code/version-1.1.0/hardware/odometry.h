#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32f10x.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>

void odometry(float right,float left);//里程计计算函数
void TIM7_Config_Init(void);
#endif
