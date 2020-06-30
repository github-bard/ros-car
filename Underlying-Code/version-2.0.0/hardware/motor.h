#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "pid.h"

/********************* 电机A **********************/

#define IN1H GPIO_SetBits(GPIOC,GPIO_Pin_0)   //PC0
#define IN2H GPIO_SetBits(GPIOC,GPIO_Pin_1)   //PC1

/********************* 电机B **********************/

#define IN3H GPIO_SetBits(GPIOC,GPIO_Pin_2)   //PC2
#define IN4H GPIO_SetBits(GPIOC,GPIO_Pin_3)   //PC3

/********************* 电机C **********************/

#define IN5H GPIO_SetBits(GPIOC,GPIO_Pin_4)   //PC4
#define IN6H GPIO_SetBits(GPIOC,GPIO_Pin_5)   //PC5

/********************* 电机D **********************/

#define IN7H GPIO_SetBits(GPIOD,GPIO_Pin_6)   //PD6
#define IN8H GPIO_SetBits(GPIOD,GPIO_Pin_7)   //PD7


/********************* 电机A **********************/

#define IN1L GPIO_ResetBits(GPIOC,GPIO_Pin_0) //PC0
#define IN2L GPIO_ResetBits(GPIOC,GPIO_Pin_1) //PC1

/********************* 电机B **********************/

#define IN3L GPIO_ResetBits(GPIOC,GPIO_Pin_2) //PC2
#define IN4L GPIO_ResetBits(GPIOC,GPIO_Pin_3) //PC3

/********************* 电机C **********************/

#define IN5L GPIO_ResetBits(GPIOC,GPIO_Pin_4) //PC4
#define IN6L GPIO_ResetBits(GPIOC,GPIO_Pin_5) //PC5

/********************* 电机D **********************/

#define IN7L GPIO_ResetBits(GPIOD,GPIO_Pin_6) //PD6
#define IN8L GPIO_ResetBits(GPIOD,GPIO_Pin_7) //PD7


void RightMovingSpeedW(unsigned int val);     //右前轮方向和速度控制函数
void LeftMovingSpeedW(unsigned int val);      //左前轮方向和速度控制函数
void LeftMovingSpeedW1(unsigned int va1);     //右后轮方向和速度控制函数
void RightMovingSpeedW1(unsigned int val);    //右后轮方向和速度控制函数

void Car_Control(float rightspeed,float leftspeed,float leftspeed1,float rightspeed1);//小车速度转化和控制函数
void Motor_Init(void);

#endif

