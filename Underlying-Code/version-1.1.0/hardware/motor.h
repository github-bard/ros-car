#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "pid.h"

#define IN1H GPIO_SetBits(GPIOC,GPIO_Pin_0)//PC0
#define IN2H GPIO_SetBits(GPIOC,GPIO_Pin_1)//PC1
#define IN3H GPIO_SetBits(GPIOC,GPIO_Pin_2)//PC2
#define IN4H GPIO_SetBits(GPIOC,GPIO_Pin_3)//PC3
#define IN5H GPIO_SetBits(GPIOC,GPIO_Pin_4)//PC4
#define IN6H GPIO_SetBits(GPIOC,GPIO_Pin_5)//PC5
#define IN7H GPIO_SetBits(GPIOD,GPIO_Pin_6)//PD6
#define IN8H GPIO_SetBits(GPIOD,GPIO_Pin_7)//PD7
	
#define IN1L GPIO_ResetBits(GPIOC,GPIO_Pin_0)//PC0
#define IN2L GPIO_ResetBits(GPIOC,GPIO_Pin_1)//PC1
#define IN3L GPIO_ResetBits(GPIOC,GPIO_Pin_2)//PC2
#define IN4L GPIO_ResetBits(GPIOC,GPIO_Pin_3)//PC3
#define IN5L GPIO_ResetBits(GPIOC,GPIO_Pin_4)//PC4
#define IN6L GPIO_ResetBits(GPIOC,GPIO_Pin_5)//PC5
#define IN7L GPIO_ResetBits(GPIOD,GPIO_Pin_6)//PD6
#define IN8L GPIO_ResetBits(GPIOD,GPIO_Pin_7)//PD7

void LeftMovingSpeedW(unsigned int val);    //左前轮方向和速度控制函数
void RightMovingSpeedW(unsigned int val2);  //右前轮方向和速度控制函数
void RightMovingSpeedW1(unsigned int val2); //右后轮方向和速度控制函数
void LeftMovingSpeedW1(unsigned int va1);   //右后轮方向和速度控制函数
void car_control(float rightspeed,float leftspeed,float rightspeed1,float leftspeed1);//小车速度转化和控制函数
void Motor_Init(void);

void M1_STOP(void);//电机1停止转动
void M2_STOP(void);//电机2停止转动
void M3_STOP(void);//电机3停止转动
void M4_STOP(void);//电机4停止转动

void M1_Forward(void);//电机1正转
void M2_Forward(void);//电机2正转
void M3_Forward(void);//电机3正转
void M4_Forward(void);//电机4正转

void M1_Reverse(void);//电机1反转
void M2_Reverse(void);//电机2反转
void M3_Reverse(void);//电机3反转
void M4_Reverse(void);//电机4反转

void Car_WalkStraight(void);//小车直走
void Car_WalkBack(void);    //小车后退
void Car_TurnLeft90(void);  //小车正转
void Car_TurnRight90(void); //小车反转

#endif

