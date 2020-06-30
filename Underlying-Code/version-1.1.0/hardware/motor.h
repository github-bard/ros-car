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

void LeftMovingSpeedW(unsigned int val);    //��ǰ�ַ�����ٶȿ��ƺ���
void RightMovingSpeedW(unsigned int val2);  //��ǰ�ַ�����ٶȿ��ƺ���
void RightMovingSpeedW1(unsigned int val2); //�Һ��ַ�����ٶȿ��ƺ���
void LeftMovingSpeedW1(unsigned int va1);   //�Һ��ַ�����ٶȿ��ƺ���
void car_control(float rightspeed,float leftspeed,float rightspeed1,float leftspeed1);//С���ٶ�ת���Ϳ��ƺ���
void Motor_Init(void);

void M1_STOP(void);//���1ֹͣת��
void M2_STOP(void);//���2ֹͣת��
void M3_STOP(void);//���3ֹͣת��
void M4_STOP(void);//���4ֹͣת��

void M1_Forward(void);//���1��ת
void M2_Forward(void);//���2��ת
void M3_Forward(void);//���3��ת
void M4_Forward(void);//���4��ת

void M1_Reverse(void);//���1��ת
void M2_Reverse(void);//���2��ת
void M3_Reverse(void);//���3��ת
void M4_Reverse(void);//���4��ת

void Car_WalkStraight(void);//С��ֱ��
void Car_WalkBack(void);    //С������
void Car_TurnLeft90(void);  //С����ת
void Car_TurnRight90(void); //С����ת

#endif

