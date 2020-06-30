#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "stdbool.h"
#include "pid.h"

typedef struct
	{
		float M1Lneth;
		float M2Lneth;
		float Avg_lenth;

	}_encoder;      //编码器 
	
//extern _encoder Encoder;

#define SPEED_SAMPLING_TIME  9    // (9+1)*500usec = 5ms  ,200hz
#define SPEED_BUFFER_SIZE 3       //左右轮速度缓存数组大小
#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))  //200hz，小车速度采样频率
//#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   编码器模式设置参数
	
//#define ENCODER1_TIMER TIM3  // 电机B码盘采集定时器 TIM3
//#define ENCODER2_TIMER TIM2   // 电机A码盘采集定时器 TIM2

#define ENCODER1_PPR           (u16)(90)  // 电机2码盘线数
#define ENCODER2_PPR           (u16)(90)  // 电机1码盘线数
#define ENCODER3_PPR           (u16)(90)  // 电机1码盘线数
#define ENCODER4_PPR           (u16)(90)  // 电机1码盘线数	
	
static unsigned short int hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//电机编码数采集时间间隔
	
s16 ENC_Calc_Rot_Speed1(void);//计算电机A的编码数
s16 ENC_Calc_Rot_Speed2(void);//计算电机B的编码数
s16 ENC_Calc_Rot_Speed3(void);//计算电机C的编码数
s16 ENC_Calc_Rot_Speed4(void);//计算电机D的编码数	

void Gain1(void);//设置电机A PID调节 
void Gain2(void);//设置电机B PID调节
void Gain3(void);//设置电机C PID调节 
void Gain4(void);//设置电机D PID调节 
	
void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM1_Encoder_Init(void);
void TIM8_Encoder_Init(void);

void ENC_Clear_Speed_Buffer(void);//速度存储器清零
void ENC_Calc_Average_Speed(void);//计算三次电机的平均编码数
void Encoder_Init(void);
void TIM6_Config_Init(void);

#endif
