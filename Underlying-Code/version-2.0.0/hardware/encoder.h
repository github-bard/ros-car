#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "stdbool.h"
#include "pid.h"

#define SPEED_SAMPLING_TIME    9                                          //(9+1)*500usec = 5ms  ,200hz
#define CNT_BUFFER_SIZE        3                                          //左右轮速度缓存数组大小
#define U16_MAX                ((u16)65535u)
#define U32_MAX                ((u32)4294967295uL)
#define SPEED_SAMPLING_FREQUENCE    (u16)(2000/(SPEED_SAMPLING_TIME+1))   //200hz，小车速度采样频率
#define ENCODER1_PPR           (u16)(90)                                  //电机A码盘线数
#define ENCODER2_PPR           (u16)(90)                                  //电机B码盘线数
#define ENCODER3_PPR           (u16)(90)                                  //电机C码盘线数
#define ENCODER4_PPR           (u16)(90)                                  //电机D码盘线数	
	
s16 ENC_Delta_CNT1(void);                                                 //计算电机A一定时间间隔内的脉冲数
s16 ENC_Delta_CNT2(void);                                                 //计算电机B一定时间间隔内的脉冲数
s16 ENC_Delta_CNT3(void);                                                 //计算电机C一定时间间隔内的脉冲数
s16 ENC_Delta_CNT4(void);                                                 //计算电机D一定时间间隔内的脉冲数

void Gain_PID1(void);                                                     //设置电机A PID调节 
void Gain_PID2(void);                                                     //设置电机B PID调节
void Gain_PID3(void);                                                     //设置电机C PID调节 
void Gain_PID4(void);                                                     //设置电机D PID调节 
	
void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM1_Encoder_Init(void);
void TIM8_Encoder_Init(void);

void ENC_Clear_CNT_Buffer(void);                                          //CNT_Buffer清零
void ENC_Average_RPM(void);                                               //读取三次脉冲数，分别求出四电机平均转速
void Encoder_Init(void);
void TIM6_Config_Init(void);                                              //编码数采集、转速计算、PID控制

#endif
