#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "stdbool.h"
#include "pid.h"

#define SPEED_SAMPLING_TIME    9                                          //(9+1)*500usec = 5ms  ,200hz
#define CNT_BUFFER_SIZE        3                                          //�������ٶȻ��������С
#define U16_MAX                ((u16)65535u)
#define U32_MAX                ((u32)4294967295uL)
#define SPEED_SAMPLING_FREQUENCE    (u16)(2000/(SPEED_SAMPLING_TIME+1))   //200hz��С���ٶȲ���Ƶ��
#define ENCODER1_PPR           (u16)(90)                                  //���A��������
#define ENCODER2_PPR           (u16)(90)                                  //���B��������
#define ENCODER3_PPR           (u16)(90)                                  //���C��������
#define ENCODER4_PPR           (u16)(90)                                  //���D��������	
	
s16 ENC_Delta_CNT1(void);                                                 //������Aһ��ʱ�����ڵ�������
s16 ENC_Delta_CNT2(void);                                                 //������Bһ��ʱ�����ڵ�������
s16 ENC_Delta_CNT3(void);                                                 //������Cһ��ʱ�����ڵ�������
s16 ENC_Delta_CNT4(void);                                                 //������Dһ��ʱ�����ڵ�������

void Gain_PID1(void);                                                     //���õ��A PID���� 
void Gain_PID2(void);                                                     //���õ��B PID����
void Gain_PID3(void);                                                     //���õ��C PID���� 
void Gain_PID4(void);                                                     //���õ��D PID���� 
	
void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM1_Encoder_Init(void);
void TIM8_Encoder_Init(void);

void ENC_Clear_CNT_Buffer(void);                                          //CNT_Buffer����
void ENC_Average_RPM(void);                                               //��ȡ�������������ֱ�����ĵ��ƽ��ת��
void Encoder_Init(void);
void TIM6_Config_Init(void);                                              //�������ɼ���ת�ټ��㡢PID����

#endif
