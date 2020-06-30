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

	}_encoder;      //������ 
	
//extern _encoder Encoder;

#define SPEED_SAMPLING_TIME  9    // (9+1)*500usec = 5ms  ,200hz
#define SPEED_BUFFER_SIZE 3       //�������ٶȻ��������С
#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))  //200hz��С���ٶȲ���Ƶ��
//#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   ������ģʽ���ò���
	
//#define ENCODER1_TIMER TIM3  // ���B���̲ɼ���ʱ�� TIM3
//#define ENCODER2_TIMER TIM2   // ���A���̲ɼ���ʱ�� TIM2

#define ENCODER1_PPR           (u16)(90)  // ���2��������
#define ENCODER2_PPR           (u16)(90)  // ���1��������
#define ENCODER3_PPR           (u16)(90)  // ���1��������
#define ENCODER4_PPR           (u16)(90)  // ���1��������	
	
static unsigned short int hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//����������ɼ�ʱ����
	
s16 ENC_Calc_Rot_Speed1(void);//������A�ı�����
s16 ENC_Calc_Rot_Speed2(void);//������B�ı�����
s16 ENC_Calc_Rot_Speed3(void);//������C�ı�����
s16 ENC_Calc_Rot_Speed4(void);//������D�ı�����	

void Gain1(void);//���õ��A PID���� 
void Gain2(void);//���õ��B PID����
void Gain3(void);//���õ��C PID���� 
void Gain4(void);//���õ��D PID���� 
	
void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM1_Encoder_Init(void);
void TIM8_Encoder_Init(void);

void ENC_Clear_Speed_Buffer(void);//�ٶȴ洢������
void ENC_Calc_Average_Speed(void);//�������ε����ƽ��������
void Encoder_Init(void);
void TIM6_Config_Init(void);

#endif
