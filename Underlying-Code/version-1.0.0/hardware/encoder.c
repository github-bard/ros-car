#include "encoder.h"

//_encoder Encoder; //ȫ�ֱ���  �汾1
struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0};//����PID�����������µ��4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//����PID�����������µ��4096
extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];//�������ٶȻ�������

u8 bSpeed_Buffer_Index = 0;//���������ֱ��������������
extern float pulse;//���A PID���ں��PWMֵ����
extern float pulse1;//���B PID���ں��PWMֵ����
extern u8 main_sta;//����������ִ�б�־λ
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dtʱ���ڵ��������ٶ�,������̼Ƽ���


s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};//�������ٶȻ�������
static unsigned int hRot_Speed2;//���Aƽ��ת�ٻ���
static unsigned int hRot_Speed1;//���Bƽ��ת�ٻ���
unsigned int Speed2=0; //���Aƽ��ת�� r/min��PID����
unsigned int Speed1=0; //���Bƽ��ת�� r/min��PID����

static volatile u16 hEncoder_Timer_Overflow1;//���B�������ɼ� 
static volatile u16 hEncoder_Timer_Overflow2;//���A�������ɼ�

//float A_REMP_PLUS;//���APID���ں��PWMֵ����
float pulse = 0;//���A PID���ں��PWMֵ����
float pulse1 = 0;//���B PID���ں��PWMֵ����

int span;//�ɼ��������������ٶȲ�ֵ

static bool bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
static bool bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ

s32 hPrevious_angle2, hPrevious_angle1;

void Encoder_Init()
{
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();
	TIM6_Config_Init();
	ENC_Clear_Speed_Buffer();
}

void TIM2_Encoder_Init(void)//PA15,PB3
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	    //�˿���ӳ����Ҫ��������ʱ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //RCC_APB2Periph_AFIO 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);     //����JTAG,�о�ûʲô��
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- ������������������  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/*- ������������������  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*- TIM2������ģʽ���� -*/
	//TIM_DeInit(TIM2); 
	TIM_TimeBaseStructure.TIM_Period = 65535;//�ƴ���;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	//���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� ��֪�����Կ������ֲ������н��ܱ�����ģʽ            
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 ���������˲���������
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);   //����TIM2��ʱ��
 }

void TIM3_Encoder_Init(void)//PA6,PA7
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*- TIM3������ģʽ���� -*/
	//TIM_DeInit(TIM3); 
	TIM_TimeBaseStructure.TIM_Period = 65535;//�ƴ���;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              
    //���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� ��֪�����Կ������ֲ������н��ܱ�����ģʽ             
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 ���������˲��� ����
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);   //����TIM3��ʱ��
 }

void TIM2_IRQHandler (void)//ִ��TIM4(���A�������ɼ�)�����ж�
{   
    TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2 != U16_MAX)//������Χ  
    {
        hEncoder_Timer_Overflow2++; //�������ۼ�
			//printf("hEncoder_Timer_Overflow2=%d\r\n",hEncoder_Timer_Overflow2);
    }
}

void TIM3_IRQHandler (void)//ִ��TIM3(���B�������ɼ�)�����ж�
{  
    TIM_ClearFlag(ENCODER1_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow1 != U16_MAX)//������Χ    
    {
        hEncoder_Timer_Overflow1++;	 //�������ۼ�
				//printf("hEncoder_Timer_Overflow1=%d\r\n",hEncoder_Timer_Overflow1);
    }
}

 
 
// void TIM2_IRQHandler(void)
//{	
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	
//	}
//	//TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		
//}

//void TIM3_IRQHandler(void)
//{	
//	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	}
//	//TIM_ClearITPendingBit(TIM3, TIM_IT_Update);		
//}

//����2���ж��Ǽ�������������ж� ���ڳ�ʼ�����������65535�� ������ÿ��5ms�Ͷ�ȡ��һ��CNT Ȼ���ֽ�������������
//�������������ж��ǲ��ᷢ���� ������ĵ����5ms���ܹ�����>=65535/4�����õ���˫���ؼ�⣩�����塣
void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	//TIM6��ʼ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=4000;//��ʱ���� �����ж�ʱ�䣺5ms�ж�һ��
	TIM_TimBaseStructrue.TIM_Prescaler=8;
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;  //���ϼ���
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

//void TIM6_IRQHandler(void)								//����ʱ�䲻��̫���������������˾ͻ��������
//{	

//	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  
//	{     
//		//Ranging(Read_Encoder(2),Read_Encoder(3));	//ÿ��5ms�ж�һ�� ��ȡ��ӦCNT��ֵ,������·��
//		//printf("Encoder.M1Lneth=%f\r\n",Encoder.M1Lneth-55000);//��ӡ�����1��·��
//		//printf("Encoder.M2Lneth=%f\r\n",Encoder.M2Lneth);//��ӡ�����2��·��	
//		//printf("Encoder.TIM3=%d\r\n",Read_Encoder(3)-300);
//		//printf("Encoder.TIM2=%d\r\n",Read_Encoder(2));
//		//printf("Encoder.TIM5=%d\r\n",Read_Encoder(5));

//	}	   
//	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//}

void TIM6_IRQHandler(void)//С���ٶȼ��㶨ʱ���жϺ���
{
	if ( TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET ) 
	{						      
        if (hSpeedMeas_Timebase_500us !=0)//����������ɼ�ʱ����δ��
        {
            hSpeedMeas_Timebase_500us--;//��ʼ����	
        }
        else    //����������ɼ�ʱ��������
        {
            s32 wtemp2,wtemp1;
           // printf("tim6 is on\r\n");
            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//�ָ�����������ɼ�ʱ����
            
            /************************ 1 ***************************/
            
            wtemp2 = ENC_Calc_Rot_Speed2(); //A ��ȡ�ı�����
            wtemp1 = ENC_Calc_Rot_Speed1(); //B ��ȡ�ı�����
            
//            //���Ϊָֹͣ����������ٶ�Ϊ�㣬������ٶȴ洢����ֹǰ���ٶȲ�̫�����С����ת
//            if((wtemp2 == 0) && (wtemp1 == 0))
//            {
//                pulse=pulse1=0;
//            }
             
            /************************ 2 ***************************/
            
            //���������������������������̼Ƽ���
            Milemeter_L_Motor= (float)wtemp1; //����������
            Milemeter_R_Motor= (float)wtemp2;
            
            main_sta|=0x02;//ִ�м�����̼����ݲ���

            /************************ 3 ***************************/
            
            //��ʼ���������ֱ�����������
            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
            bSpeed_Buffer_Index++;//������λ
            
            //���������ֱ���������������ж�
            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
            {
                bSpeed_Buffer_Index=0;//���������ֱ������������������
            }
            
            /************************ 4 ***************************/
            
            ENC_Calc_Average_Speed();//�������ε����ƽ��������
            Gain2(); //���Aת��PID���ڿ��� ��
            Gain1(); //���Bת��PID���ڿ��� ��
        }
        
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);//����жϱ�־λ    		 
	}		 
}


// /**************************************************************************
//�������ܣ���λʱ���ȡ����������,�˺����ڶ�ʱ���ж��������ҵ��Ƕ�ʱ��6ÿ��5ms��ȡһ��
//��ڲ�������ʱ��
//����  ֵ������ֵ
//**************************************************************************/
//int x=0;
//int Read_Encoder(u8 TIMX)
//{
//   int Encoder_TIM;
//   switch(TIMX)
//	 {
//		 case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
//		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
//		 //case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
//		 //case 5:  Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0;break;	
//		 default:  Encoder_TIM=0;
//	 }
//		return Encoder_TIM;
//}

// /**************************************************************************
//�������ܣ���λ��ຯ��
//��ڲ�����int
//����  ֵ��void
////�ú����������ǵõ���������ڵ�λʱ���ڼ�¼��CNT���������õ��Ƕ�ʱ��6��
////�����Encoder.M1Lneth+��Encoder.M2Lneth+��Encoder.Avg_lenth+��ÿ�ζ�ʱ�Ժ��CNT�����·�̵��ۼ�
////Encoder.M1Lneth�����Լ�����Ľṹ�� 
//**************************************************************************/
//	
//void Ranging(int M1_cnt,int M2_cnt) //M1���cnt M2����cnt
//{
//	if((M1_cnt>=0&&M2_cnt>=0)||(M1_cnt<0&&M2_cnt<0))
//	{	
//		Encoder.M1Lneth+=M1_cnt;//*0.0000744*0.85;//0.0000744*0.85���Ҹ���ʵ��ת����·�̻������ ��ͬ�����Ӳ�һ��
//		Encoder.M2Lneth+=M2_cnt;//*0.0000744*0.85;
//		Encoder.Avg_lenth+=(((M1_cnt+M2_cnt)/2)*0.0000744)*0.85; 
//	}
//}
s16 ENC_Calc_Rot_Speed2(void)//������A�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)//���A������ٶȻ�������
    {  
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2; 	
        hCurrent_angle_sample_one = ENCODER2_TIMER->CNT;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;   

        if ( (ENCODER2_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            // encoder timer down-counting ��ת���ٶȼ���     
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR) -(hCurrent_angle_sample_one - hPrevious_angle2));
        }
        else  
        {
            //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle2 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR));
        }		
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement2 = false;//���A������ٶȻ��������־λ
        temp = 0;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;       
    }
    hPrevious_angle2 = haux;  
    return((s16) temp);
}


s16 ENC_Calc_Rot_Speed1(void)//������B�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)//���B������ٶȻ�������
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow1; 	//�õ�����ʱ���ڵı�����	
        hCurrent_angle_sample_one = ENCODER1_TIMER->CNT;
        hEncoder_Timer_Overflow1 = 0;//����������ۼ�
        haux = ENCODER1_TIMER->CNT;   

        if ( (ENCODER1_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            // encoder timer down-counting ��ת���ٶȼ���
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));	
        }
        else  
        {
            //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement1 = false;//���B������ٶȻ��������־λ
        temp = 0;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;       
    }
    hPrevious_angle1 = haux;  
    return((s16) temp);
}

void ENC_Clear_Speed_Buffer(void)//�ٶȴ洢������
{   
    u32 i;

    //����������ٶȻ�������
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        hSpeed_Buffer2[i] = 0;
        hSpeed_Buffer1[i] = 0;
    }
    
    bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
    bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ
}

void ENC_Calc_Average_Speed(void)//�������ε����ƽ��������
{   
    u32 i;
	signed long long wtemp3=0;
	signed long long wtemp4=0;

    //�ۼӻ�������ڵ��ٶ�ֵ
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp4 += hSpeed_Buffer2[i];
		wtemp3 += hSpeed_Buffer1[i];
	}
    
    //ȡƽ����ƽ����������λΪ ��/s	
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //ƽ�������� ��/s	
    
    //��ƽ����������λתΪ r/min
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR); 
		
	hRot_Speed2= ((s16)(wtemp4));//ƽ��ת�� r/min
	hRot_Speed1= ((s16)(wtemp3));//ƽ��ת�� r/min
	Speed2=hRot_Speed2;//ƽ��ת�� r/min
	Speed1=hRot_Speed1;//ƽ��ת�� r/min
}

void Gain2(void)//���õ��A PID���� PA2
{
	//static float pulse = 0;
    
	span=1*(Speed1-Speed2);//�ɼ��������������ٶȲ�ֵ
	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);//PID����
    
    //pwm��������
	if(pulse > 3600) pulse = 3600;
	if(pulse < 0) pulse = 0;
    
	//A_REMP_PLUS=pulse;//���APID���ں��PWMֵ����
}


void Gain1(void)//���õ��B PID���� PA1
{
	//static float pulse1 = 0;
    
	span=1*(Speed2-Speed1);//�ɼ��������������ٶȲ�ֵ
	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);//PID����
    
    ////pwm ��������
	if(pulse1 > 3600) pulse1 = 3600;
	if(pulse1 < 0) pulse1 = 0;
	
	TIM4->CCR3 = pulse1;//���B��ֵPWM
	//TIM2->CCR3 = A_REMP_PLUS;//���A��ֵPWM
    TIM4->CCR4 = pulse;//���A��ֵPWM
}


