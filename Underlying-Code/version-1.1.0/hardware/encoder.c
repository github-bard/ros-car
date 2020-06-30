#include "encoder.h"

//extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[], \
//hSpeed_Buffer3[],hSpeed_Buffer4[];                            //�����ٶȻ�������

extern u8 main_sta;                                               //����������ִ�б�־λ

int span;                                                         //�ɼ��������������ٶȲ�ֵ
u8 bSpeed_Buffer_Index = 0;                                       //�������ӱ��������������
//float A_REMP_PLUS;//���APID���ں��PWMֵ����

s32 hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer3[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer4[SPEED_BUFFER_SIZE]={0};                        //�����ٶȻ�������

s32 hPrevious_angle1;
s32 hPrevious_angle2;
s32 hPrevious_angle3; 
s32 hPrevious_angle4;                                             //�Ƕ�

float Milemeter_L_Motor=0;
float Milemeter_R_Motor=0;
float	Milemeter_L1_Motor=0;
float	Milemeter_R1_Motor=0;                                       //dtʱ���ڵ������ٶ�,������̼Ƽ���

static unsigned int hRot_Speed1;                                  //���Aƽ��ת�ٻ���
static unsigned int hRot_Speed2;                                  //���Bƽ��ת�ٻ���
static unsigned int hRot_Speed3;                                  //���Cƽ��ת�ٻ���
static unsigned int hRot_Speed4;                                  //���Dƽ��ת�ٻ���
	
unsigned int Speed1=0;                                            //���Aƽ��ת�� r/min��PID����
unsigned int Speed2=0;                                            //���Bƽ��ת�� r/min��PID����
unsigned int Speed3=0;                                            //���Cƽ��ת�� r/min��PID����
unsigned int Speed4=0;                                            //���Dƽ��ת�� r/min��PID����

static volatile u16 hEncoder_Timer_Overflow1;                     //���A�������ɼ� 
static volatile u16 hEncoder_Timer_Overflow2;                     //���B�������ɼ�
static volatile u16 hEncoder_Timer_Overflow3;                     //���C�������ɼ� 
static volatile u16 hEncoder_Timer_Overflow4;                     //���D�������ɼ�

struct PID Control_left={0.01,0.1,0.75,0,0,0,0,0,0};            //��ǰ��A��PID�����������µ��4096   �ṹ�嶨����pid.h
struct PID Control_right={0.01,0.1,0.75,0,0,0,0,0,0};            //��ǰ��B��PID�����������µ��4096
struct PID Control_left1={0.01,0.1,0.75,0,0,0,0,0,0};           //�����C��PID�����������µ��4096
struct PID Control_right1={0.01,0.1,0.75,0,0,0,0,0,0};           //�Һ���D��PID�����������µ��4096	
	
float pulse1=0;                                                    //���A PID���ں��PWMֵ����
float pulse2=0;                                                   //���B PID���ں��PWMֵ����
float pulse3=0;                                                   //���C PID���ں��PWMֵ����
float pulse4=0;                                                   //���D PID���ں��PWMֵ����

static bool bIs_First_Measurement1=true;                          //���A������ٶȻ��������־λ
static bool bIs_First_Measurement2=true;                          //���B������ٶȻ��������־λ
static bool bIs_First_Measurement3=true;                          //���C������ٶȻ��������־λ
static bool bIs_First_Measurement4=true;                          //���D������ٶȻ��������־λ
 

void Encoder_Init()
{
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();
	TIM1_Encoder_Init();
	TIM8_Encoder_Init();
	TIM6_Config_Init();
	ENC_Clear_Speed_Buffer();
}

void TIM2_Encoder_Init(void)                                      //PA15,PB3
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //�˿���ӳ��
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	          //�˿���ӳ����Ҫ��������ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //RCC_APB2Periph_AFIO ?���ÿ����ΰ�
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);        //����JTAG,�о�ûʲô��
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- ������������������  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*- TIM2������ģʽ���� -*/
	//TIM_DeInit(TIM2); 
	TIM_TimeBaseStructure.TIM_Period = 65535;                       //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                        //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	/*- ���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� -*/          
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                        //���������˲���������
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);                                          //����TIM2��ʱ��
 }

void TIM3_Encoder_Init(void)                                      //PA6,PA7
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
	TIM_TimeBaseStructure.TIM_Period = 65535;                        //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  /*- ���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� -*/            
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //���������˲��� ����
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);                                           //����TIM3��ʱ��
 }

void TIM1_Encoder_Init(void)                                       //PA8��PA9
{     
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*- TIM3������ģʽ���� -*/
	//TIM_DeInit(TIM8); 
	//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);	
	TIM_TimeBaseStructure.TIM_Period = 65535;                      	 //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   
	
  /*- ���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� -*/            
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //���������˲��� ����
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM1,0);
	TIM_Cmd(TIM1, ENABLE);                                           //����TIM1��ʱ��
}

void TIM8_Encoder_Init(void)                                       //PC6,PC7
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	/*- ������������������ -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*- TIM3������ģʽ���� -*/
	//TIM_DeInit(TIM8); 
	//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);	
	TIM_TimeBaseStructure.TIM_Period = 65535;                         //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                          //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	
   /*- ���ñ�����ģʽ����Դ�ͼ���˫���ؼ�� -*/            
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                  //ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	    //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                          //���������˲��� ����
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE);                                            //����TIM8��ʱ��
 }
 
void TIM2_IRQHandler (void)                                         //ִ��TIM2(���A�������ɼ�)�����ж�
{   
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow1!=U16_MAX)                          //������Χ  
    {
        hEncoder_Timer_Overflow1++;                                 //�������ۼ�
			  printf("hEncoder_Timer_Overflow2=%d\r\n",hEncoder_Timer_Overflow1);
    }
}

void TIM3_IRQHandler (void)                                         //ִ��TIM2(���B�������ɼ�)�����ж�
{  
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2!=U16_MAX)                          //������Χ    
    {
        hEncoder_Timer_Overflow2++;	                                //�������ۼ�
			  printf("hEncoder_Timer_Overflow1=%d\r\n",hEncoder_Timer_Overflow2);
    }
}

void TIM1_UP_IRQHandler(void)                                        //ִ��TIM1(���B�������ɼ�)�����ж�
{  
	
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow3!=U16_MAX)                           //������Χ    
    {
        hEncoder_Timer_Overflow3++;	                                 //�������ۼ�
			  printf("hEncoder_Timer_Overflow3=%d\r\n",hEncoder_Timer_Overflow3);
    }
} 

void TIM8_UP_IRQHandler(void)                                        //ִ��TIM3(���B�������ɼ�)�����ж�
{  
    TIM_ClearFlag(TIM8,  TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow4!=U16_MAX)                           //������Χ    
    {
        hEncoder_Timer_Overflow4++;	                                 //�������ۼ�
			  printf("hEncoder_Timer_Overflow4=%d\r\n",hEncoder_Timer_Overflow4);
    }
}

/* 
	 ����2���ж��Ǽ�������������жϣ�
   ���ڳ�ʼ�����������65535�Σ�
   ������ÿ��5ms�Ͷ�ȡ��һ��CNT��
   Ȼ���ֽ������������ˣ�
   �������������ж��ǲ��ᷢ���ģ� 
   ������ĵ����5ms���ܹ�����>=65535/4��
  �����õ���˫���ؼ�⣩�����塣
*/

void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=4000;                               //��ʱ���� �����ж�ʱ�䣺5ms�ж�һ��
	TIM_TimBaseStructrue.TIM_Prescaler=8;
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;            //���ϼ���
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

void TIM6_IRQHandler(void)                                            //С���ٶȼ��㶨ʱ���жϺ���
{
	if ( TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET) 
	{						      
        if (hSpeedMeas_Timebase_500us!=0)                             //����������ɼ�ʱ����δ��
        {
            hSpeedMeas_Timebase_500us--;                              //��ʼ����	
        }
        else                                                          //����������ɼ�ʱ��������
        {
            s32 wtemp1,wtemp2,wtemp3,wtemp4;
            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;          //�ָ�����������ɼ�ʱ����            
           
   					/************************ 1 ***************************/            
            wtemp1=ENC_Calc_Rot_Speed1();                             //A ��ȡ�ı�����
            wtemp2=ENC_Calc_Rot_Speed2();                             //B ��ȡ�ı�����
            wtemp3=ENC_Calc_Rot_Speed3();                             //C ��ȡ�ı�����
            wtemp4=ENC_Calc_Rot_Speed4();                             //D ��ȡ�ı�����
					
//          ���Ϊָֹͣ����������ٶ�Ϊ�㣬������ٶȴ洢����ֹǰ���ٶȲ�̫�����С����ת
//          if((wtemp2 == 0)&&(wtemp1 == 0))
//          {
//          	pulse=pulse1=0;
//          }
             
            /************************ 2 ***************************/           
            /* ���������������������������̼Ƽ��� */
            Milemeter_L_Motor=(float)wtemp1;                         //����������
            Milemeter_R_Motor=(float)wtemp2;
            Milemeter_L1_Motor=(float)wtemp3;
					  Milemeter_R1_Motor=(float)wtemp4;
						
            main_sta|=0x02;//ִ�м�����̼����ݲ���

            /************************ 3 ***************************/           
            /* ��ʼ�������ӱ����������� */
            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
						hSpeed_Buffer3[bSpeed_Buffer_Index] = wtemp3;
						hSpeed_Buffer4[bSpeed_Buffer_Index] = wtemp4;
            bSpeed_Buffer_Index++;//������λ            
            
						//���������ֱ���������������ж�
            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)             //�������ӱ���������������ж�
            {
                bSpeed_Buffer_Index=0;                              //�������ӱ������������������
            }
            
            /************************ 4 ***************************/            
            ENC_Calc_Average_Speed();                               //�������ε����ƽ��������
            Gain1();                                                //���Aת��PID���ڿ��� 
            Gain2();                                                //���Bת��PID���ڿ��� 
						Gain3();                                                //���Cת��PID���ڿ��� 
						Gain4();                                                //���Dת��PID���ڿ��� 
        }       
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);                  //����жϱ�־λ    		 
	}		 
}

s16 ENC_Calc_Rot_Speed1(void)                                       //������A�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)                                    //���A������ٶȻ�������
    {  
        hEnc_Timer_Overflow_sample_one=hEncoder_Timer_Overflow1; 	
        hCurrent_angle_sample_one=TIM2->CNT;
        hEncoder_Timer_Overflow1=0;
        haux = TIM2->CNT;   

        if ( (TIM2->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting ��ת���ٶȼ���     
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));
        }
        else  
        {
                                                                    //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1+ (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));
        }		
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement1= false;                              //���A������ٶȻ��������־λ
        temp = 0;
        hEncoder_Timer_Overflow1=0;
        haux = TIM2->CNT;       
    }
    hPrevious_angle1=haux;  
    return((s16) temp);
}


s16 ENC_Calc_Rot_Speed2(void)                                       //������B�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)                                    //���B������ٶȻ�������
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2; 	//�õ�����ʱ���ڵı�����	
        hCurrent_angle_sample_one=TIM3->CNT;
        hEncoder_Timer_Overflow2=0;                                 //����������ۼ�
        haux=TIM3->CNT;   

        if ((TIM3->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting ��ת���ٶȼ���
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER2_PPR)-(hCurrent_angle_sample_one-hPrevious_angle2));	
        }
        else  
        {
            //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle2+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER2_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement2=false;                               //���B������ٶȻ��������־λ
        temp=0;
        hEncoder_Timer_Overflow2=0;
        haux =TIM3->CNT;       
    }
    hPrevious_angle2=haux;  
    return((s16)temp);
}

s16 ENC_Calc_Rot_Speed3(void)                                       //������B�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement3)                                    //���B������ٶȻ�������
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow3; 	//�õ�����ʱ���ڵı�����	
        hCurrent_angle_sample_one=TIM1->CNT;
        hEncoder_Timer_Overflow3=0;                                 //����������ۼ�
        haux=TIM1->CNT;   

        if ((TIM1->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting ��ת���ٶȼ���
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER3_PPR)-(hCurrent_angle_sample_one-hPrevious_angle3));	
        }
        else  
        {
            //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle3+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER3_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement3=false;                               //���B������ٶȻ��������־λ
        temp=0;
        hEncoder_Timer_Overflow3=0;
        haux =TIM1->CNT;       
    }
    hPrevious_angle3=haux;  
    return((s16)temp);
}

s16 ENC_Calc_Rot_Speed4(void)                                       //������B�ı�����
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement4)                                    //���B������ٶȻ�������
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow4; 	//�õ�����ʱ���ڵı�����	
        hCurrent_angle_sample_one=TIM8->CNT;
        hEncoder_Timer_Overflow4=0;                                 //����������ۼ�
        haux=TIM8->CNT;   

        if ((TIM8->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting ��ת���ٶȼ���
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER4_PPR)-(hCurrent_angle_sample_one-hPrevious_angle4));	
        }
        else  
        {
            //encoder timer up-counting ��ת���ٶȼ���
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle4+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER4_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement4=false;                               //���B������ٶȻ��������־λ
        temp=0;
        hEncoder_Timer_Overflow4=0;
        haux =TIM8->CNT;       
    }
    hPrevious_angle4=haux;  
    return((s16)temp);
}

void ENC_Clear_Speed_Buffer(void)                                   //�ٶȴ洢������
{   
    u32 i;

    //����������ٶȻ�������
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
      hSpeed_Buffer1[i]=0;
			hSpeed_Buffer2[i]=0;
      hSpeed_Buffer3[i]=0;
			hSpeed_Buffer4[i]=0;
    }
    
    bIs_First_Measurement1=true;                                    //���A������ٶȻ��������־λ
    bIs_First_Measurement2=true;                                    //���B������ٶȻ��������־λ
		bIs_First_Measurement2=true;                                    //���B������ٶȻ��������־λ
		bIs_First_Measurement2=true;                                    //���B������ٶȻ��������־λ
}

void ENC_Calc_Average_Speed(void)                                   //�������ε����ƽ��������
{   
	u32 i;
	signed long long wtemp1=0;
	signed long long wtemp2=0;
	signed long long wtemp3=0;
	signed long long wtemp4=0;
   
	for (i=0;i<SPEED_BUFFER_SIZE;i++)                                 //�ۼӻ�������ڵ��ٶ�ֵ
	{
		wtemp1+=hSpeed_Buffer1[i];
		wtemp2+=hSpeed_Buffer2[i];
		wtemp3+=hSpeed_Buffer3[i];
		wtemp4+=hSpeed_Buffer4[i];
	}
   
	wtemp1/=(SPEED_BUFFER_SIZE);                                      //ȡƽ����ƽ����������λΪ ��/s	
	wtemp2/=(SPEED_BUFFER_SIZE);                                      //ƽ�������� ��/s	
  wtemp3/=(SPEED_BUFFER_SIZE);
	wtemp4/=(SPEED_BUFFER_SIZE);
    
	wtemp1=(wtemp1*SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);          //��ƽ����������λתΪ r/min
	wtemp2=(wtemp2*SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR);
	wtemp3=(wtemp3*SPEED_SAMPLING_FREQ)*60/(4*ENCODER3_PPR);
	wtemp4=(wtemp4*SPEED_SAMPLING_FREQ)*60/(4*ENCODER4_PPR);	
	
	hRot_Speed1=((s16)(wtemp1));                                      //ƽ��ת�� r/min
	hRot_Speed2=((s16)(wtemp2));                                      //ƽ��ת�� r/min
	hRot_Speed3=((s16)(wtemp3));                                      //ƽ��ת�� r/min
	hRot_Speed4=((s16)(wtemp4));                                      //ƽ��ת�� r/min
	
	Speed1=hRot_Speed1;                                               //ƽ��ת�� r/min
	Speed2=hRot_Speed2;                                               //ƽ��ת�� r/min
	Speed3=hRot_Speed3;                                               //ƽ��ת�� r/min
	Speed4=hRot_Speed4;                                               //ƽ��ת�� r/min
}

void Gain1(void)                                                    //���õ��A PID����
{
	//static float pulse = 0;    
	span=1*(Speed2-Speed1);                                           //�ɼ��������������ٶȲ�ֵ
	pulse1=pulse1+PID_calculate(&Control_left,hRot_Speed1);            //PID����      
	if(pulse1>3600) 
		pulse1=3600;                                                     //pwm��������
	if(pulse1<0) 
		pulse1=0;    
	TIM4->CCR3=pulse1;                                                 //���A��ֵPWM
	//A_REMP_PLUS=pulse;                                              //���APID���ں��PWMֵ����
}

void Gain2(void)                                                    //���õ��B PID����
{
	//static float pulse1 = 0;    
	span=1*(Speed1-Speed2);                                           //�ɼ��������������ٶȲ�ֵ
	pulse2=pulse2+PID_calculate(&Control_right,hRot_Speed2);           //PID���� 
	if(pulse2>3600) 
		pulse2=3600;                                                    //pwm ��������
	if(pulse2<0) 
		pulse2=0;	
	TIM4->CCR4=pulse2;//���B��ֵPWM	
	//TIM2->CCR3 = A_REMP_PLUS;//���A��ֵPWM
}

void Gain3(void)                                                    //���õ��C PID����
{
	//static float pulse1 = 0;    
	span=1*(Speed4-Speed3);                                           //�ɼ��������������ٶȲ�ֵ
	pulse3=pulse3+PID_calculate(&Control_right1,hRot_Speed3);           //PID���� 
	if(pulse3>3600) 
		pulse3=3600;                                                    //pwm ��������
	if(pulse3<0) 
		pulse3=0;	
	TIM5->CCR3=pulse3;//���B��ֵPWM	
	//TIM2->CCR3 = A_REMP_PLUS;//���A��ֵPWM
}

void Gain4(void)                                                    //���õ��D PID����
{
	//static float pulse1 = 0;    
	span=1*(Speed3-Speed4);                                           //�ɼ��������������ٶȲ�ֵ
	pulse4=pulse4+PID_calculate(&Control_left1,hRot_Speed4);           //PID���� 
	if(pulse4>3600) 
		pulse4=3600;                                                    //pwm ��������
	if(pulse4<0) 
		pulse4=0;	
	TIM5->CCR4=pulse4;//���B��ֵPWM	
	//TIM2->CCR3 = A_REMP_PLUS;//���A��ֵPWM
}

