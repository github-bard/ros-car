#include "encoder.h"

/****************************************** �ⲿ���� **********************************************/

/************** ����ִ�б�־λ ****************/
extern u8 main_sta;                                              

/******************** PID���� **********************/
//�ṹ�嶨����pid.h
extern struct PID Control_right;                                   //��ǰ��A��PID����
extern struct PID Control_left;                                    //��ǰ��B��PID����
extern struct PID Control_left1;                                   //�����C��PID����
extern struct PID Control_right1;                                  //�Һ���D��PID����


/******************************************* ���� *************************************************/
unsigned short int sampling_interval = SPEED_SAMPLING_TIME;        //����������ɼ�ʱ����
int two_wheels_diff;                                               //�洢����ת�ٲ�ֵ�����ݸ�PID_Calculate����
u8 Speed_Buffer_Underlabeling = 0;                                 //ȫ�ֱ�����ÿִ��һ��TIM6�жϺ������Լ�һ�Σ�CNT_Buffer������λһ�Σ�ֱ���ﵽ���ֵ����
                                                                   
/************** �������������� *********************/
s32 CNT_Buffer1[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer2[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer3[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer4[CNT_BUFFER_SIZE]={0};

/*********** ����ǰһ�α�������CNTֵ ***************/
s32 previous_cnt1;
s32 previous_cnt2;
s32 previous_cnt3; 
s32 previous_cnt4;

/***** һ��ʱ�����������������ڴ��ݸ���̼� ******/
float Odometer_R_CNT=0;
float Odometer_L_CNT=0;
float	Odometer_L1_CNT=0;
float	Odometer_R1_CNT=0;                                       

/*************** ���ƽ��ת�� **********************/
unsigned int RPM_Value1=0;                                        //���Aƽ��ת�� r/min��PID����
unsigned int RPM_Value2=0;                                        //���Bƽ��ת�� r/min��PID����
unsigned int RPM_Value3=0;                                        //���Cƽ��ת�� r/min��PID����
unsigned int RPM_Value4=0;                                        //���Dƽ��ת�� r/min��PID����

/*************** ��ʱ��������� ********************/
static volatile u16 Encoder_Timer_Overflow1;                      
static volatile u16 Encoder_Timer_Overflow2;                     
static volatile u16 Encoder_Timer_Overflow3;                     
static volatile u16 Encoder_Timer_Overflow4;                     	

/******************** �Ƚ�ֵ�洢 *******************/
float pulse1=0;                                                   //���A ������PID���ں�Ƚ�ֵ�����ڷ��ظ���ʱ��CCR�Ĵ���
float pulse2=0;                                                   
float pulse3=0;                                                   
float pulse4=0;

/************* ���CNT_Buffer�����־λ ************/
static bool Is_First_Measurement1=true;                           //���A����CNT_Buffer1��־λ
static bool Is_First_Measurement2=true;                           //���B����CNT_Buffer2��־λ
static bool Is_First_Measurement3=true;                           //���C����CNT_Buffer3��־λ
static bool Is_First_Measurement4=true;                           //���D����CNT_Buffer4��־λ
 

/********************************************* ���� **********************************************/

/**************** ��������ʼ������ **************/
void Encoder_Init()
{
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();
	TIM1_Encoder_Init();
	TIM8_Encoder_Init();
	TIM6_Config_Init();                                              //��ʱ��6��ʼ����������������ʱ�ɼ�����ȡ
	ENC_Clear_CNT_Buffer();
}

/********************************* ��ʱ��������ģʽ��ʼ�� *****************************/

/**************** ��ʱ��2������ʼ�� *************/
//�õ�����PA15��PB3
void TIM2_Encoder_Init(void)
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  
	/************* �˿���ӳ�� *************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //�˿���ӳ����Ҫ��������ʱ��
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	          //�˿���ӳ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);            //��ʱ��2ʱ��ʹ��
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);        //����JTAG,�о�ûʲô��
	
  /*********** ʹ��PA15���� *************/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/************* ʹ��PB3���� ************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/************ TIM2 �������� ***********/
	TIM_TimeBaseStructure.TIM_Period = 65535;                       //�ƴ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                        //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	/******* ���ñ���������Դ�ͼ��� *******/          
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                        //���������˲���������
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);                                          //����TIM2��ʱ��
 }

/**************** ��ʱ��3������ʼ�� *************/
//�õ�����PA6��PA7
void TIM3_Encoder_Init(void)
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	
	/************* ʹ��PA6��PA7 ************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/************ TIM3 �������� ***********/ 
	TIM_TimeBaseStructure.TIM_Period = 65535;                        //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  /****** ���ñ�����ģʽ����Դ�ͼ��� *****/            
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //���������˲��� ����
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);                                           //����TIM3��ʱ��
 }

/**************** ��ʱ��1������ʼ�� *************/
//�õ�����PA8��PA9
void TIM1_Encoder_Init(void)
{     
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
	/************* ʹ��PA8��PA9 *******************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/**************** TIM1 �������� ***************/
	TIM_TimeBaseStructure.TIM_Period = 65535;                      	 //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   
	
  /********* ���ñ�����ģʽ����Դ�ͼ��� *********/            
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ӳ�䵽TI��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //���������˲��� ����
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM1,0);
	TIM_Cmd(TIM1, ENABLE);                                           //����TIM1��ʱ��
}

/**************** ��ʱ��1������ʼ�� *************/
//�õ�����PC6��PC7
void TIM8_Encoder_Init(void) 
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	
	/************* ʹ��PC6��PC7 *******************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/**************** TIM8 �������� ***************/	
	TIM_TimeBaseStructure.TIM_Period = 65535;                         //�ƴ������������õúúÿ���
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                          //Ԥ��Ƶ��Ϊ�㣬ÿ�����ض����Լ�⵽
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	
  /********* ���ñ�����ģʽ����Դ�ͼ��� *********/            
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                  //ѡ������� IC1ӳ�䵽TI1��
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	    //˫���ز���
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //ӳ�䵽TI��
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                          //���������˲��� ����
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE);                                            //����TIM8��ʱ��
 }

/******************************** ��ʱ���жϴ����� *************************************/

/*************** ��ʱ��2�жϴ����� *****************/
void TIM2_IRQHandler (void)                                         //ִ��TIM2(���A�������ɼ�)�����ж�
{   
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow1!=U16_MAX)                           //������Χ  
    {
        Encoder_Timer_Overflow1++;                                  //�������ۼ�
			  //printf("Encoder_Timer_Overflow1:%d\r\n",Encoder_Timer_Overflow1);           //���ڵ��Ա������ж��Ƿ��������С�������������������
    }
}

/*************** ��ʱ��3�жϴ����� *****************/
void TIM3_IRQHandler (void)                                         //ִ��TIM2(���B�������ɼ�)�����ж�
{  
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow2!=U16_MAX)                           //������Χ    
    { 
        Encoder_Timer_Overflow2++;	                                //�������ۼ�
				//printf("Encoder_Timer_Overflow2:%d\r\n",Encoder_Timer_Overflow2);
    }
}

/*************** ��ʱ��1�жϴ����� *****************/
//�߼���ʱ�����жϲ�ͬ��ͬʱ��ʱ�������ָ�ϸ
void TIM1_UP_IRQHandler(void)                                        //ִ��TIM1(���C�������ɼ�)�����ж�
{  
	
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
    if (Encoder_Timer_Overflow3!=U16_MAX)                            //������Χ    
    {
        Encoder_Timer_Overflow3++;	                                 //�������ۼ�
				//printf("Encoder_Timer_Overflow3:%d\r\n",Encoder_Timer_Overflow3);
    }
} 

/*************** ��ʱ��8�жϴ����� *****************/
void TIM8_UP_IRQHandler(void)                                        //ִ��TIM8(���D�������ɼ�)�����ж�
{  
    TIM_ClearFlag(TIM8,  TIM_FLAG_Update);
    if (Encoder_Timer_Overflow4!=U16_MAX)                            //������Χ    
    {
        Encoder_Timer_Overflow4++;	                                 //�������ۼ�
				//printf("Encoder_Timer_Overflow4:%d\r\n",Encoder_Timer_Overflow4);
    }
}

/********************************************* �������ɼ������� *************************************/

/***************** ��ʱ��6��ʼ������ *****************/
void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=399;                                //��ʱ���� �����ж�ʱ�䣺5ms�ж�һ��
	TIM_TimBaseStructrue.TIM_Prescaler=8;                               //Tout=(ARR+1)*(PSC+1)/Tclk
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;            //���ϼ���
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

/****************** ��ʱ��6�жϺ��� *******************/
void TIM6_IRQHandler(void)                                            //�������ɼ���ת�ټ��㡢PID����
{
	
	//printf("TIM6_IRQHandler is on.\r\n");                             //�����ã�����TIM6IRQHandler�Ƿ���������

	if ( TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET) 
	{						      
        if (sampling_interval!=0)                                     //�������������ʱ����δ��
        {
            sampling_interval--;                                      //��ʼ����						
        }
        else                                                          //�������������ʱ��������
        {
            sampling_interval = SPEED_SAMPLING_TIME;                  //�ָ��������������ʱ����
					  s32 temp_cnt1,temp_cnt2,temp_cnt3,temp_cnt4;              //��ʱ���������ȡ����������������������                                 
           
   					/************************ ��ȡ������ *******************/            
            temp_cnt1=ENC_Delta_CNT1();                               //temp_cnt1�����ȡ��������������������������
            temp_cnt2=ENC_Delta_CNT2();                               
            temp_cnt3=ENC_Delta_CNT3();                               
            temp_cnt4=ENC_Delta_CNT4(); 

					  //printf("temp_cnt:%d\r\n",temp_cnt1);                      //������
					
//          ���Ϊָֹͣ����������ٶ�Ϊ�㣬������ٶȴ洢����ֹǰ���ٶȲ�̫�����С����ת
//          if((temp_cnt2 == 0)&&(temp_cnt1 == 0))
//          {
//          	pulse=pulse1=0;
//          }
             
            /********************** ���������� *********************/           
            Odometer_R_CNT=(float)temp_cnt1;                          //�������ź�����ȫ�ֱ����ķ�ʽ�����odometer����
            Odometer_L_CNT=(float)temp_cnt2;
            Odometer_L1_CNT=(float)temp_cnt3;
					  Odometer_R1_CNT=(float)temp_cnt4;						
            main_sta|=0x02;                                           //ִ�м�����̼����ݲ���

            /***************** �������������������� ****************/           
            CNT_Buffer1[Speed_Buffer_Underlabeling] = temp_cnt1;      //Speed_Buffer_Underlabeling ȫ�ֱ�����ÿִ��һ���жϺ����Լ�һ�Σ�ֱ���ﵽ���ֵ����     
            CNT_Buffer2[Speed_Buffer_Underlabeling] = temp_cnt2;
						CNT_Buffer3[Speed_Buffer_Underlabeling] = temp_cnt3;
						CNT_Buffer4[Speed_Buffer_Underlabeling] = temp_cnt4;
            Speed_Buffer_Underlabeling++;                             //������λ            
           
						/********************* ��������ж� ********************/
            if(Speed_Buffer_Underlabeling >=CNT_BUFFER_SIZE)          //�������ӱ���������������ж�
            {
                Speed_Buffer_Underlabeling=0;                         //�������ӱ������������������
            }
          
            ENC_Average_RPM();                                        //�������ε����ƽ��������
						
						/********************** PID���ڿ��� ********************/
            Gain_PID1();                                            //���Aת��PID���ڿ��� 
            Gain_PID2();                                            //���Bת��PID���ڿ��� 
						Gain_PID3();                                            //���Cת��PID���ڿ��� 
						Gain_PID4();                                            //���Dת��PID���ڿ��� 
        }       
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);                  //�жϱ�־λ����    		 
	}		 
}

/********** ������Aһ��ʱ�����ڵ������� **********/
s16 ENC_Delta_CNT1(void)
{   
	  s32 delta_cnt1;                                                 //֮ǰcnt�͵�ǰcnt�Ĳ�ֵ
    u16 encoder_timer_overflow_sample;                              //��ʱ����TIM2�жϲ�������
    u16 current_cnt1;                                               //��ǰcntֵ��ȡ��previous_cnt��ȫ�ֱ�����current_cnt�Ǿֲ�����

    if (!Is_First_Measurement1)                                     //�������һ�����в�ִ���⣬�Ժ�ִ�и�if�����Σ��ж�CNT_Buffer1�Ƿ�����
    {  
        encoder_timer_overflow_sample=Encoder_Timer_Overflow1; 	
        current_cnt1=TIM2->CNT;
        Encoder_Timer_Overflow1=0;  

        if ( (TIM2->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            //encoder timer down-counting ��ת�������     
            delta_cnt1 = (s32)((encoder_timer_overflow_sample)*(4*ENCODER1_PPR)-(current_cnt1-previous_cnt1));//(current_cnt1 - previous_cnt1 - (encoder_timer_overflow_sample)*(4*ENCODER1_PPR));
        }
        else  
        {
            //encoder timer up-counting ��ת�������
            delta_cnt1 = (s32)(current_cnt1 - previous_cnt1 + (encoder_timer_overflow_sample)*(4*ENCODER1_PPR));
        }		
    } 
    else                                                           //�����һ�����У�ִ�и�else������
    {
        Is_First_Measurement1= false;                              //���A����CNT_Buffer1��־λ
        delta_cnt1 = 0;
        Encoder_Timer_Overflow1=0;                                 //�жϴ�����¼����
    }
    previous_cnt1=TIM2->CNT;  
    return((s16) delta_cnt1);
}

/********** ������Bһ��ʱ�����ڵ������� **********/
s16 ENC_Delta_CNT2(void)
{   
    s32 delta_cnt2;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt2;                                               

    if (!Is_First_Measurement2)                                    //�ж�CNT_Buffer2�Ƿ�����
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow2; 	 //�õ�����ʱ���ڵı�����	
        current_cnt2=TIM3->CNT;
        Encoder_Timer_Overflow2=0;                                 //����������ۼ�   

        if ((TIM3->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting ��ת�������
            delta_cnt2=(s32)((encoder_timer_overflow_sample)*(4*ENCODER2_PPR)-(current_cnt2-previous_cnt2));	
        }
        else  
        {
            //encoder timer up-counting ��ת�������
            delta_cnt2 =(s32)( current_cnt2 - previous_cnt2 + (encoder_timer_overflow_sample)*(4*ENCODER2_PPR) );
        }
    } 
    else
    {
        Is_First_Measurement2=false;                               //���B����CNT_Buffer2��־λ
        delta_cnt2=0;
        Encoder_Timer_Overflow2=0;       
    }
    previous_cnt2=TIM3->CNT;  
    return((s16) delta_cnt2);
}

/********** ������Cһ��ʱ�����ڵ������� **********/
s16 ENC_Delta_CNT3(void)
{   
    s32 delta_cnt3;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt3;

    if (!Is_First_Measurement3)                                    //�ж�CNT_Buffer3�Ƿ�����
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow3; 	 //�õ�����ʱ���ڵı�����	
        current_cnt3=TIM1->CNT;
        Encoder_Timer_Overflow3=0;                                 //����������ۼ�  

        if ((TIM1->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting ��ת�������
            delta_cnt3=(s32)((encoder_timer_overflow_sample)*(4*ENCODER3_PPR)-(current_cnt3-previous_cnt3));
        }
        else  
        {
            //encoder timer up-counting ��ת�������
            delta_cnt3 =(s32)( current_cnt3 - previous_cnt3 + (encoder_timer_overflow_sample)*(4*ENCODER3_PPR) );
        }
    } 
    else
    {
        Is_First_Measurement3=false;                               //���C����CNT_Buffer3��־λ
        delta_cnt3=0;
        Encoder_Timer_Overflow3=0;       
    }
    previous_cnt3=TIM1->CNT;  
    return((s16) delta_cnt3);
}

/********** ������Dһ��ʱ�����ڵ������� **********/
s16 ENC_Delta_CNT4(void) 
{   
    s32 delta_cnt4;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt4;

    if (!Is_First_Measurement4)                                    //�ж�CNT_Buffer4�Ƿ�����
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow4; 	 //�õ�����ʱ���ڵı�����	
        current_cnt4=TIM8->CNT;
        Encoder_Timer_Overflow4=0;                                 //����������ۼ�  

        if ((TIM8->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting ��ת�������
            delta_cnt4=(s32)((encoder_timer_overflow_sample)*(4*ENCODER4_PPR)-(current_cnt4-previous_cnt4));	
        }
        else  
        {
            //encoder timer up-counting ��ת�������
            delta_cnt4 =(s32)(current_cnt4-previous_cnt4+(encoder_timer_overflow_sample)*(4*ENCODER4_PPR));
        }
    } 
    else
    {
        Is_First_Measurement4=false;                               //���D����CNT_Buffer4��־λ
        delta_cnt4=0;
        Encoder_Timer_Overflow4=0;      
    }
    previous_cnt4=TIM8->CNT;  
    return((s16) delta_cnt4);
}

/********************* CNT_Buffer���� ********************/
void ENC_Clear_CNT_Buffer(void)
{   
    u32 i;
    for (i=0;i<CNT_BUFFER_SIZE;i++)
    {
      CNT_Buffer1[i]=0;
			CNT_Buffer2[i]=0;
      CNT_Buffer3[i]=0;
			CNT_Buffer4[i]=0;
    }   
    Is_First_Measurement1=true;                                    
    Is_First_Measurement2=true;                                    
		Is_First_Measurement3=true;                                    
		Is_First_Measurement4=true;                                    
}

/********* ��ȡ�������������ֱ�����ĵ��ƽ��ת�� ********/
void ENC_Average_RPM(void)
{   
	u32 i;
	signed long long wtemp1=0;                                       //�����ڲ���ʱ��������ͬ���躬�岻ͬ
	signed long long wtemp2=0;
	signed long long wtemp3=0;
	signed long long wtemp4=0;
   
	for (i=0;i<CNT_BUFFER_SIZE;i++)                                  //�ۼӻ�������ڵ��ٶ�ֵ
	{
		wtemp1+=CNT_Buffer1[i];                                        //wtemp1ΪCNT_Buffer1�����ܺ�
		wtemp2+=CNT_Buffer2[i]; 
		wtemp3+=CNT_Buffer3[i];
		wtemp4+=CNT_Buffer4[i];
	}
   
	wtemp1/=(CNT_BUFFER_SIZE);                                       //wtemp1Ϊƽ������������λΪ��/s	
	wtemp2/=(CNT_BUFFER_SIZE);
  wtemp3/=(CNT_BUFFER_SIZE);
	wtemp4/=(CNT_BUFFER_SIZE);
    
	wtemp1=(wtemp1*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER1_PPR);    //��ƽ����������λתΪr/min ��rpm�����ʱ�׼��λ
	wtemp2=(wtemp2*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER2_PPR);    //SPEED_SAMPLING_FREQUENCE ����Ƶ��
	wtemp3=(wtemp3*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER3_PPR);    //��SPEED_SAMPLING_TIME����ʱ�京���ϳɵ���������ֵ�ϻ���ת��   
	wtemp4=(wtemp4*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER4_PPR);	   //ENCODER4_PPR ���4��������                                            
	
	RPM_Value1=((s16)(wtemp1));                                      //ƽ��ת�� r/min
	RPM_Value2=((s16)(wtemp2));                                      //RPM_Value1 ȫ�ֱ���
	RPM_Value3=((s16)(wtemp3));                                                
	RPM_Value4=((s16)(wtemp4));                                                
}


/************************* PID���ڲ����ش� *****************************/

/*************** ���A PID���� ****************/
void Gain_PID1(void)
{    
	two_wheels_diff=1*(RPM_Value2-RPM_Value1);                       //ȫ�ֱ������洢����ת�ٲ�ֵ���ڱ������ڲ����ݸ�PID_Calculate����
	pulse1=pulse1+PID_Calculate(&Control_right,RPM_Value1);          //PID����      
	if(pulse1>3600) 
		pulse1=3600;                                                   //pwm��������
	if(pulse1<0) 
		pulse1=0;    
	TIM4->CCR3=pulse1;                                               //���A��ֵPWM
}

/*************** ���B PID���� ****************/
void Gain_PID2(void)
{   
	two_wheels_diff=1*(RPM_Value1-RPM_Value2);                       //ȫ�ֱ������洢����ת�ٲ�ֵ���ڱ������ڲ����ݸ�PID_Calculate����
	pulse2=pulse2+PID_Calculate(&Control_left,RPM_Value2);           //PID���� 
	if(pulse2>3600) 
		pulse2=3600;                                                   //pwm��������
	if(pulse2<0) 
		pulse2=0;	
	TIM4->CCR4=pulse2;                                               //���B��ֵPWM	
}

/*************** ���C PID���� ****************/
void Gain_PID3(void)
{    
	two_wheels_diff=1*(RPM_Value4-RPM_Value3);                       //ȫ�ֱ������洢����ת�ٲ�ֵ���ڱ������ڲ����ݸ�PID_Calculate����
	pulse3=pulse3+PID_Calculate(&Control_left1,RPM_Value3);          //PID����
	if(pulse3>3600) 
		pulse3=3600;                                                   //pwm��������
	if(pulse3<0) 
		pulse3=0;	
	TIM5->CCR3=pulse3;                                               //���C��ֵPWM	
}

/*************** ���D PID���� ****************/
void Gain_PID4(void)
{   
	two_wheels_diff=1*(RPM_Value3-RPM_Value4);                       //ȫ�ֱ������洢����ת�ٲ�ֵ�����ݸ�PID_Calculate����
	pulse4=pulse4+PID_Calculate(&Control_right1,RPM_Value4);         //PID���� 
	if(pulse4>3600) 
		pulse4=3600;                                                   //pwm��������
	if(pulse4<0) 
		pulse4=0;	
	TIM5->CCR4=pulse4;                                               //���D��ֵPWM	
}

