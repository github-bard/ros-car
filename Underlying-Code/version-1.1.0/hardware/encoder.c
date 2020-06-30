#include "encoder.h"

//extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[], \
//hSpeed_Buffer3[],hSpeed_Buffer4[];                            //轮子速度缓存数组

extern u8 main_sta;                                               //主函数步骤执行标志位

int span;                                                         //采集回来的左右轮速度差值
u8 bSpeed_Buffer_Index = 0;                                       //缓存轮子编码数到数组变量
//float A_REMP_PLUS;//电机APID调节后的PWM值缓存

s32 hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer3[SPEED_BUFFER_SIZE]={0};
s32 hSpeed_Buffer4[SPEED_BUFFER_SIZE]={0};                        //轮子速度缓存数组

s32 hPrevious_angle1;
s32 hPrevious_angle2;
s32 hPrevious_angle3; 
s32 hPrevious_angle4;                                             //角度

float Milemeter_L_Motor=0;
float Milemeter_R_Motor=0;
float	Milemeter_L1_Motor=0;
float	Milemeter_R1_Motor=0;                                       //dt时间内的轮子速度,用于里程计计算

static unsigned int hRot_Speed1;                                  //电机A平均转速缓存
static unsigned int hRot_Speed2;                                  //电机B平均转速缓存
static unsigned int hRot_Speed3;                                  //电机C平均转速缓存
static unsigned int hRot_Speed4;                                  //电机D平均转速缓存
	
unsigned int Speed1=0;                                            //电机A平均转速 r/min，PID调节
unsigned int Speed2=0;                                            //电机B平均转速 r/min，PID调节
unsigned int Speed3=0;                                            //电机C平均转速 r/min，PID调节
unsigned int Speed4=0;                                            //电机D平均转速 r/min，PID调节

static volatile u16 hEncoder_Timer_Overflow1;                     //电机A编码数采集 
static volatile u16 hEncoder_Timer_Overflow2;                     //电机B编码数采集
static volatile u16 hEncoder_Timer_Overflow3;                     //电机C编码数采集 
static volatile u16 hEncoder_Timer_Overflow4;                     //电机D编码数采集

struct PID Control_left={0.01,0.1,0.75,0,0,0,0,0,0};            //左前轮A的PID参数，适于新电机4096   结构体定义在pid.h
struct PID Control_right={0.01,0.1,0.75,0,0,0,0,0,0};            //右前轮B的PID参数，适于新电机4096
struct PID Control_left1={0.01,0.1,0.75,0,0,0,0,0,0};           //左后轮C的PID参数，适于新电机4096
struct PID Control_right1={0.01,0.1,0.75,0,0,0,0,0,0};           //右后轮D的PID参数，适于新电机4096	
	
float pulse1=0;                                                    //电机A PID调节后的PWM值缓存
float pulse2=0;                                                   //电机B PID调节后的PWM值缓存
float pulse3=0;                                                   //电机C PID调节后的PWM值缓存
float pulse4=0;                                                   //电机D PID调节后的PWM值缓存

static bool bIs_First_Measurement1=true;                          //电机A以清除速度缓存数组标志位
static bool bIs_First_Measurement2=true;                          //电机B以清除速度缓存数组标志位
static bool bIs_First_Measurement3=true;                          //电机C以清除速度缓存数组标志位
static bool bIs_First_Measurement4=true;                          //电机D以清除速度缓存数组标志位
 

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
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //端口重映射
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	          //端口重映射需要开启复用时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //RCC_APB2Periph_AFIO ?不用开两次吧
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);        //禁用JTAG,感觉没什么用
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- 正交编码器输入引脚  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*- TIM2编码器模式配置 -*/
	//TIM_DeInit(TIM2); 
	TIM_TimeBaseStructure.TIM_Period = 65535;                       //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                        //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	/*- 配置编码器模式触发源和极性双边沿检测 -*/          
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                        //配置输入滤波器，不滤
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);                                          //启动TIM2定时器
 }

void TIM3_Encoder_Init(void)                                      //PA6,PA7
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*- TIM3编码器模式配置 -*/
	//TIM_DeInit(TIM3); 
	TIM_TimeBaseStructure.TIM_Period = 65535;                        //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  /*- 配置编码器模式触发源和极性双边沿检测 -*/            
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //配置输入滤波器 不滤
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);                                           //启动TIM3定时器
 }

void TIM1_Encoder_Init(void)                                       //PA8、PA9
{     
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*- TIM3编码器模式配置 -*/
	//TIM_DeInit(TIM8); 
	//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);	
	TIM_TimeBaseStructure.TIM_Period = 65535;                      	 //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   
	
  /*- 配置编码器模式触发源和极性双边沿检测 -*/            
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //配置输入滤波器 不滤
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM1,0);
	TIM_Cmd(TIM1, ENABLE);                                           //启动TIM1定时器
}

void TIM8_Encoder_Init(void)                                       //PC6,PC7
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	/*- 正交编码器输入引脚 -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*- TIM3编码器模式配置 -*/
	//TIM_DeInit(TIM8); 
	//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);	
	TIM_TimeBaseStructure.TIM_Period = 65535;                         //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                          //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	
   /*- 配置编码器模式触发源和极性双边沿检测 -*/            
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                  //选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	    //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	            //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                          //配置输入滤波器 不滤
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE);                                            //启动TIM8定时器
 }
 
void TIM2_IRQHandler (void)                                         //执行TIM2(电机A编码器采集)计数中断
{   
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow1!=U16_MAX)                          //不超范围  
    {
        hEncoder_Timer_Overflow1++;                                 //脉冲数累加
			  printf("hEncoder_Timer_Overflow2=%d\r\n",hEncoder_Timer_Overflow1);
    }
}

void TIM3_IRQHandler (void)                                         //执行TIM2(电机B编码器采集)计数中断
{  
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2!=U16_MAX)                          //不超范围    
    {
        hEncoder_Timer_Overflow2++;	                                //脉冲数累加
			  printf("hEncoder_Timer_Overflow1=%d\r\n",hEncoder_Timer_Overflow2);
    }
}

void TIM1_UP_IRQHandler(void)                                        //执行TIM1(电机B编码器采集)计数中断
{  
	
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow3!=U16_MAX)                           //不超范围    
    {
        hEncoder_Timer_Overflow3++;	                                 //脉冲数累加
			  printf("hEncoder_Timer_Overflow3=%d\r\n",hEncoder_Timer_Overflow3);
    }
} 

void TIM8_UP_IRQHandler(void)                                        //执行TIM3(电机B编码器采集)计数中断
{  
    TIM_ClearFlag(TIM8,  TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow4!=U16_MAX)                           //不超范围    
    {
        hEncoder_Timer_Overflow4++;	                                 //脉冲数累加
			  printf("hEncoder_Timer_Overflow4=%d\r\n",hEncoder_Timer_Overflow4);
    }
}

/* 
	 以下2个中断是计数器溢出更新中断，
   我在初始化里面给的是65535次，
   但是我每隔5ms就读取了一次CNT，
   然后又将计数器清零了，
   所以下面两个中断是不会发生的， 
   除非你的电机在5ms内能够产生>=65535/4，
  （我用的是双边沿检测）个脉冲。
*/

void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=4000;                               //暂时设置 计数中断时间：5ms中断一次
	TIM_TimBaseStructrue.TIM_Prescaler=8;
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;            //向上计数
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

void TIM6_IRQHandler(void)                                            //小车速度计算定时器中断函数
{
	if ( TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET) 
	{						      
        if (hSpeedMeas_Timebase_500us!=0)                             //电机编码数采集时间间隔未到
        {
            hSpeedMeas_Timebase_500us--;                              //开始倒数	
        }
        else                                                          //电机编码数采集时间间隔到了
        {
            s32 wtemp1,wtemp2,wtemp3,wtemp4;
            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;          //恢复电机编码数采集时间间隔            
           
   					/************************ 1 ***************************/            
            wtemp1=ENC_Calc_Rot_Speed1();                             //A 获取的编码数
            wtemp2=ENC_Calc_Rot_Speed2();                             //B 获取的编码数
            wtemp3=ENC_Calc_Rot_Speed3();                             //C 获取的编码数
            wtemp4=ENC_Calc_Rot_Speed4();                             //D 获取的编码数
					
//          如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
//          if((wtemp2 == 0)&&(wtemp1 == 0))
//          {
//          	pulse=pulse1=0;
//          }
             
            /************************ 2 ***************************/           
            /* 储存编码数（脉冲数），用于里程计计算 */
            Milemeter_L_Motor=(float)wtemp1;                         //储存脉冲数
            Milemeter_R_Motor=(float)wtemp2;
            Milemeter_L1_Motor=(float)wtemp3;
					  Milemeter_R1_Motor=(float)wtemp4;
						
            main_sta|=0x02;//执行计算里程计数据步骤

            /************************ 3 ***************************/           
            /* 开始缓存轮子编码数到数组 */
            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
						hSpeed_Buffer3[bSpeed_Buffer_Index] = wtemp3;
						hSpeed_Buffer4[bSpeed_Buffer_Index] = wtemp4;
            bSpeed_Buffer_Index++;//数组移位            
            
						//缓存左右轮编码数到数组结束判断
            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)             //缓存轮子编码数到数组结束判断
            {
                bSpeed_Buffer_Index=0;                              //缓存轮子编码数到数组变量清零
            }
            
            /************************ 4 ***************************/            
            ENC_Calc_Average_Speed();                               //计算三次电机的平均编码数
            Gain1();                                                //电机A转速PID调节控制 
            Gain2();                                                //电机B转速PID调节控制 
						Gain3();                                                //电机C转速PID调节控制 
						Gain4();                                                //电机D转速PID调节控制 
        }       
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);                  //清除中断标志位    		 
	}		 
}

s16 ENC_Calc_Rot_Speed1(void)                                       //计算电机A的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)                                    //电机A以清除速度缓存数组
    {  
        hEnc_Timer_Overflow_sample_one=hEncoder_Timer_Overflow1; 	
        hCurrent_angle_sample_one=TIM2->CNT;
        hEncoder_Timer_Overflow1=0;
        haux = TIM2->CNT;   

        if ( (TIM2->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting 反转的速度计算     
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));
        }
        else  
        {
                                                                    //encoder timer up-counting 正转的速度计算
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1+ (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));
        }		
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement1= false;                              //电机A以清除速度缓存数组标志位
        temp = 0;
        hEncoder_Timer_Overflow1=0;
        haux = TIM2->CNT;       
    }
    hPrevious_angle1=haux;  
    return((s16) temp);
}


s16 ENC_Calc_Rot_Speed2(void)                                       //计算电机B的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)                                    //电机B以清除速度缓存数组
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2; 	//得到采样时间内的编码数	
        hCurrent_angle_sample_one=TIM3->CNT;
        hEncoder_Timer_Overflow2=0;                                 //清除脉冲数累加
        haux=TIM3->CNT;   

        if ((TIM3->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting 反转的速度计算
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER2_PPR)-(hCurrent_angle_sample_one-hPrevious_angle2));	
        }
        else  
        {
            //encoder timer up-counting 正转的速度计算
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle2+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER2_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement2=false;                               //电机B以清除速度缓存数组标志位
        temp=0;
        hEncoder_Timer_Overflow2=0;
        haux =TIM3->CNT;       
    }
    hPrevious_angle2=haux;  
    return((s16)temp);
}

s16 ENC_Calc_Rot_Speed3(void)                                       //计算电机B的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement3)                                    //电机B以清除速度缓存数组
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow3; 	//得到采样时间内的编码数	
        hCurrent_angle_sample_one=TIM1->CNT;
        hEncoder_Timer_Overflow3=0;                                 //清除脉冲数累加
        haux=TIM1->CNT;   

        if ((TIM1->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting 反转的速度计算
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER3_PPR)-(hCurrent_angle_sample_one-hPrevious_angle3));	
        }
        else  
        {
            //encoder timer up-counting 正转的速度计算
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle3+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER3_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement3=false;                               //电机B以清除速度缓存数组标志位
        temp=0;
        hEncoder_Timer_Overflow3=0;
        haux =TIM1->CNT;       
    }
    hPrevious_angle3=haux;  
    return((s16)temp);
}

s16 ENC_Calc_Rot_Speed4(void)                                       //计算电机B的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement4)                                    //电机B以清除速度缓存数组
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow4; 	//得到采样时间内的编码数	
        hCurrent_angle_sample_one=TIM8->CNT;
        hEncoder_Timer_Overflow4=0;                                 //清除脉冲数累加
        haux=TIM8->CNT;   

        if ((TIM8->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
                                                                    // encoder timer down-counting 反转的速度计算
            wDelta_angle=(s32)((hEnc_Timer_Overflow_sample_one)*(4*ENCODER4_PPR)-(hCurrent_angle_sample_one-hPrevious_angle4));	
        }
        else  
        {
            //encoder timer up-counting 正转的速度计算
            wDelta_angle =(s32)(hCurrent_angle_sample_one-hPrevious_angle4+(hEnc_Timer_Overflow_sample_one)*(4*ENCODER4_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement4=false;                               //电机B以清除速度缓存数组标志位
        temp=0;
        hEncoder_Timer_Overflow4=0;
        haux =TIM8->CNT;       
    }
    hPrevious_angle4=haux;  
    return((s16)temp);
}

void ENC_Clear_Speed_Buffer(void)                                   //速度存储器清零
{   
    u32 i;

    //清除左右轮速度缓存数组
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
      hSpeed_Buffer1[i]=0;
			hSpeed_Buffer2[i]=0;
      hSpeed_Buffer3[i]=0;
			hSpeed_Buffer4[i]=0;
    }
    
    bIs_First_Measurement1=true;                                    //电机A以清除速度缓存数组标志位
    bIs_First_Measurement2=true;                                    //电机B以清除速度缓存数组标志位
		bIs_First_Measurement2=true;                                    //电机B以清除速度缓存数组标志位
		bIs_First_Measurement2=true;                                    //电机B以清除速度缓存数组标志位
}

void ENC_Calc_Average_Speed(void)                                   //计算三次电机的平均编码数
{   
	u32 i;
	signed long long wtemp1=0;
	signed long long wtemp2=0;
	signed long long wtemp3=0;
	signed long long wtemp4=0;
   
	for (i=0;i<SPEED_BUFFER_SIZE;i++)                                 //累加缓存次数内的速度值
	{
		wtemp1+=hSpeed_Buffer1[i];
		wtemp2+=hSpeed_Buffer2[i];
		wtemp3+=hSpeed_Buffer3[i];
		wtemp4+=hSpeed_Buffer4[i];
	}
   
	wtemp1/=(SPEED_BUFFER_SIZE);                                      //取平均，平均脉冲数单位为 个/s	
	wtemp2/=(SPEED_BUFFER_SIZE);                                      //平均脉冲数 个/s	
  wtemp3/=(SPEED_BUFFER_SIZE);
	wtemp4/=(SPEED_BUFFER_SIZE);
    
	wtemp1=(wtemp1*SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);          //将平均脉冲数单位转为 r/min
	wtemp2=(wtemp2*SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR);
	wtemp3=(wtemp3*SPEED_SAMPLING_FREQ)*60/(4*ENCODER3_PPR);
	wtemp4=(wtemp4*SPEED_SAMPLING_FREQ)*60/(4*ENCODER4_PPR);	
	
	hRot_Speed1=((s16)(wtemp1));                                      //平均转速 r/min
	hRot_Speed2=((s16)(wtemp2));                                      //平均转速 r/min
	hRot_Speed3=((s16)(wtemp3));                                      //平均转速 r/min
	hRot_Speed4=((s16)(wtemp4));                                      //平均转速 r/min
	
	Speed1=hRot_Speed1;                                               //平均转速 r/min
	Speed2=hRot_Speed2;                                               //平均转速 r/min
	Speed3=hRot_Speed3;                                               //平均转速 r/min
	Speed4=hRot_Speed4;                                               //平均转速 r/min
}

void Gain1(void)                                                    //设置电机A PID调节
{
	//static float pulse = 0;    
	span=1*(Speed2-Speed1);                                           //采集回来的左右轮速度差值
	pulse1=pulse1+PID_calculate(&Control_left,hRot_Speed1);            //PID调节      
	if(pulse1>3600) 
		pulse1=3600;                                                     //pwm幅度抑制
	if(pulse1<0) 
		pulse1=0;    
	TIM4->CCR3=pulse1;                                                 //电机A赋值PWM
	//A_REMP_PLUS=pulse;                                              //电机APID调节后的PWM值缓存
}

void Gain2(void)                                                    //设置电机B PID调节
{
	//static float pulse1 = 0;    
	span=1*(Speed1-Speed2);                                           //采集回来的左右轮速度差值
	pulse2=pulse2+PID_calculate(&Control_right,hRot_Speed2);           //PID调节 
	if(pulse2>3600) 
		pulse2=3600;                                                    //pwm 幅度抑制
	if(pulse2<0) 
		pulse2=0;	
	TIM4->CCR4=pulse2;//电机B赋值PWM	
	//TIM2->CCR3 = A_REMP_PLUS;//电机A赋值PWM
}

void Gain3(void)                                                    //设置电机C PID调节
{
	//static float pulse1 = 0;    
	span=1*(Speed4-Speed3);                                           //采集回来的左右轮速度差值
	pulse3=pulse3+PID_calculate(&Control_right1,hRot_Speed3);           //PID调节 
	if(pulse3>3600) 
		pulse3=3600;                                                    //pwm 幅度抑制
	if(pulse3<0) 
		pulse3=0;	
	TIM5->CCR3=pulse3;//电机B赋值PWM	
	//TIM2->CCR3 = A_REMP_PLUS;//电机A赋值PWM
}

void Gain4(void)                                                    //设置电机D PID调节
{
	//static float pulse1 = 0;    
	span=1*(Speed3-Speed4);                                           //采集回来的左右轮速度差值
	pulse4=pulse4+PID_calculate(&Control_left1,hRot_Speed4);           //PID调节 
	if(pulse4>3600) 
		pulse4=3600;                                                    //pwm 幅度抑制
	if(pulse4<0) 
		pulse4=0;	
	TIM5->CCR4=pulse4;//电机B赋值PWM	
	//TIM2->CCR3 = A_REMP_PLUS;//电机A赋值PWM
}

