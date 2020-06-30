#include "encoder.h"

//_encoder Encoder; //全局变量  版本1
struct PID Control_left  ={0.01,0.1,0.75,0,0,0,0,0,0};//左轮PID参数，适于新电机4096
struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//右轮PID参数，适于新电机4096
extern s32 hSpeed_Buffer1[],hSpeed_Buffer2[];//左右轮速度缓存数组

u8 bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变量
extern float pulse;//电机A PID调节后的PWM值缓存
extern float pulse1;//电机B PID调节后的PWM值缓存
extern u8 main_sta;//主函数步骤执行标志位
float  Milemeter_L_Motor=0,Milemeter_R_Motor=0;//dt时间内的左右轮速度,用于里程计计算


s32 hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};//左右轮速度缓存数组
static unsigned int hRot_Speed2;//电机A平均转速缓存
static unsigned int hRot_Speed1;//电机B平均转速缓存
unsigned int Speed2=0; //电机A平均转速 r/min，PID调节
unsigned int Speed1=0; //电机B平均转速 r/min，PID调节

static volatile u16 hEncoder_Timer_Overflow1;//电机B编码数采集 
static volatile u16 hEncoder_Timer_Overflow2;//电机A编码数采集

//float A_REMP_PLUS;//电机APID调节后的PWM值缓存
float pulse = 0;//电机A PID调节后的PWM值缓存
float pulse1 = 0;//电机B PID调节后的PWM值缓存

int span;//采集回来的左右轮速度差值

static bool bIs_First_Measurement2 = true;//电机A以清除速度缓存数组标志位
static bool bIs_First_Measurement1 = true;//电机B以清除速度缓存数组标志位

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
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	    //端口重映射需要开启复用时钟
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //RCC_APB2Periph_AFIO 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);     //禁用JTAG,感觉没什么用
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*- 正交编码器输入引脚  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/*- 正交编码器输入引脚  -*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*- TIM2编码器模式配置 -*/
	//TIM_DeInit(TIM2); 
	TIM_TimeBaseStructure.TIM_Period = 65535;//计次数;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	//配置编码器模式触发源和极性双边沿检测 不知道可以看技术手册上面有介绍编码器模式            
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器，不滤
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);   //启动TIM2定时器
 }

void TIM3_Encoder_Init(void)//PA6,PA7
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
	TIM_TimeBaseStructure.TIM_Period = 65535;//计次数;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              
    //配置编码器模式触发源和极性双边沿检测 不知道可以看技术手册上面有介绍编码器模式             
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	//双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);   //启动TIM3定时器
 }

void TIM2_IRQHandler (void)//执行TIM4(电机A编码器采集)计数中断
{   
    TIM_ClearFlag(ENCODER2_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow2 != U16_MAX)//不超范围  
    {
        hEncoder_Timer_Overflow2++; //脉冲数累加
			//printf("hEncoder_Timer_Overflow2=%d\r\n",hEncoder_Timer_Overflow2);
    }
}

void TIM3_IRQHandler (void)//执行TIM3(电机B编码器采集)计数中断
{  
    TIM_ClearFlag(ENCODER1_TIMER, TIM_FLAG_Update);
    if (hEncoder_Timer_Overflow1 != U16_MAX)//不超范围    
    {
        hEncoder_Timer_Overflow1++;	 //脉冲数累加
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

//以下2个中断是计数器溢出更新中断 我在初始化里面给的是65535次 但是我每隔5ms就读取了一次CNT 然后又将计数器清零了
//所以下面两个中断是不会发生的 除非你的电机在5ms内能够产生>=65535/4（我用的是双边沿检测）个脉冲。
void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	//TIM6初始化
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=4000;//暂时设置 计数中断时间：5ms中断一次
	TIM_TimBaseStructrue.TIM_Prescaler=8;
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;  //向上计数
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

//void TIM6_IRQHandler(void)								//定是时间不能太长否则计数器溢出了就会测量错误
//{	

//	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  
//	{     
//		//Ranging(Read_Encoder(2),Read_Encoder(3));	//每隔5ms中断一次 读取对应CNT的值,并计算路程
//		//printf("Encoder.M1Lneth=%f\r\n",Encoder.M1Lneth-55000);//打印出电机1的路程
//		//printf("Encoder.M2Lneth=%f\r\n",Encoder.M2Lneth);//打印出电机2的路程	
//		//printf("Encoder.TIM3=%d\r\n",Read_Encoder(3)-300);
//		//printf("Encoder.TIM2=%d\r\n",Read_Encoder(2));
//		//printf("Encoder.TIM5=%d\r\n",Read_Encoder(5));

//	}	   
//	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//}

void TIM6_IRQHandler(void)//小车速度计算定时器中断函数
{
	if ( TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET ) 
	{						      
        if (hSpeedMeas_Timebase_500us !=0)//电机编码数采集时间间隔未到
        {
            hSpeedMeas_Timebase_500us--;//开始倒数	
        }
        else    //电机编码数采集时间间隔到了
        {
            s32 wtemp2,wtemp1;
           // printf("tim6 is on\r\n");
            hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//恢复电机编码数采集时间间隔
            
            /************************ 1 ***************************/
            
            wtemp2 = ENC_Calc_Rot_Speed2(); //A 获取的编码数
            wtemp1 = ENC_Calc_Rot_Speed1(); //B 获取的编码数
            
//            //如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
//            if((wtemp2 == 0) && (wtemp1 == 0))
//            {
//                pulse=pulse1=0;
//            }
             
            /************************ 2 ***************************/
            
            //储存编码数（脉冲数），用于里程计计算
            Milemeter_L_Motor= (float)wtemp1; //储存脉冲数
            Milemeter_R_Motor= (float)wtemp2;
            
            main_sta|=0x02;//执行计算里程计数据步骤

            /************************ 3 ***************************/
            
            //开始缓存左右轮编码数到数组
            hSpeed_Buffer2[bSpeed_Buffer_Index] = wtemp2;
            hSpeed_Buffer1[bSpeed_Buffer_Index] = wtemp1;
            bSpeed_Buffer_Index++;//数组移位
            
            //缓存左右轮编码数到数组结束判断
            if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
            {
                bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清零
            }
            
            /************************ 4 ***************************/
            
            ENC_Calc_Average_Speed();//计算三次电机的平均编码数
            Gain2(); //电机A转速PID调节控制 右
            Gain1(); //电机B转速PID调节控制 左
        }
        
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);//清除中断标志位    		 
	}		 
}


// /**************************************************************************
//函数功能：单位时间读取编码器计数,此函数在定时器中断里面用我的是定时器6每隔5ms读取一次
//入口参数：定时器
//返回  值：计数值
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
//函数功能：单位测距函数
//入口参数：int
//返回  值：void
////该函数的作用是得到两个电机在单位时间内记录的CNT个数（我用的是定时器6）
////里面的Encoder.M1Lneth+、Encoder.M2Lneth+和Encoder.Avg_lenth+是每次定时以后的CNT算出的路程的累加
////Encoder.M1Lneth是我自己定义的结构体 
//**************************************************************************/
//	
//void Ranging(int M1_cnt,int M2_cnt) //M1电机cnt M2左电机cnt
//{
//	if((M1_cnt>=0&&M2_cnt>=0)||(M1_cnt<0&&M2_cnt<0))
//	{	
//		Encoder.M1Lneth+=M1_cnt;//*0.0000744*0.85;//0.0000744*0.85是我根据实际转动的路程换算而来 不同的轮子不一样
//		Encoder.M2Lneth+=M2_cnt;//*0.0000744*0.85;
//		Encoder.Avg_lenth+=(((M1_cnt+M2_cnt)/2)*0.0000744)*0.85; 
//	}
//}
s16 ENC_Calc_Rot_Speed2(void)//计算电机A的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement2)//电机A以清除速度缓存数组
    {  
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow2; 	
        hCurrent_angle_sample_one = ENCODER2_TIMER->CNT;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;   

        if ( (ENCODER2_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            // encoder timer down-counting 反转的速度计算     
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR) -(hCurrent_angle_sample_one - hPrevious_angle2));
        }
        else  
        {
            //encoder timer up-counting 正转的速度计算
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle2 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER2_PPR));
        }		
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement2 = false;//电机A以清除速度缓存数组标志位
        temp = 0;
        hEncoder_Timer_Overflow2 = 0;
        haux = ENCODER2_TIMER->CNT;       
    }
    hPrevious_angle2 = haux;  
    return((s16) temp);
}


s16 ENC_Calc_Rot_Speed1(void)//计算电机B的编码数
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one;
    u16 hCurrent_angle_sample_one;
    s32 temp;
    s16 haux;

    if (!bIs_First_Measurement1)//电机B以清除速度缓存数组
    {   
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow1; 	//得到采样时间内的编码数	
        hCurrent_angle_sample_one = ENCODER1_TIMER->CNT;
        hEncoder_Timer_Overflow1 = 0;//清除脉冲数累加
        haux = ENCODER1_TIMER->CNT;   

        if ( (ENCODER1_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            // encoder timer down-counting 反转的速度计算
            wDelta_angle = (s32)((hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR) -(hCurrent_angle_sample_one - hPrevious_angle1));	
        }
        else  
        {
            //encoder timer up-counting 正转的速度计算
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle1 + (hEnc_Timer_Overflow_sample_one) * (4*ENCODER1_PPR));
        }
        temp=wDelta_angle;
    } 
    else
    {
        bIs_First_Measurement1 = false;//电机B以清除速度缓存数组标志位
        temp = 0;
        hEncoder_Timer_Overflow1 = 0;
        haux = ENCODER1_TIMER->CNT;       
    }
    hPrevious_angle1 = haux;  
    return((s16) temp);
}

void ENC_Clear_Speed_Buffer(void)//速度存储器清零
{   
    u32 i;

    //清除左右轮速度缓存数组
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        hSpeed_Buffer2[i] = 0;
        hSpeed_Buffer1[i] = 0;
    }
    
    bIs_First_Measurement2 = true;//电机A以清除速度缓存数组标志位
    bIs_First_Measurement1 = true;//电机B以清除速度缓存数组标志位
}

void ENC_Calc_Average_Speed(void)//计算三次电机的平均编码数
{   
    u32 i;
	signed long long wtemp3=0;
	signed long long wtemp4=0;

    //累加缓存次数内的速度值
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp4 += hSpeed_Buffer2[i];
		wtemp3 += hSpeed_Buffer1[i];
	}
    
    //取平均，平均脉冲数单位为 个/s	
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //平均脉冲数 个/s	
    
    //将平均脉冲数单位转为 r/min
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR); 
		
	hRot_Speed2= ((s16)(wtemp4));//平均转速 r/min
	hRot_Speed1= ((s16)(wtemp3));//平均转速 r/min
	Speed2=hRot_Speed2;//平均转速 r/min
	Speed1=hRot_Speed1;//平均转速 r/min
}

void Gain2(void)//设置电机A PID调节 PA2
{
	//static float pulse = 0;
    
	span=1*(Speed1-Speed2);//采集回来的左右轮速度差值
	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);//PID调节
    
    //pwm幅度抑制
	if(pulse > 3600) pulse = 3600;
	if(pulse < 0) pulse = 0;
    
	//A_REMP_PLUS=pulse;//电机APID调节后的PWM值缓存
}


void Gain1(void)//设置电机B PID调节 PA1
{
	//static float pulse1 = 0;
    
	span=1*(Speed2-Speed1);//采集回来的左右轮速度差值
	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);//PID调节
    
    ////pwm 幅度抑制
	if(pulse1 > 3600) pulse1 = 3600;
	if(pulse1 < 0) pulse1 = 0;
	
	TIM4->CCR3 = pulse1;//电机B赋值PWM
	//TIM2->CCR3 = A_REMP_PLUS;//电机A赋值PWM
    TIM4->CCR4 = pulse;//电机A赋值PWM
}


