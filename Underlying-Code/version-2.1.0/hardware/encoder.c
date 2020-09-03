#include "encoder.h"

/****************************************** 外部引用 **********************************************/

/************** 步骤执行标志位 ****************/
extern u8 main_sta;                                              

/******************** PID参数 **********************/
//结构体定义在pid.h
extern struct PID Control_right;                                   //右前轮A的PID参数
extern struct PID Control_left;                                    //左前轮B的PID参数
extern struct PID Control_left1;                                   //左后轮C的PID参数
extern struct PID Control_right1;                                  //右后轮D的PID参数


/******************************************* 变量 *************************************************/
unsigned short int sampling_interval = SPEED_SAMPLING_TIME;        //电机编码数采集时间间隔
int two_wheels_diff;                                               //存储两轮转速差值，传递给PID_Calculate函数
u8 Speed_Buffer_Underlabeling = 0;                                 //全局变量，每执行一次TIM6中断函数，自加一次，CNT_Buffer数组移位一次，直到达到最大值清零
                                                                   
/************** 脉冲数缓冲数组 *********************/
s32 CNT_Buffer1[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer2[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer3[CNT_BUFFER_SIZE]={0};
s32 CNT_Buffer4[CNT_BUFFER_SIZE]={0};

/*********** 保存前一次编码器的CNT值 ***************/
s32 previous_cnt1;
s32 previous_cnt2;
s32 previous_cnt3; 
s32 previous_cnt4;

/***** 一定时间间隔内脉冲数，用于传递给里程计 ******/
float Odometer_R_CNT=0;
float Odometer_L_CNT=0;
float	Odometer_L1_CNT=0;
float	Odometer_R1_CNT=0;                                       

/*************** 电机平均转速 **********************/
unsigned int RPM_Value1=0;                                        //电机A平均转速 r/min，PID调节
unsigned int RPM_Value2=0;                                        //电机B平均转速 r/min，PID调节
unsigned int RPM_Value3=0;                                        //电机C平均转速 r/min，PID调节
unsigned int RPM_Value4=0;                                        //电机D平均转速 r/min，PID调节

/*************** 定时器溢出次数 ********************/
static volatile u16 Encoder_Timer_Overflow1;                      
static volatile u16 Encoder_Timer_Overflow2;                     
static volatile u16 Encoder_Timer_Overflow3;                     
static volatile u16 Encoder_Timer_Overflow4;                     	

/******************** 比较值存储 *******************/
float pulse1=0;                                                   //电机A 经过的PID调节后比较值，用于返回给定时器CCR寄存器
float pulse2=0;                                                   
float pulse3=0;                                                   
float pulse4=0;

/************* 清除CNT_Buffer数组标志位 ************/
static bool Is_First_Measurement1=true;                           //电机A清零CNT_Buffer1标志位
static bool Is_First_Measurement2=true;                           //电机B清零CNT_Buffer2标志位
static bool Is_First_Measurement3=true;                           //电机C清零CNT_Buffer3标志位
static bool Is_First_Measurement4=true;                           //电机D清零CNT_Buffer4标志位
 

/********************************************* 函数 **********************************************/

/**************** 编码器初始化函数 **************/
void Encoder_Init()
{
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();
	TIM1_Encoder_Init();
	TIM8_Encoder_Init();
	TIM6_Config_Init();                                              //定时器6初始化，用作编码数定时采集、读取
	ENC_Clear_CNT_Buffer();
}

/********************************* 定时器编码器模式初始化 *****************************/

/**************** 定时器2函数初始化 *************/
//用到引脚PA15、PB3
void TIM2_Encoder_Init(void)
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  
	/************* 端口重映射 *************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            //端口重映射需要开启复用时钟
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);	          //端口重映射使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);            //定时器2时钟使能
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);        //禁用JTAG,感觉没什么用
	
  /*********** 使能PA15引脚 *************/	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/************* 使能PB3引脚 ************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/************ TIM2 基本配置 ***********/
	TIM_TimeBaseStructure.TIM_Period = 65535;                       //计次数
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                        //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              
    
	/******* 配置编码器触发源和极性 *******/          
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	          //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                        //配置输入滤波器，不滤
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE);                                          //启动TIM2定时器
 }

/**************** 定时器3函数初始化 *************/
//用到引脚PA6、PA7
void TIM3_Encoder_Init(void)
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	
	/************* 使能PA6、PA7 ************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/************ TIM3 基本配置 ***********/ 
	TIM_TimeBaseStructure.TIM_Period = 65535;                        //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  /****** 配置编码器模式触发源和极性 *****/            
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //CC1S=01 	选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //配置输入滤波器 不滤
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE);                                           //启动TIM3定时器
 }

/**************** 定时器1函数初始化 *************/
//用到引脚PA8、PA9
void TIM1_Encoder_Init(void)
{     
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
	/************* 使能PA8、PA9 *******************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	/**************** TIM1 基本配置 ***************/
	TIM_TimeBaseStructure.TIM_Period = 65535;                      	 //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                         //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   
	
  /********* 配置编码器模式触发源和极性 *********/            
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                 //选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	   //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //映射到TI上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	           //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                         //配置输入滤波器 不滤
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM1,0);
	TIM_Cmd(TIM1, ENABLE);                                           //启动TIM1定时器
}

/**************** 定时器1函数初始化 *************/
//用到引脚PC6、PC7
void TIM8_Encoder_Init(void) 
{      	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	
	/************* 使能PC6、PC7 *******************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/**************** TIM8 基本配置 ***************/	
	TIM_TimeBaseStructure.TIM_Period = 65535;                         //计次数，周期配置得好好看看
	TIM_TimeBaseStructure.TIM_Prescaler = 0;                          //预分频设为零，每个边沿都可以检测到
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0; 
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
	
  /********* 配置编码器模式触发源和极性 *********/            
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                  //选择输入端 IC1映射到TI1上
 	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	    //双边沿捕获
 	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //映射到TI上
 	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	            //配置输入分频,不分频
 	TIM_ICInitStructure.TIM_ICFilter = 0x01;                          //配置输入滤波器 不滤
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
			
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 	
	TIM_SetCounter(TIM8,0);
	TIM_Cmd(TIM8, ENABLE);                                            //启动TIM8定时器
 }

/******************************** 定时器中断处理函数 *************************************/

/*************** 定时器2中断处理函数 *****************/
void TIM2_IRQHandler (void)                                         //执行TIM2(电机A编码器采集)计数中断
{   
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow1!=U16_MAX)                           //不超范围  
    {
        Encoder_Timer_Overflow1++;                                  //脉冲数累加
			  //printf("Encoder_Timer_Overflow1:%d\r\n",Encoder_Timer_Overflow1);           //用于调试编码器中断是否正常运行、编码器接线有无问题
    }
}

/*************** 定时器3中断处理函数 *****************/
void TIM3_IRQHandler (void)                                         //执行TIM2(电机B编码器采集)计数中断
{  
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow2!=U16_MAX)                           //不超范围    
    { 
        Encoder_Timer_Overflow2++;	                                //脉冲数累加
				//printf("Encoder_Timer_Overflow2:%d\r\n",Encoder_Timer_Overflow2);
    }
}

/*************** 定时器1中断处理函数 *****************/
//高级定时器的中断不同于同时定时器，划分更细
void TIM1_UP_IRQHandler(void)                                        //执行TIM1(电机C编码器采集)计数中断
{  
	
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
    if (Encoder_Timer_Overflow3!=U16_MAX)                            //不超范围    
    {
        Encoder_Timer_Overflow3++;	                                 //脉冲数累加
				//printf("Encoder_Timer_Overflow3:%d\r\n",Encoder_Timer_Overflow3);
    }
} 

/*************** 定时器8中断处理函数 *****************/
void TIM8_UP_IRQHandler(void)                                        //执行TIM8(电机D编码器采集)计数中断
{  
    TIM_ClearFlag(TIM8,  TIM_FLAG_Update);
    if (Encoder_Timer_Overflow4!=U16_MAX)                            //不超范围    
    {
        Encoder_Timer_Overflow4++;	                                 //脉冲数累加
				//printf("Encoder_Timer_Overflow4:%d\r\n",Encoder_Timer_Overflow4);
    }
}

/********************************************* 编码数采集及计算 *************************************/

/***************** 定时器6初始化函数 *****************/
void TIM6_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=399;                                //暂时设置 计数中断时间：5ms中断一次
	TIM_TimBaseStructrue.TIM_Prescaler=8;                               //Tout=(ARR+1)*(PSC+1)/Tclk
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;            //向上计数
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6, ENABLE); 
}

/****************** 定时器6中断函数 *******************/
void TIM6_IRQHandler(void)                                            //编码数采集、转速计算、PID控制
{
	
	//printf("TIM6_IRQHandler is on.\r\n");                             //调试用，测试TIM6IRQHandler是否正常触发

	if ( TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET) 
	{						      
        if (sampling_interval!=0)                                     //电机编码数采样时间间隔未到
        {
            sampling_interval--;                                      //开始倒数						
        }
        else                                                          //电机编码数采样时间间隔到了
        {
            sampling_interval = SPEED_SAMPLING_TIME;                  //恢复电机编码数采样时间间隔
					  s32 temp_cnt1,temp_cnt2,temp_cnt3,temp_cnt4;              //临时变量储存获取脉冲数，供其他变量调用                                 
           
   					/************************ 获取脉冲数 *******************/            
            temp_cnt1=ENC_Delta_CNT1();                               //temp_cnt1储存获取到的脉冲数，供其他变量调用
            temp_cnt2=ENC_Delta_CNT2();                               
            temp_cnt3=ENC_Delta_CNT3();                               
            temp_cnt4=ENC_Delta_CNT4(); 

					  //printf("temp_cnt:%d\r\n",temp_cnt1);                      //调试用
					
//          如果为停止指令，即左右轮速度为零，则清除速度存储器防止前后速度差太大造成小车冲转
//          if((temp_cnt2 == 0)&&(temp_cnt1 == 0))
//          {
//          	pulse=pulse1=0;
//          }
             
            /********************** 储存脉冲数 *********************/           
            Odometer_R_CNT=(float)temp_cnt1;                          //将脉冲信号数以全局变量的方式共享给odometer函数
            Odometer_L_CNT=(float)temp_cnt2;
            Odometer_L1_CNT=(float)temp_cnt3;
					  Odometer_R1_CNT=(float)temp_cnt4;						
            main_sta|=0x02;                                           //执行计算里程计数据步骤

            /***************** 传递脉冲数到缓存数组 ****************/           
            CNT_Buffer1[Speed_Buffer_Underlabeling] = temp_cnt1;      //Speed_Buffer_Underlabeling 全局变量，每执行一次中断函数自加一次，直到达到最大值清零     
            CNT_Buffer2[Speed_Buffer_Underlabeling] = temp_cnt2;
						CNT_Buffer3[Speed_Buffer_Underlabeling] = temp_cnt3;
						CNT_Buffer4[Speed_Buffer_Underlabeling] = temp_cnt4;
            Speed_Buffer_Underlabeling++;                             //数组移位            
           
						/********************* 缓存结束判断 ********************/
            if(Speed_Buffer_Underlabeling >=CNT_BUFFER_SIZE)          //缓存轮子编码数到数组结束判断
            {
                Speed_Buffer_Underlabeling=0;                         //缓存轮子编码数到数组变量清零
            }
          
            ENC_Average_RPM();                                        //计算三次电机的平均编码数
						
						/********************** PID调节控制 ********************/
            Gain_PID1();                                            //电机A转速PID调节控制 
            Gain_PID2();                                            //电机B转速PID调节控制 
						Gain_PID3();                                            //电机C转速PID调节控制 
						Gain_PID4();                                            //电机D转速PID调节控制 
        }       
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);                  //中断标志位清零    		 
	}		 
}

/********** 计算电机A一定时间间隔内的脉冲数 **********/
s16 ENC_Delta_CNT1(void)
{   
	  s32 delta_cnt1;                                                 //之前cnt和当前cnt的差值
    u16 encoder_timer_overflow_sample;                              //定时采样TIM2中断产生次数
    u16 current_cnt1;                                               //当前cnt值读取，previous_cnt是全局变量，current_cnt是局部变量

    if (!Is_First_Measurement1)                                     //除程序第一次运行不执行外，以后都执行该if函数段；判断CNT_Buffer1是否清零
    {  
        encoder_timer_overflow_sample=Encoder_Timer_Overflow1; 	
        current_cnt1=TIM2->CNT;
        Encoder_Timer_Overflow1=0;  

        if ( (TIM2->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {
            //encoder timer down-counting 反转情况计算     
            delta_cnt1 = (s32)((encoder_timer_overflow_sample)*(4*ENCODER1_PPR)-(current_cnt1-previous_cnt1));//(current_cnt1 - previous_cnt1 - (encoder_timer_overflow_sample)*(4*ENCODER1_PPR));
        }
        else  
        {
            //encoder timer up-counting 正转情况计算
            delta_cnt1 = (s32)(current_cnt1 - previous_cnt1 + (encoder_timer_overflow_sample)*(4*ENCODER1_PPR));
        }		
    } 
    else                                                           //程序第一次运行，执行该else函数段
    {
        Is_First_Measurement1= false;                              //电机A清零CNT_Buffer1标志位
        delta_cnt1 = 0;
        Encoder_Timer_Overflow1=0;                                 //中断次数记录清零
    }
    previous_cnt1=TIM2->CNT;  
    return((s16) delta_cnt1);
}

/********** 计算电机B一定时间间隔内的脉冲数 **********/
s16 ENC_Delta_CNT2(void)
{   
    s32 delta_cnt2;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt2;                                               

    if (!Is_First_Measurement2)                                    //判断CNT_Buffer2是否清零
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow2; 	 //得到采样时间内的编码数	
        current_cnt2=TIM3->CNT;
        Encoder_Timer_Overflow2=0;                                 //清除脉冲数累加   

        if ((TIM3->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting 反转情况计算
            delta_cnt2=(s32)((encoder_timer_overflow_sample)*(4*ENCODER2_PPR)-(current_cnt2-previous_cnt2));	
        }
        else  
        {
            //encoder timer up-counting 正转情况计算
            delta_cnt2 =(s32)( current_cnt2 - previous_cnt2 + (encoder_timer_overflow_sample)*(4*ENCODER2_PPR) );
        }
    } 
    else
    {
        Is_First_Measurement2=false;                               //电机B清零CNT_Buffer2标志位
        delta_cnt2=0;
        Encoder_Timer_Overflow2=0;       
    }
    previous_cnt2=TIM3->CNT;  
    return((s16) delta_cnt2);
}

/********** 计算电机C一定时间间隔内的脉冲数 **********/
s16 ENC_Delta_CNT3(void)
{   
    s32 delta_cnt3;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt3;

    if (!Is_First_Measurement3)                                    //判断CNT_Buffer3是否清零
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow3; 	 //得到采样时间内的编码数	
        current_cnt3=TIM1->CNT;
        Encoder_Timer_Overflow3=0;                                 //清除脉冲数累加  

        if ((TIM1->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting 反转情况计算
            delta_cnt3=(s32)((encoder_timer_overflow_sample)*(4*ENCODER3_PPR)-(current_cnt3-previous_cnt3));
        }
        else  
        {
            //encoder timer up-counting 正转情况计算
            delta_cnt3 =(s32)( current_cnt3 - previous_cnt3 + (encoder_timer_overflow_sample)*(4*ENCODER3_PPR) );
        }
    } 
    else
    {
        Is_First_Measurement3=false;                               //电机C清零CNT_Buffer3标志位
        delta_cnt3=0;
        Encoder_Timer_Overflow3=0;       
    }
    previous_cnt3=TIM1->CNT;  
    return((s16) delta_cnt3);
}

/********** 计算电机D一定时间间隔内的脉冲数 **********/
s16 ENC_Delta_CNT4(void) 
{   
    s32 delta_cnt4;
    u16 encoder_timer_overflow_sample;
    u16 current_cnt4;

    if (!Is_First_Measurement4)                                    //判断CNT_Buffer4是否清零
    {   
        encoder_timer_overflow_sample = Encoder_Timer_Overflow4; 	 //得到采样时间内的编码数	
        current_cnt4=TIM8->CNT;
        Encoder_Timer_Overflow4=0;                                 //清除脉冲数累加  

        if ((TIM8->CR1&TIM_CounterMode_Down)==TIM_CounterMode_Down)  
        {
            //encoder timer down-counting 反转情况计算
            delta_cnt4=(s32)((encoder_timer_overflow_sample)*(4*ENCODER4_PPR)-(current_cnt4-previous_cnt4));	
        }
        else  
        {
            //encoder timer up-counting 正转情况计算
            delta_cnt4 =(s32)(current_cnt4-previous_cnt4+(encoder_timer_overflow_sample)*(4*ENCODER4_PPR));
        }
    } 
    else
    {
        Is_First_Measurement4=false;                               //电机D清零CNT_Buffer4标志位
        delta_cnt4=0;
        Encoder_Timer_Overflow4=0;      
    }
    previous_cnt4=TIM8->CNT;  
    return((s16) delta_cnt4);
}

/********************* CNT_Buffer清零 ********************/
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

/********* 读取三次脉冲数，分别求出四电机平均转速 ********/
void ENC_Average_RPM(void)
{   
	u32 i;
	signed long long wtemp1=0;                                       //函数内部临时变量，不同步骤含义不同
	signed long long wtemp2=0;
	signed long long wtemp3=0;
	signed long long wtemp4=0;
   
	for (i=0;i<CNT_BUFFER_SIZE;i++)                                  //累加缓存次数内的速度值
	{
		wtemp1+=CNT_Buffer1[i];                                        //wtemp1为CNT_Buffer1数组总和
		wtemp2+=CNT_Buffer2[i]; 
		wtemp3+=CNT_Buffer3[i];
		wtemp4+=CNT_Buffer4[i];
	}
   
	wtemp1/=(CNT_BUFFER_SIZE);                                       //wtemp1为平均脉冲数，单位为个/s	
	wtemp2/=(CNT_BUFFER_SIZE);
  wtemp3/=(CNT_BUFFER_SIZE);
	wtemp4/=(CNT_BUFFER_SIZE);
    
	wtemp1=(wtemp1*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER1_PPR);    //将平均脉冲数单位转为r/min 即rpm，国际标准单位
	wtemp2=(wtemp2*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER2_PPR);    //SPEED_SAMPLING_FREQUENCE 采样频率
	wtemp3=(wtemp3*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER3_PPR);    //与SPEED_SAMPLING_TIME采样时间含义上成倒数，但数值上还需转换   
	wtemp4=(wtemp4*SPEED_SAMPLING_FREQUENCE)*60/(4*ENCODER4_PPR);	   //ENCODER4_PPR 电机4码盘线数                                            
	
	RPM_Value1=((s16)(wtemp1));                                      //平均转速 r/min
	RPM_Value2=((s16)(wtemp2));                                      //RPM_Value1 全局变量
	RPM_Value3=((s16)(wtemp3));                                                
	RPM_Value4=((s16)(wtemp4));                                                
}


/************************* PID调节参数回传 *****************************/

/*************** 电机A PID调节 ****************/
void Gain_PID1(void)
{    
	two_wheels_diff=1*(RPM_Value2-RPM_Value1);                       //全局变量，存储两轮转速差值，在本函数内部传递给PID_Calculate函数
	pulse1=pulse1+PID_Calculate(&Control_right,RPM_Value1);          //PID调节      
	if(pulse1>3600) 
		pulse1=3600;                                                   //pwm幅度抑制
	if(pulse1<0) 
		pulse1=0;    
	TIM4->CCR3=pulse1;                                               //电机A赋值PWM
}

/*************** 电机B PID调节 ****************/
void Gain_PID2(void)
{   
	two_wheels_diff=1*(RPM_Value1-RPM_Value2);                       //全局变量，存储两轮转速差值，在本函数内部传递给PID_Calculate函数
	pulse2=pulse2+PID_Calculate(&Control_left,RPM_Value2);           //PID调节 
	if(pulse2>3600) 
		pulse2=3600;                                                   //pwm幅度抑制
	if(pulse2<0) 
		pulse2=0;	
	TIM4->CCR4=pulse2;                                               //电机B赋值PWM	
}

/*************** 电机C PID调节 ****************/
void Gain_PID3(void)
{    
	two_wheels_diff=1*(RPM_Value4-RPM_Value3);                       //全局变量，存储两轮转速差值，在本函数内部传递给PID_Calculate函数
	pulse3=pulse3+PID_Calculate(&Control_left1,RPM_Value3);          //PID调节
	if(pulse3>3600) 
		pulse3=3600;                                                   //pwm幅度抑制
	if(pulse3<0) 
		pulse3=0;	
	TIM5->CCR3=pulse3;                                               //电机C赋值PWM	
}

/*************** 电机D PID调节 ****************/
void Gain_PID4(void)
{   
	two_wheels_diff=1*(RPM_Value3-RPM_Value4);                       //全局变量，存储两轮转速差值，传递给PID_Calculate函数
	pulse4=pulse4+PID_Calculate(&Control_right1,RPM_Value4);         //PID调节 
	if(pulse4>3600) 
		pulse4=3600;                                                   //pwm幅度抑制
	if(pulse4<0) 
		pulse4=0;	
	TIM5->CCR4=pulse4;                                               //电机D赋值PWM	
}

