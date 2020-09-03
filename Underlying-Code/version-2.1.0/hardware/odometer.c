#include "odometer.h"

/*************************************************** 外部引用 ***********************************************************/

/************ 标志位 **************/
//usart.c中定义
extern u8 main_sta;

/***** 串口得到的四轮预期速度 *****/
//usart.c中定义
extern float odometer_right;
extern float odometer_left;
extern float odometer_right1;
extern float odometer_left1;

/*************** 电机平均转速 **********************/
extern unsigned int RPM_Value1;                           
extern unsigned int RPM_Value2;                           
extern unsigned int RPM_Value3;                           
extern unsigned int RPM_Value4;                           


/******************************************************* 变量 ***********************************************************/

/*********** 里程计数据 ***********/
float position_x=0;
float position_y=0;
float oriention=0;                                          //oriention方向
float velocity_linear=0;
float velocity_angular=0;

/************ 标志位 **************/
u8 once=1;                                                  //一次性标志位

/************ 小车参数 ************/

/*
	C语言中常量加后缀问题：
		u为unsigned，l为long，f为float。
    C语言中常量的默认情况：
			整型默认为有符号int，浮点型默认为double，如果整形常量大小超出int，默认自然就为long int；
			如果需要无符号整形常量的话就可以在常量后面加u或是U，如0u或是0U；
			同理可以在常量后面加ul或UL表示无符号长整形常量；
			而浮点型常量后缀只有f或F，l或L，没有u或U，因为浮点数一般都为有符号。
*/

float wheel_interval= 200.0000f;                            //轮距/mm 
float wheelbase=210.000f;                                   //轴距/mm
float multiplier=4.0f;                                      //倍频数                         
float deceleration_ratio=34.0f;                             //减速比
float wheel_diameter=75.0f;                                 //轮子直径/mm
float dt=0.005f;                                            //采样时间间隔/s 5ms
float line_number=90.0f;                                    //码盘线数

/************ 计算常量 ************/
float pi_1_2=1.570796f;			                                //π/2
float pi=3.141593f;                                         //π
float pi_3_2=4.712389f;			                                //π*3/2
float pi_2_1=6.283186f;			                                //π*2
float const_frame=0;                                        //轮子转一圈，每单位要走的路径
float const_angle=0;                                        //绕Yaw轴的单位角度

/************ 微分量 **************/
float oriention_interval=0;                                 //dt时间内方向变化值
float sin_=0;                                               //sin角度计算值
float cos_=0;                                               //cos角度计算值
float delta_distance=0;                                     //采样时间间隔内运动的距离
float delta_oriention=0;                                    //采样时间间隔内运动的角度
float distance_sum=0;                                       //dt时间内行驶距离
float distance_diff=0;                                      //dt时间内行驶角度
float oriention_1=0;                                        //                                    ??


/******************************************************** 函数 **********************************************************/

/********************* 里程计计算函数 ************************/
//参数由encoder.c中TIM6_IRQHandler函数传入
void odometer(float R_CNT,float L_CNT,float L1_CNT,float R1_CNT)
{	
	if(once)                                                                              //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);          //路径/圈=轮子直径*码盘线速*倍频数*减速比 mm/个
		const_angle=const_frame/wheel_interval;                                             //单位角度=（路径/圈）/轴距
		once=0;
	}
    
//	distance_sum = 0.25f*(R_CNT+L_CNT+L1_CNT+R1_CNT);                                   //在很短的时间内，轮子的平均脉冲信号数
//	distance_diff = (R_CNT+R1_CNT)/2 - (L_CNT+L1_CNT)/2;                                //在很短的时间内，小车行驶的角度为两轮速度差
//                                                                                      //将左前左后轮等效到中间一轮，四轮简化成两轮差速模型，左转为正
//  /************* 根据左右轮的方向，纠正短时间内小车行驶的路程和角度量的正负 **********/
//	if((odometer_right>0)&&(odometer_left>0))            //左右均正
//	{
//		delta_distance = distance_sum;
//		delta_oriention = distance_diff;
//	}
//	else if((odometer_right<0)&&(odometer_left<0))       //左右均负
//	{
//		delta_distance = -distance_sum;
//		delta_oriention = -distance_diff;
//	}
//	else if((odometer_right<0)&&(odometer_left>0))       //左正右负
//	{
//		delta_distance = -distance_diff;
//		delta_oriention = -2.0f*distance_sum;		
//	}
//	else if((odometer_right>0)&&(odometer_left<0))       //左负右正
//	{
//		delta_distance = distance_diff;
//		delta_oriention = 2.0f*distance_sum;
//	}
//	else
//	{
//		delta_distance=0;
//		delta_oriention=0;
//	}
//    
//	oriention_interval = delta_oriention * const_angle;                                   //采样时间内走的角度	
//	oriention = oriention + oriention_interval;                                           //计算出里程计方向角
//	oriention_1 = oriention + 0.5f * oriention_interval;                                  //里程计方向角数据位数变化，用于三角函数计算
//	
//  sin_ = sin(oriention_1);                                                              //计算出采样时间内y坐标
//	cos_ = cos(oriention_1);                                                              //计算出采样时间内x坐标
//	
//  position_x = position_x + delta_distance * cos_ * const_frame;                        //计算出里程计x坐标
//	position_y = position_y + delta_distance * sin_ * const_frame;                        //计算出里程计y坐标
//    
//	velocity_linear = delta_distance*const_frame / dt;                                    //计算出里程计小车线速度
//	velocity_angular = oriention_interval / dt;                                           //计算出里程计小车角速度

	/********* 四轮差速运动模型 *******/
	int mark=1;
	float temp1=0;
	float temp2=0;
	if(odometer_right>0&&odometer_left>0&&odometer_left1>0&&odometer_right1>0&&fabs(odometer_left+odometer_left1-odometer_right-odometer_right1)<2)     //向前直行
	{
		velocity_linear = (RPM_Value1+RPM_Value2+RPM_Value3+RPM_Value4)*pi*wheel_diameter/(4*deceleration_ratio*60);
		velocity_angular = 0;
		mark=0;
	}
	
	if(odometer_right<0&&odometer_left<0&&odometer_left1<0&&odometer_right1<0&&fabs(odometer_left+odometer_left1-odometer_right-odometer_right1)<2)    //向后直行
	{
		velocity_linear = (RPM_Value1+RPM_Value2+RPM_Value3+RPM_Value4)*pi*wheel_diameter/(4*deceleration_ratio*60);
		velocity_angular = 0;
		mark=0;
	}
	
	if(odometer_right>0&&odometer_right1>0&&(odometer_right+odometer_right1-odometer_left-odometer_left1)>2)           //向左转
	{
		velocity_linear = (RPM_Value1+RPM_Value2+RPM_Value3+RPM_Value4)*pi*wheel_diameter/(4*deceleration_ratio*60);
		temp1=(RPM_Value1-RPM_Value2-RPM_Value3+RPM_Value4)*pi*wheel_diameter;
		temp2=sqrt(wheel_interval* wheel_interval+wheelbase*wheelbase);
		velocity_angular = (wheel_interval*temp1)/(temp2*60*deceleration_ratio);
		mark=0;
	}

	if(odometer_left>0&&odometer_left1>0&&(odometer_left+odometer_left1-odometer_right-odometer_right1)>2)             //向右转
	{
		velocity_linear = (RPM_Value1+RPM_Value2+RPM_Value3+RPM_Value4)*pi*wheel_diameter/(4*deceleration_ratio*60);
		temp1=(RPM_Value1-RPM_Value2-RPM_Value3+RPM_Value4)*pi*wheel_diameter;
		temp2=sqrt(wheel_interval* wheel_interval+wheelbase*wheelbase);
		velocity_angular = (wheel_interval*temp1)/(temp2*60*deceleration_ratio);
	}
	
	if(mark)
	{
		velocity_linear=0;
		velocity_angular=0;
	}
	
	oriention = oriention+velocity_angular*dt;
	position_x = position_x + cos(velocity_angular*dt);
	position_y = position_y + sin(velocity_angular*dt);
	
  /********* 方向角角度纠正 *********/
	if(oriention > pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}

/***************** 里程计发布初始化 *********************/
void Odometer_Pub_Init(void)
{
		TIM7_Config_Init();
}

/***************** 定时器7初始化函数 ********************/
void TIM7_Config_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimBaseStructrue;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimBaseStructrue.TIM_Period=500;                                                  //计数中断时间：5ms中断一次
	TIM_TimBaseStructrue.TIM_Prescaler=(7200-1);
	TIM_TimBaseStructrue.TIM_CounterMode=TIM_CounterMode_Up;                              //向上计数
	TIM_TimBaseStructrue.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7,&TIM_TimBaseStructrue);

	TIM_ClearFlag(TIM7,TIM_FLAG_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7, ENABLE); 
}

/******************** 定时器7中断函数 ******************/
void TIM7_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM7 , TIM_IT_Update) != RESET ) 
	{	
		main_sta|=0x01;                                                                    //执行发送里程计数据步骤
		TIM_ClearITPendingBit(TIM7, TIM_FLAG_Update);                                      //清除中断标志位  		 
	}		 
}









