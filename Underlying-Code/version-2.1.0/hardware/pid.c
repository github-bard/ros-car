#include "pid.h"

/*************************************************** 外部引用 *************************************************/
extern int two_wheels_diff;                                                        //采集回来的左右轮速度差值

/**************************************************** 变量 ****************************************************/

float MaxValue=9;                                                       //输出最大限幅
float MinValue=-9;                                                      //输出最小限幅
float OutputValue;                                                      //PID输出暂存变量,用于积分饱和抑制

/***************************** 四轮PID参数 *****************************/
struct PID Control_left={0.01,0.1,0.75,0,0,0,0,0,0};              //左前轮A的PID参数，适于
struct PID Control_right={0.01,0.1,0.75,0,0,0,0,0,0};             //右前轮B的PID参数，适于
struct PID Control_left1={0.01,0.1,0.75,0,0,0,0,0,0};             //左后轮C的PID参数，适于
struct PID Control_right1={0.01,0.1,0.75,0,0,0,0,0,0};            //右后轮D的PID参数，适于


/**************************************************** 函数 ****************************************************/

/******************************** PID计算 ******************************/
float PID_Calculate(struct PID *Control,float CurrentValue_left )
{
	
	float Value_Kp;                                                       //比例分量
	float Value_Ki;                                                       //积分分量
	float Value_Kd;                                                       //微分分量
	
	Control->error_0 = Control->OwenValue - CurrentValue_left + 0*two_wheels_diff;   //基波分量，Control->OwenValue为想要的速度，CurrentValue_left为电机真实速度
	Value_Kp = Control->Kp * Control->error_0 ;
	Control->Sum_error += Control->error_0;     
	
  /*************** 积分饱和抑制 ***************/
  OutputValue = Control->OutputValue;
  if(OutputValue>5 || OutputValue<-5)	
  {
			Control->Ki = 0; 
  }
	
	Value_Ki = Control->Ki * Control->Sum_error;
	Value_Kd = Control->Kd * ( Control->error_0 - Control->error_1);
	Control->error_1 = Control->error_0;                                  //保存一次谐波
	Control->OutputValue = Value_Kp  + Value_Ki + Value_Kd;               //输出值计算，注意加减
	
  /******************* 限幅 *******************/
	if(Control->OutputValue > MaxValue)
			Control->OutputValue = MaxValue;
	if(Control->OutputValue < MinValue)
			Control->OutputValue = MinValue;
    
	return (Control->OutputValue) ;
}

