#include "pid.h"

/*************************************************** �ⲿ���� *************************************************/
extern int two_wheels_diff;                                                        //�ɼ��������������ٶȲ�ֵ

/**************************************************** ���� ****************************************************/

float MaxValue=9;                                                       //�������޷�
float MinValue=-9;                                                      //�����С�޷�
float OutputValue;                                                      //PID����ݴ����,���ڻ��ֱ�������

/***************************** ����PID���� *****************************/
struct PID Control_left={0.01,0.1,0.75,0,0,0,0,0,0};              //��ǰ��A��PID����������
struct PID Control_right={0.01,0.1,0.75,0,0,0,0,0,0};             //��ǰ��B��PID����������
struct PID Control_left1={0.01,0.1,0.75,0,0,0,0,0,0};             //�����C��PID����������
struct PID Control_right1={0.01,0.1,0.75,0,0,0,0,0,0};            //�Һ���D��PID����������


/**************************************************** ���� ****************************************************/

/******************************** PID���� ******************************/
float PID_Calculate(struct PID *Control,float CurrentValue_left )
{
	
	float Value_Kp;                                                       //��������
	float Value_Ki;                                                       //���ַ���
	float Value_Kd;                                                       //΢�ַ���
	
	Control->error_0 = Control->OwenValue - CurrentValue_left + 0*two_wheels_diff;   //����������Control->OwenValueΪ��Ҫ���ٶȣ�CurrentValue_leftΪ�����ʵ�ٶ�
	Value_Kp = Control->Kp * Control->error_0 ;
	Control->Sum_error += Control->error_0;     
	
  /*************** ���ֱ������� ***************/
  OutputValue = Control->OutputValue;
  if(OutputValue>5 || OutputValue<-5)	
  {
			Control->Ki = 0; 
  }
	
	Value_Ki = Control->Ki * Control->Sum_error;
	Value_Kd = Control->Kd * ( Control->error_0 - Control->error_1);
	Control->error_1 = Control->error_0;                                  //����һ��г��
	Control->OutputValue = Value_Kp  + Value_Ki + Value_Kd;               //���ֵ���㣬ע��Ӽ�
	
  /******************* �޷� *******************/
	if(Control->OutputValue > MaxValue)
			Control->OutputValue = MaxValue;
	if(Control->OutputValue < MinValue)
			Control->OutputValue = MinValue;
    
	return (Control->OutputValue) ;
}

