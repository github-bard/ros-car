#include "motor.h"

extern struct PID Control_left;//����PID�����������µ��4096
extern struct PID Control_right;//����PID�����������µ��4096


void Motor_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//���Ƶ��50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);

}

void LeftMovingSpeedW(unsigned int val)//���ַ�����ٶȿ��ƺ���
{     
    if(val>10000)
    {  
        //GPIO_SetBits(GPIOC, GPIO_Pin_0);	
        //GPIO_ResetBits(GPIOC, GPIO_Pin_1);	
        	IN1H;
	        IN2L;
        Control_left.OwenValue=(val-10000);//PID���ڵ�Ŀ�������			
    }
    else if(val<10000)
    {  
        
			  //GPIO_SetBits(GPIOC, GPIO_Pin_1);	
        //(GPIOC, GPIO_Pin_0);	
        IN1L;
	      IN2H;
        Control_left.OwenValue=(10000-val);//PID���ڵ�Ŀ�������	 
    }	
    else
    {
         //GPIO_SetBits(GPIOC, GPIO_Pin_0);	
         //GPIO_SetBits(GPIOC, GPIO_Pin_1);
         	IN1H;
	        IN2H;
          Control_left.OwenValue=0;//PID���ڵ�Ŀ�������
    }					
}

void RightMovingSpeedW(unsigned int val2)//���ַ�����ٶȿ��ƺ���
{    
    if(val2>10000)
    {  
        /* motor A ��ת*/
        GPIO_SetBits(GPIOC, GPIO_Pin_2);	
        GPIO_ResetBits(GPIOC, GPIO_Pin_3); 
        //IN3H;
	      //IN4L;
        Control_right.OwenValue=(val2-10000);//PID���ڵ�Ŀ�������
    }
    else if(val2<10000)
    {  
        /* motor A ��ת*/
         GPIO_SetBits(GPIOC, GPIO_Pin_3);	
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);	
        //IN3L;
	      //IN4H;
        Control_right.OwenValue=(10000-val2);//PID���ڵ�Ŀ�������	 
    }	
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_2);	
        GPIO_SetBits(GPIOC, GPIO_Pin_3);
        //IN3H;
	      //IN4H;
        Control_right.OwenValue=0;//PID���ڵ�Ŀ�������
    }												
}

void car_control(float rightspeed,float leftspeed)//С���ٶ�ת���Ϳ��ƺ���
{
    float k2=17.179;         //�ٶ�ת������,ת/����	
    
    //���Ӵ��ڽ��յ����ٶ�ת����ʵ�ʿ���С�����ٶȣ�����PWM��
    int right_speed=(int)k2*rightspeed;
    int left_speed=(int)k2*leftspeed;          //ΪʲôҪ��ǿ����������ת��?
    
    RightMovingSpeedW(right_speed+10000);
    LeftMovingSpeedW(left_speed+10000);
}

//void Contact_Init(void)//�����ַ�����ٶȳ�ʼ��
//{
//	LeftMovingSpeedW(12000); //���B
//	RightMovingSpeedW(12000);//���A	
//}



void M1_STOP(void)//���1ֹͣת��
 {
	IN1H;
	IN2H;
 }
 void M2_STOP(void)//���2ֹͣת��
 {
	IN3H;
	IN4H;
 }

// void M3_STOP(void)//���3ֹͣת��
// {
//	IN5H;
//	IN6H;
// }
// void M4_STOP(void)//���4ֹͣת��
// {
//	IN7H;
//	IN8H;
// }
 
 void M1_Forward(void)//���1��ת
 {
	IN1H;
	IN2L;
 }
 void M2_Forward(void)//���2��ת
 {
	IN3H;
	IN4L;
 }
// void M3_Forward(void)//���3��ת
// {
//	IN5H;
//	IN6L;
// }
// void M4_Forward(void)//���4��ת
// {
//	IN7H;
//	IN8L;
// }
 
 void M1_Reverse(void)//���1��ת
 {
	 IN1L;
	 IN2H;
 }
 void M2_Reverse(void)//���2��ת
 {
	IN3L;
	IN4H;
 }
// void M3_Reverse(void)//���3��ת
// {
//	IN5L;
//	IN6H;
// }
// void M4_Reverse(void)//���4��ת
// {
//	IN7L;
//	IN8H;
// }

 void Car_WalkStraight(void)//С��ֱ��
 {
	M1_Forward();
	M2_Forward();
//	M3_Reverse();
//	M4_Reverse();
	
 }
 void Car_WalkBack(void)//С������
 {
	M1_Reverse();
	M2_Reverse();
//	M3_Forward();
//	M4_Forward();
 }
 
 void Car_TurnLeft90(void)//С����ת
 {
	M1_Reverse();
	M2_Reverse();
//	M3_Forward();
//	M4_Forward(); 
 
 }
 void Car_TurnRight90(void)//С����ת
 {
	M1_Forward();
	M2_Forward();
//	M3_Reverse();
//	M4_Reverse();
 }
 
