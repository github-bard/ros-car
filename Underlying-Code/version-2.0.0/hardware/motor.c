#include "motor.h"

/*********************************************** �ⲿ���� **************************************************/
extern struct PID Control_right;                                               //��ǰ��PID����
extern struct PID Control_left;                                                //��ǰ��PID����
extern struct PID Control_left1;                                               //�����PID����
extern struct PID Control_right1;                                              //�Һ���PID����


/************************************************** ���� ***************************************************/

/************************* �����ʼ�� ***************************/
void Motor_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*************** ʹ������PC0��PC1��PC2 ***************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //���Ƶ��50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOC,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);

	/*************** ʹ������PC3��PC4��PC5 ***************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //���Ƶ��50MHZ
	GPIO_Init(GPIOC,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOC,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	
	/******************* ʹ������PD6��PD7 ****************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;                               //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                              //���Ƶ��50MHZ
	GPIO_Init(GPIOD,&GPIO_InitStructure);	
	GPIO_ResetBits(GPIOD,GPIO_Pin_6|GPIO_Pin_7);
}

/****************** ��ǰ�ַ�����ٶȿ��ƺ��� *******************/
//�Ѿ����ǵ������װλ�ò�ͬ����ת��ͬ����ע������������װ˳�򣬺�ǰ�ں�
//���˳ʱ��ת���ǵ����ת�������ʱ��ת���ǵ����ת
void RightMovingSpeedW(unsigned int val)
{    
    if(val>10000)
    {   
        IN1H;
	      IN2L;
        Control_right.OwenValue=(val-10000);                                   //PID���ڵ�Ŀ�������
    }
    else if(val<10000)
    {	
        IN1L;
	      IN2H;
        Control_right.OwenValue=(10000-val);                                   //PID���ڵ�Ŀ�������	 
    }	
    else
    {
        IN1H;
	      IN2H;
        Control_right.OwenValue=0;                                             //PID���ڵ�Ŀ�������
    }												
}

/****************** ��ǰ�ַ�����ٶȿ��ƺ��� *******************/
void LeftMovingSpeedW(unsigned int val)                                        //��ǰ�ַ�����ٶȿ��ƺ���
{     
    if(val>10000)
    {  	
        IN3L;
	      IN4H;
        Control_left.OwenValue=(val-10000);                                    //PID���ڵ�Ŀ�������			
    }
    else if(val<10000)
    {  
        IN3H;
	      IN4L;
        Control_left.OwenValue=(10000-val);                                    //PID���ڵ�Ŀ�������	 
    }	
    else
    {
        IN3H;
	      IN4H;
        Control_left.OwenValue=0;                                              //PID���ڵ�Ŀ�������
    }					
}

/****************** ����ַ�����ٶȿ��ƺ��� *******************/
void LeftMovingSpeedW1(unsigned int val)                                       //����ַ�����ٶȿ��ƺ���
{     
    if(val>10000)
    {
        IN5L;
	      IN6H;
        Control_left1.OwenValue=(val-10000);                                   //PID���ڵ�Ŀ�������			
    }
    else if(val<10000)
    {
        IN5H;
	      IN6L;
        Control_left1.OwenValue=(10000-val);                                   //PID���ڵ�Ŀ�������	 
    }	
    else
    {
        IN5H;
	      IN6H;
        Control_left1.OwenValue=0;                                             //PID���ڵ�Ŀ�������
    }					
}

/****************** �Һ��ַ�����ٶȿ��ƺ��� *******************/
void RightMovingSpeedW1(unsigned int val)
{    
    if(val>10000)
    {  
        IN7H;
	      IN8L;
        Control_right1.OwenValue=(val-10000);                                  //PID���ڵ�Ŀ�������
    }
    else if(val<10000)
    {	
        IN7L;
	      IN8H;
        Control_right1.OwenValue=(10000-val);                                  //PID���ڵ�Ŀ�������	 
    }	
    else
    {
        IN7H;
	      IN8H;
        Control_right1.OwenValue=0;                                            //PID���ڵ�Ŀ�������
    }												
}

/****************** С���ٶ�ת���Ϳ��ƺ��� *******************/
void Car_Control(float rightspeed,float leftspeed,float leftspeed1,float rightspeed1)
{
    float k2=17.179;                                                           //�ٶ�ת������,ת/����	                                                                                 
    int right_speed=(int)k2*rightspeed;                                        //���Ӵ��ڽ��յ�����Ԥ�ڵ�ÿ�����ӵ����ٶ�
    int left_speed=(int)k2*leftspeed;																			  	 //ΪʲôҪ��ǿ����������ת��? 
	  int left_speed1=(int)k2*leftspeed1;
	  int right_speed1=(int)k2*rightspeed1; 
	
		RightMovingSpeedW(right_speed+10000);                                      //���A
    LeftMovingSpeedW(left_speed+10000);                                        //���B
	  LeftMovingSpeedW1(left_speed1+10000);                                      //���C
	  RightMovingSpeedW1(right_speed1+10000);                                    //���D
}
