#include "usart.h"
#include "odometer.h"

/**************************************** ����ר������ ********************************************************/

/**************** ��׼����Ҫ��֧�ֺ��� ****************/
#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 

}; 
FILE __stdout;

/******* ����_sys_exit()�Ա���ʹ�ð�����ģʽ **********/   
void _sys_exit(int x) 
{ 
	x = x; 
} 

/****************** �ض���fputc���� *******************/
int fputc(int ch, FILE *f)
{      
	while((USART3->SR&0X40)==0){};                                                   //ѭ������,ֱ���������   
    USART3->DR = (u8) ch;      
	return ch;
}
#endif

/********************************************************** �ⲿ���� ************************************************************/

/**************** ��̼����� *****************/
//odometer.c�ж���
extern float position_x;
extern float position_y;
extern float oriention;
extern float velocity_linear;
extern float velocity_angular;

/** һ��ʱ�����������������ڴ��ݸ���̼� ***/
//encoder.c�ж��壬��ֵ
extern float Odometer_L_CNT;
extern float Odometer_R_CNT; 
extern float Odometer_L1_CNT;
extern float Odometer_R1_CNT;


/********************************************************** ���� ***************************************************************/

/*********** ���͸����ڵ���̼����� **********/
char odometer_data[21]={0};

/********** У����̼��� *********/
float odometer_right=0;
float odometer_left=0;
float odometer_right1=0;
float odometer_left1=0;

/************** ִ�б�־λ ******************/
u8 main_sta=0;       //��������USART3_handle��������if��ȥ�������flag��1��ӡ��̼ƣ���2���ü�����̼����ݺ�������3���ڽ��ճɹ�����4���ڽ���ʧ�ܣ�

/************ ���ڽ�����ر��� **************/
u8  USART_RX_BUF[USART_REC_LEN];                                                    //���ջ���,���USART_REC_LEN 200���ֽ�
u16 USART_RX_STA=0;                                                                 //����״̬���λ	
u8  serial_rec=0x31;                                                                //���մ������ݱ���

/******************************************************** ������ ***************************************************************/

/************ �Ӵ��ڽ��������ٶ� ************/
//��Ҫ�����������ݸ�odometer_**
union recieve_data                             
{
	float recieve_float;                                    
	unsigned char recieve_char[4];
}leftdata,rightdata,leftdata1,rightdata1; 

/********* ��ʱ��ȡ��̼����ݹ����� *********/
//��ת������������̼����ݣ�����󴫵ݸ�odometer_data[21]
union odometer
{
	float odometer_float;
	unsigned char odometer_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     //Ҫ��������̼����ݣ��ֱ�ΪX��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�


/************************************************** ���� *****************************************************/

/******************************* ����3��ʼ�� ***************************/
//ʹ��PB10��PB11
void USART3_Init(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;                                         //���ڶ˿����ýṹ�����
	USART_InitTypeDef    USART_InitStructure;                                        //���ڲ������ýṹ�����

	/*********** ��1������GPIO��USART������ʱ�� ***********/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	                         //�򿪴���3ʱ��
	USART_DeInit(USART3);                                                            //��λ����3

	/* ��2������USART3 Tx�����ͽţ���GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			                                 //����3���ͽ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		                               //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	                               //����ٶ�50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				                                   //��ʼ��GPIOB
														  
	/* ��3������USART Rx�����սţ���GPIO����Ϊ��������ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			                                 //����3���ս�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	                                   //��������            
	GPIO_Init(GPIOB, &GPIO_InitStructure);				                                   //��ʼ��GPIOB

	/************** ��4��������USART3���� *******************/
	USART_InitStructure.USART_BaudRate           = 115200;							             //���������ã�115200
	USART_InitStructure.USART_WordLength         = USART_WordLength_8b;			         //����λ�����ã�8λ
	USART_InitStructure.USART_StopBits           = USART_StopBits_1;				         //ֹͣλ���ã�1λ
	USART_InitStructure.USART_Parity             = USART_Parity_No;				           //�Ƿ���żУ�飺��
	USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;   //Ӳ��������ģʽ���ã�û��ʹ��
	USART_InitStructure.USART_Mode               = USART_Mode_Rx | USART_Mode_Tx;	   //�����뷢�Ͷ�ʹ��
	USART_Init(USART3, &USART_InitStructure);										                     //��ʼ��USART3

  /*********** �򿪷����жϺͽ����ж�(�����Ҫ�ж�) ******/
	//USART_ITConfig(USART3, USART_IT_TXE, ENABLE);                                  //ʹ��ָ����USART3�����ж�
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                   //ʹ��ָ����USART3�����ж�

	/************ ��5����ʹ�� USART3�� ������� ************/
	USART_Cmd(USART3, ENABLE); 

  /***** �����������1���ֽ��޷���ȷ���ͳ�ȥ������ *****/
  USART_ClearFlag(USART3, USART_FLAG_TC);                                          //��մ���3���ͱ�־λ
}

/******************************** ����3�жϺ��� **************************/
//ͨ�����ڽ��ձ�־λ�Լ����ݱ�־λ���жϽ����Ƿ���ɣ�ͬʱ�ı�main_staִֵ�в�ͬ����
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                            //�ж��Ƿ���ܵ�����
  {
		serial_rec =USART_ReceiveData(USART3);                                         //��ȡ���յ�������		
		if((USART_RX_STA&0x8000)==0)                                                   //����δ���
    {
			if(USART_RX_STA&0x4000)                                                      //���յ���0x0D
      {
				if(serial_rec==0x0a)
        {
					if((USART_RX_STA&0x3f)==16)                                              //������Ч�ֽ���16
          {				
            USART_RX_STA|=0x8000;	                                                 //���������                       
						main_sta|=0x04;
						main_sta&=0xF7;
          }
          else
          {
            main_sta|=0x08;
            main_sta&=0xFB;
            USART_RX_STA=0;                                                        //���մ���,���¿�ʼ
          }
        }
        else 
        {
          main_sta|=0x08;
					USART_RX_STA=0;                                                          //���մ���,���¿�ʼ
        }
      }
      else                                                                         //��û�յ�0X0D
      {
				if(serial_rec==0x0d)
				{
					USART_RX_STA|=0x4000;
				}
        else
        {
					USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
          USART_RX_STA++;									
          if(USART_RX_STA>(USART_REC_LEN-1))
          { 
						main_sta|=0x08;
						USART_RX_STA=0;                                                        //�������ݴ���,���¿�ʼ����
          }							
        }		 
      }
   }   		 
 }
}

/******************************** ����3���մ����� *************************/
void USART3_Handle(void)
{
		/************ ���� ************/
	  u8 i=0;
		/************ 1��ִ�з�����̼����ݲ��� *****************/
		if(main_sta&0x01)                                          
		{
        /************** ��̼����ݻ�ȡ *************/
				x_data.odometer_float      = position_x;                 //��λmm
				y_data.odometer_float      = position_y;                 //��λmm
				theta_data.odometer_float  = oriention;                  //��λrad
				vel_linear.odometer_float  = velocity_linear;            //��λmm/s
				vel_angular.odometer_float = velocity_angular;           //��λrad/s
        
		   	//printf("posintion_x:%f\r\n",position_x);
		  	//printf("x_data.odometer_float:%f\r\n",x_data.odometer_float);
			
        /* ����ȡ����̼��������ַ���ʽ�浽Ҫ���͵����� */
				for(i=0;i<4;i++)
				{
						odometer_data[i]    = x_data.odometer_char[i];
						odometer_data[i+4]  = y_data.odometer_char[i];
						odometer_data[i+8]  = theta_data.odometer_char[i];
						odometer_data[i+12] = vel_linear.odometer_char[i];
						odometer_data[i+16] = vel_angular.odometer_char[i];
				}      
				odometer_data[20]='\n';                                  //��ӽ�����
				
		    /************** �������ݵ����� *************/
				for(i=0;i<21;i++)
				{
						USART_ClearFlag(USART3,USART_FLAG_TC);               //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
						USART_SendData(USART3,odometer_data[i]);             //����һ���ֽڵ�����	
						while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	                           //�ȴ����ͽ���			
				}           
				main_sta&=0xFE;                                          //����־λ����
		}
		
		/************** 2��ִ�м�����̼����ݲ��� ***************/
		if(main_sta&0x02)
		{
				odometer(Odometer_R_CNT,Odometer_L_CNT,Odometer_L1_CNT,Odometer_R1_CNT);       //������̼�        
				main_sta&=0xFD;                                          //����־λ����
		} 
		
		/************** 3��������ָ��û����ȷ���� ************/
		if(main_sta&0x08)        
		{
				USART_ClearFlag(USART3,USART_FLAG_TC);                   //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����
        for(i=0;i<3;i++)
        {
            USART_SendData(USART3,0x00);	
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        }		
        USART_SendData(USART3,'\n');	
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	
        main_sta&=0xF7;                                           //��־λ����
		}
		
		/***************** 4�����ڽ��պ��� **********************/
		if(USART_RX_STA&0x8000)
		{			
        for(i=0;i<4;i++)
        {
				/************** ���������ٶ� *************/
            rightdata.recieve_char[i]  = USART_RX_BUF[i];
            leftdata.recieve_char[i]   = USART_RX_BUF[i+4];
						leftdata1.recieve_char[i]  = USART_RX_BUF[i+4];
						rightdata1.recieve_char[i] = USART_RX_BUF[i];
        }
        
				/************** ���������ٶ� *************/                        // ???�ַ�ֱ�ӳˣ�
				rightdata.recieve_float  = rightdata.recieve_char[0]*100+rightdata.recieve_char[1]*10+rightdata.recieve_char[2]+rightdata.recieve_char[3]*0.1;
				leftdata.recieve_float   = leftdata.recieve_char[0]*100+leftdata.recieve_char[1]*10+leftdata.recieve_char[2]+leftdata.recieve_char[3]*0.1;
				leftdata1.recieve_float  = leftdata1.recieve_char[0]*100+leftdata1.recieve_char[1]*10+leftdata1.recieve_char[2]+leftdata1.recieve_char[3]*0.1;
				rightdata1.recieve_float = rightdata1.recieve_char[0]*100+rightdata1.recieve_char[1]*10+rightdata1.recieve_char[2]+rightdata1.recieve_char[3]*0.1;
				
        odometer_right  = rightdata.recieve_float;                  //У����̼���
        odometer_left   = leftdata.recieve_float;         	                   	 
        odometer_left1  = leftdata1.recieve_float;                              
				odometer_right1 = rightdata1.recieve_float;                             
				
				USART_RX_STA=0;                                             //������ձ�־λ					  
		}
}












