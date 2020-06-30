#include "usart.h"

extern u8 main_sta;//����������ִ�б�־λ

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){};//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif

 /**
  * @brief  USART1 GPIO ����,����ģʽ���á�115200 8-N-1
  * @param  ��
  * @retval ��
  */
	//���ã��������룿
void USART1_Config(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;   //���ڶ˿����ýṹ�����
	USART_InitTypeDef    USART_InitStructure;  //���ڲ������ýṹ�����

	//��1������GPIO��USART������ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //��GPIOAʱ�Ӻ�    ����ʱ�� ûɶӰ��    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//�򿪴��ڸ���ʱ��
	USART_DeInit(USART1);  //��λ����1

	//��2������USART1 Tx�����ͽţ���GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			   //����1���ͽ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		   //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //����ٶ�50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   //��ʼ��GPIOA
														  
	//��3������USART Rx�����սţ���GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			   //����1���ս�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //��������
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������            
	GPIO_Init(GPIOA, &GPIO_InitStructure);				   //��ʼ��GPIOA

	//��4��������USART1����
	USART_InitStructure.USART_BaudRate             = 115200;							 //���������ã�115200
	USART_InitStructure.USART_WordLength           = USART_WordLength_8b;			 //����λ�����ã�8λ
	USART_InitStructure.USART_StopBits             = USART_StopBits_1;				 //ֹͣλ���ã�1λ
	USART_InitStructure.USART_Parity               = USART_Parity_No;				 //�Ƿ���żУ�飺��
	USART_InitStructure.USART_HardwareFlowControl  = USART_HardwareFlowControl_None; //Ӳ��������ģʽ���ã�û��ʹ��
	USART_InitStructure.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;	 //�����뷢�Ͷ�ʹ��
	USART_Init(USART1, &USART_InitStructure);										 //��ʼ��USART1

    //�򿪷����жϺͽ����ж�(�����Ҫ�ж�)
	 // USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  // ʹ��ָ����USART1�����ж� ��
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ʹ��ָ����USART1�����ж� ��

	//��5����ʹ�� USART1�� �������
	USART_Cmd(USART1, ENABLE);							   //ʹ�� USART1

    //�����������1���ֽ��޷���ȷ���ͳ�ȥ������
    USART_ClearFlag(USART1, USART_FLAG_TC);                //�崮��1���ͱ�־
}


u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_STA=0;   //����״̬���	
u8 serial_rec=0x31;   //���մ������ݱ���

void USART1_IRQHandler(void)//�����жϺ���
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�Ƿ���ܵ�����
    {
		//printf("yes1\r\n");
		serial_rec =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
			//printf("serial_rec:%c\r\n",serial_rec);			
		if((USART_RX_STA&0x8000)==0)//����δ���
        {
            //printf("yes2\r\n");
					if(USART_RX_STA&0x4000)//���յ���0x0d
            {//printf("yes3\r\n");
							  printf("serial_rec=%d\r\n",serial_rec);
                if(serial_rec==0x0a)
                {//printf("yes4\r\n");
                    if((USART_RX_STA&0x3f)==8)
                    {			//printf("yes5\r\n");				
                        USART_RX_STA|=0x8000;	//���������                       
											  main_sta|=0x04;
                        main_sta&=0xF7;
											  //printf("finish");
                    }
                    else
                    {//printf("yes6\r\n");
                        main_sta|=0x08;
                        main_sta&=0xFB;
                        USART_RX_STA=0;//���մ���,���¿�ʼ
                    }
                }
                else 
                {//printf("yes7\r\n");
                    main_sta|=0x08;
                    USART_RX_STA=0;//���մ���,���¿�ʼ
                }
            }
            else //��û�յ�0X0D
            {//	printf("yes8\r\n");
                if(serial_rec==0x0d)
								{
									//printf("yes9\r\n");
									USART_RX_STA|=0x4000;
								}
                else
                {//printf("yes10\r\n");
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=serial_rec ;
                    USART_RX_STA++;
									//printf("USART_RX_STA=%d\r\n",USART_RX_STA);
									
                    if(USART_RX_STA>(USART_REC_LEN-1))
                    {//printf("yes11\r\n");
											  
                        main_sta|=0x08;
                        USART_RX_STA=0;//�������ݴ���,���¿�ʼ����
                    }							
                }		 
            }
        }   		 
    }
}
 

/*********************************************END OF FILE**********************/
