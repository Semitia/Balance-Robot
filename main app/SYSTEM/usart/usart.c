#include "usart.h"

#define USART1_DMA 1

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

u8 USART_RX_BUF[USART_REC_LEN];     //????,??64???.
//????
//bit7,??????
//bit6,???0x0d
//bit5~0,??????????
u8 USART_RX_STA=0;       //??????

//#if USART1_DMA
u8 USART1_TX_BUF[USART2_MAX_TX_LEN]; 	
u8 usart1_rxbuf[USART2_MAX_RX_LEN];				           		
u8 USART1_TX_FLAG=0;					
u8 USART1_RX_FLAG=0;			
u8 USART1_RX_LEN = 0;	
	
	
/**
 *@brief DMA�ʹ�������
**/
void DMA1_USART1_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						

	//DMAͨ��6�Ĵ�������Ϊȱʡ
	DMA_DeInit(DMA1_Channel5);
	//
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);	
	//���ý��ջ������׵�ַ
	DMA1_Init.DMA_MemoryBaseAddr = (u32)usart1_rxbuf;  
	//���ݴ��䷽�����赽�ڴ�
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;		
	//DMAͨ�������С
	DMA1_Init.DMA_BufferSize = USART1_MAX_RX_LEN;	
	//�����ַ����
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	
	//�ڴ��ַ����
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;					
	//���ݿ��8λ
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		
	//���ݿ��Ҳ��8λ���ڴ�
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		
	//����ģʽ
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;	
	//ͨ�����ȼ�Ϊ��
	DMA1_Init.DMA_Priority = DMA_Priority_High; 	
	//������Ϊ�ڴ浽�ڴ洫��
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;								
	//��ʼ��
	DMA_Init(DMA1_Channel5,&DMA1_Init);

	DMA_DeInit(DMA1_Channel4);											
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);					
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART1_TX_BUF;              		
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								
	DMA1_Init.DMA_BufferSize = USART1_MAX_TX_LEN;							
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									
	DMA_Init(DMA1_Channel4,&DMA1_Init); 									

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
	NVIC_Init(&NVIC_InitStructure);											

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							
	NVIC_Init(&NVIC_InitStructure);										

	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);							
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);							

	DMA_Cmd(DMA1_Channel5,ENABLE);           								
	DMA_Cmd(DMA1_Channel4,DISABLE);           								

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);        					
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);        					
}

/**
 *@brief DMA ���ͺ���
 *@param ���ݵ�ַ
 *@param ����
**/
void DMA_USART1_Tx_Data(u8 *buffer, u32 size)//��λΪ�ֽ�
{
	if(!size) {return;}
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;							
	DMA1_Channel4->CMAR  = (uint32_t)buffer;	
	DMA1_Channel4->CNDTR = size;    			
	DMA_Cmd(DMA1_Channel4, ENABLE);				
}

char line[3] = "\r\n";
void printf_s(char *s, u8 newline)
{
	char *p = s;
	uint32_t size = strlen(s);
/*if(newline)
	{
		u8 i;
		char *str = (char*)malloc(size+2);
		for(i=0; i<size; i++) 
		{str[i]=s[i];}
		strcat(str,"\r\n");
		size+=2;
		p = str;
	}*/
	DMA_USART1_Tx_Data(p,size);
	if(newline) DMA_USART1_Tx_Data(line,2);
	/*
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;			
	DMA1_Channel4->CMAR  = (uint32_t)p;	
	DMA1_Channel4->CNDTR = size;    			
	DMA_Cmd(DMA1_Channel4, ENABLE);
	*/
//	if(newline) free(p);
	return;
}

#define UP 3
#define DOWN -2
void printf_f(char *name, float data)
{
	char *str = (char*)malloc(UP+DOWN+2);
	int i,p=0;//pָ��str�ĵڼ�λ
	bool reach_flag=0;//������λ����
	printf_s(name,0);
	if(data<0) {str[p++]='-';data*=-1;}
	for(i=UP;i>=0;i--)
	{
		char num = data/(pow(10,i));
		if(!num && !reach_flag) {continue;}
		data-=num*pow(10,i);
		str[p++] = num+48;
		reach_flag=1;
	}
	if(!reach_flag) {str[p++] = '0';}
	str[p++] = '.';
	for(i=-1;i>=DOWN;i--)
	{
		char num = data/(pow(10,i));
		data-=num*pow(10,i);
		str[p++] = num+48;
	}
	str[p] = '\0';
	for(i=0;i<p;i++)
	{
		DMA_USART1_Tx_Data(&str[i],1);
	}	
	//DMA_USART1_Tx_Data(str,p);
	//printf_s(str,0);
	free(str);
	return;
}

void printf1(u8 *name, float data)
{
	while(USART1_TX_FLAG);						
	USART1_TX_FLAG=1;							
	//DMA1_Channel4->CMAR  = (uint32_t)buffer;	
	//DMA1_Channel4->CNDTR = size;    			
	//DMA_Cmd(DMA1_Channel4, ENABLE);		
}

/**
 *@brief DMA1��������ж�
**/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC4)!= RESET)	
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4); 	
		USART_ClearFlag(USART1,USART_FLAG_TC);
		DMA_Cmd(DMA1_Channel4, DISABLE );   	
		USART1_TX_FLAG=0;					
	}
}

/**
 *@brief ����1�����ж�
**/
void USART1_IRQHandler(void)                	
{
	//u8 *p;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)	
	{
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,USART_FLAG_TC);		
		DMA_Cmd(DMA1_Channel5, DISABLE );   						
		USART1_RX_LEN = USART1_MAX_RX_LEN - DMA1_Channel5->CNDTR;	
		//p = u1rxbuf;
		USART1_RX_FLAG=1;		
		DMA1_Channel5->CNDTR = USART1_MAX_RX_LEN;					
		DMA_Cmd(DMA1_Channel5, ENABLE);     						
  }
}
/*
#else //������DMA
int start_time,end_time;

void USART1_IRQHandler(void)                	//??1??????
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //????(?????????0x0d 0x0a??)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//????????
		if((USART_RX_STA&0x80)==0)//?????
		{
			if(USART_RX_STA&0x40)//????0x0d
			{
				if(Res!=0x0a) USART_RX_STA=0;//????,????
				else 
				{
					USART_RX_STA|=0x80;	//????? 
					end_time = TIM1->CNT;
				}
			}
			else //????0X0D
			{	
				if(Res==0x0d) USART_RX_STA|=0x40;
				else
				{
					if( (USART_RX_STA&0X3F) == 0) start_time = TIM1->CNT;
					USART_RX_BUF[USART_RX_STA&0X3F]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>63) USART_RX_STA=0;//??????,??????	  
				}		 
			}
		}   		 
   } 
} 

#endif
*/
void uart1_init(u32 bound)
{
	//GPIO????
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	//USART ?????
	USART_InitStructure.USART_BaudRate = bound;//?????9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
//#if USART1_DMA
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);									
	USART_ClearFlag(USART1,USART_FLAG_TC);										
	USART_Cmd(USART1, ENABLE);													
	DMA1_USART1_Init();				
//#else
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//????
//	USART_Cmd(USART1, ENABLE);                    //???? 
//#endif
}

/*USART3*/
u8 USART3_RX_BUF[USART_REC_LEN];     //���ջ���,���64���ֽ�.
//����״̬
//bit7��������ɱ�־
//bit6�����յ�0x0d
//bit5~0�����յ�����Ч�ֽ���Ŀ
u8 USART3_RX_STA=0;       //����״̬���

void uart3_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//USART3_TX   PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//USART3_RX	  PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
}

void Uart3SendByte(char byte)   //���ڷ���һ���ֽ�
{
        USART_SendData(USART3, byte);        //ͨ���⺯��  ��������
        while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
        //�ȴ�������ɡ�   ��� USART_FLAG_TC �Ƿ���1��    //���⺯�� P359 ����
}

void Uart3SendBuf(char *buf, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)Uart3SendByte(*buf++);
}

void Uart3SendStr(char *str)
{
	u16 i,len;
	len = strlen(str);
	for(i=0; i<len; i++)Uart3SendByte(*str++);
}

void USART3_IRQHandler(void)                	//����1�жϷ������
	{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
		if((USART3_RX_STA&0x80)==0)//����δ���
		{
			if(USART3_RX_STA&0x40)//���յ���0x0d
			{
				if(Res!=0x0a)USART3_RX_STA=0;//���մ���,���¿�ʼ
				else USART3_RX_STA|=0x80;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART3_RX_STA|=0x40;
				else
				{
					USART3_RX_BUF[USART3_RX_STA&0X3F]=Res ;
					USART3_RX_STA++;
					if(USART3_RX_STA>63)USART3_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
    } 
} 

int tr(char t)
{ 
    return t-48; 
}

/**
 * @brief ���ַ�����startλ��ʼ��numλת��Ϊfloat��
 * @param s �ַ���
 * @param start ��ʼλ
 * @param num λ��
 * @param p �׸����� 10^p
 * @return float 
 */
float tr_s(u8 *s, int start, int num, int p)
{
    float ans=0;
    while(num>0)
    {
        ans+= tr(s[start++])*pow(10,p--);
        num--;
    }
    return ans;
}
