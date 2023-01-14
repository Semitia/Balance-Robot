#ifndef __USART_H
#define __USART_H
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
/*USART1*/
#define USART1_MAX_TX_LEN 200
#define USART1_MAX_RX_LEN 200
//#if USART1_DMA
extern u8 usart1_rxbuf[USART1_MAX_RX_LEN];						
extern u8 USART1_RX_FLAG;			
extern u8 USART1_RX_LEN;	

void DMA_USART1_Tx_Data(u8 *buffer, u32 size);
void DMA1_USART1_Init(void);


extern u8 USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         		//����״̬���	
extern int start_time, end_time;
void USART1_IRQHandler(void);     	//����1�жϷ������
//#else
//#endif
void uart1_init(u32 bound);					//����1��ʼ������
void printf1(u8 *name, float data);
void printf_s(u8 *s, u8 newline);

extern u8  USART3_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART3_RX_STA;         		//����״̬���	
void uart3_init(u32 bound);					//����1��ʼ������
void USART3_IRQHandler(void);     	//����1�жϷ������
void Uart3SendByte(char byte);   //´®¿Ú·¢ËÍÒ»¸ö×Ö½Ú
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);

int tr(char t);
float tr_s(u8 *s, int start, int num, int p);

#endif


