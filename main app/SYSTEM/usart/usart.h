#ifndef __USART_H
#define __USART_H
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         		//����״̬���	
extern int start_time, end_time;
void uart1_init(u32 bound);					//����1��ʼ������
void USART1_IRQHandler(void);     	//����1�жϷ������

extern u8  USART3_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART3_RX_STA;         		//����״̬���	
void uart3_init(u32 bound);					//����1��ʼ������
void USART3_IRQHandler(void);     	//����1�жϷ������
#endif


