#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
extern u8  USART3_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART3_RX_STA;         		//����״̬���	
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte(char byte);   //���ڷ���һ���ֽ�
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);
#endif

