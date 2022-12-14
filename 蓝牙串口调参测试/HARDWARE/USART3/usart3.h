#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
extern u8  USART3_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8 USART3_RX_STA;         		//接收状态标记	
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void Uart3SendByte(char byte);   //串口发送一个字节
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);
#endif

