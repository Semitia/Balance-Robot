#include "sys.h"
/***************OLED��IO�ӿ�**************************
								GND   ��Դ��
								VCC   ��5V��3.3v��Դ
								SCL   ��PB8��SCL��
								SDA   ��PB9��SDA�� 
****************OLED��IO�ӿ�**************************/

/*����������IO��,�����Գ�������������ڴ���3��*/

u8 recieve_bluetooth_DATA=0;		 //	�����������ݱ�־

float balance_UP_KP,balance_UP_KD,velocity_KP, velocity_KI, Mechanical_angle;
void data_receive3(void)
{
	u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		
		len=USART3_RX_STA&0x3f;//�õ��˴ν��յ������ݳ���
		printf("\r\nthe data you input is:\r\n");
		for(t=0;t<len;t++)
		{
			printf("%d",USART3_RX_BUF[t]);
		}
		balance_UP_KP = (float) ( 100*(USART3_RX_BUF[0]-'0') + 10*(USART3_RX_BUF[1]-'0') + (USART3_RX_BUF[2]-'0') );
		printf("UP_KP:%.0f, ",balance_UP_KP);
		balance_UP_KD = (float)( (USART3_RX_BUF[4]-'0') + 0.1*(USART3_RX_BUF[5]-'0') + 0.01*(USART3_RX_BUF[6]-'0') );
		printf("UP_KD:%.2f, ",balance_UP_KD);
		velocity_KP = -(float)( 100*(USART3_RX_BUF[8]-'0') + 10*(USART3_RX_BUF[9]-'0') + (USART3_RX_BUF[10]-'0') );
		printf("V_KP:%.0f, ",velocity_KP);
		velocity_KI = -(float)( (USART3_RX_BUF[12]-'0') + 0.1*(USART3_RX_BUF[13]-'0') + 0.01*(USART3_RX_BUF[14]-'0') );
		printf("V_KI:%.2f, ",velocity_KI);
		Mechanical_angle = (float)( (USART3_RX_BUF[17]-'0') + 0.1*(USART3_RX_BUF[18]-'0') + 0.01*(USART3_RX_BUF[19]-'0') );
		if(USART3_RX_BUF[16]-'0' == 1) {Mechanical_angle *= -1;}
		printf("Mechanical:%.2f\r\n",Mechanical_angle);
		USART3_RX_STA=0;
	}
	return;
}

int main(void)	
{ 
	delay_init();	    	           //=====��ʱ������ʼ��	
	NVIC_Configuration();					 //=====�ж����ȼ�����
	uart1_init(9600);	          	 //=====����1��ʼ��,����λ�����ӣ����Ͷ�usart3���������ݵĴ���
	uart3_init(9600);							 //=====����3��ʼ����������ʼ��
	delay_ms(100);
/*****************�޸�������Ĭ��ͨ�Ų������Լ�����Ĭ�ϵ�����******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAME333DayuRobot\r\n");//��������ģ��ָ��--��������Ϊ��Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //��������ģ��ָ��,�����������ó�115200
	delay_ms(100);		
	uart3_init(115200);
/*****************************************************************************/	
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	KEY_Init();                    //=====������ʼ��
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
  while(1)	
	{
			delay_ms(200);						 	
			LED=~LED;										//��������һֱ����������
	} 	
}

