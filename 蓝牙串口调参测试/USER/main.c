#include "sys.h"
/***************OLED的IO接口**************************
								GND   电源地
								VCC   接5V或3.3v电源
								SCL   接PB8（SCL）
								SDA   接PB9（SDA） 
****************OLED的IO接口**************************/

/*关于蓝牙的IO口,本测试程序的蓝牙挂载在串口3上*/

u8 recieve_bluetooth_DATA=0;		 //	蓝牙接受数据标志

float balance_UP_KP,balance_UP_KD,velocity_KP, velocity_KI, Mechanical_angle;
void data_receive3(void)
{
	u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		
		len=USART3_RX_STA&0x3f;//得到此次接收到的数据长度
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
	delay_init();	    	           //=====延时函数初始化	
	NVIC_Configuration();					 //=====中断优先级分组
	uart1_init(9600);	          	 //=====串口1初始化,与上位机连接，发送对usart3传来的数据的处理
	uart3_init(9600);							 //=====串口3初始化即蓝牙初始化
	delay_ms(100);
/*****************修改蓝牙的默认通信波特率以及蓝牙默认的名字******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAME333DayuRobot\r\n");//发送蓝牙模块指令--设置名字为：Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //发送蓝牙模块指令,将波特率设置成115200
	delay_ms(100);		
	uart3_init(115200);
/*****************************************************************************/	
	LED_Init();                    //=====初始化与 LED 连接的IO
	KEY_Init();                    //=====按键初始化
	OLED_Init();                   //=====OLED初始化
	OLED_Clear();									 //=====OLED清屏
  while(1)	
	{
			delay_ms(200);						 	
			LED=~LED;										//表明程序一直处于运行中
	} 	
}

