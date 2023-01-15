#include "sys.h"
/****************************ȫ�ֱ���*************************************/  
state_t past,next,now;

float Voltage;  														 //��ص�ѹ������صı���
float pitch,roll,yaw; 								  					 //ŷ����(��̬��)
float aacx,aacy,aacz;													 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;											 	 //������ԭʼ����

float SR04_Distance;                 									 //���������

int   Encoder_Left,Encoder_Right;         		 //���ұ��������������
int 	Moto1=0,Moto2=0;												 //������������ո��������PWM

int Velocity=0,Turn=0;
u8 Mode=0; //0��ֹ��1�ٶȿ��ƣ�2λ�ÿ��ƣ�3λ�ÿ��ƣ����ϣ�
int Uart_Receive=0;
u8 key=0;								 									 //�����ļ�ֵ
u8 TkSensor=0;
/***********************************************************************/
u8 buffer[USART2_MAX_TX_LEN]="Oh, nice to meet you!!\r\n";
u32 buf_size;

int main(void)	
{ 
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	PlugIn_Init();								 //=====��ʼ���� USB ���ӵ�IO
	KEY_Init();                    //=====��ʼ���밴�����ӵ�IO
	delay_init();	    	           //=====��ʱ������ʼ��	
	uart1_init(115200);	           //=====����1��ʼ��		
	DMA1_USART2_Init( );
	uart3_init(9600);
	delay_ms(100);
/*****************�޸�������Ĭ��ͨ�Ų������Լ�����Ĭ�ϵ�����*****************
	Uart2SendStr("AT\r\n");
	Uart2SendStr("AT+NAME333DayuRobot\r\n");//��������ģ��ָ��--��������
	delay_ms(100);	
	Uart2SendStr("AT+BAUD8\r\n"); 		 //��������ģ��ָ��,�����������ó�115200
	delay_ms(100);		
	uart2_init(115200);
	uart3_init(115200);
****************************************************************************/	
	/*****************�޸�������Ĭ��ͨ�Ų������Լ�����Ĭ�ϵ�����******************/
	Uart3SendStr("AT\r\n");
	Uart3SendStr("AT+NAMEAnya!\r\n");//��������ģ��ָ��--��������Ϊ��Bliz
	delay_ms(100);	
	Uart3SendStr("AT+BAUD8\r\n"); 		 //��������ģ��ָ��,�����������ó�115200
	delay_ms(100);		
	uart3_init(115200);
/*****************************************************************************/	
	Adc_Init();                    //=====��ʼ��ADC
	//SR04_Configuration();
	Encoder_Init_TIM2();           //=====��ʼ��������2
	Encoder_Init_TIM3();
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
	MPU_Init();					    			 //=====��ʼ��MPU6050
	mpu_dmp_init();								 //=====��ʼ��MPU6050��DMPģʽ					 
	TIM1_PWM_Init(7199,0);   			 //=====��ʼ��PWM 10KHZ,������������� 
	//TIM1_PWM_Init(10000,7199);
	delay_ms(1000);								 //=====��ʱ1s ���С���ϵ�������ת������
	Motor_Init();									 //=====��ʼ���������ӵ�Ӳ��IO�ӿ� 
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	NVIC_Configuration();					 //=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	oled_first_show();					   //ֻ��Ҫ��ʾһ�ε��ַ�,�ڴ�ˢ��һ�μ��ɡ�
	Timer4_Init(5000,7199);	    	 //=====��������ʱ����ʼ��
	//Initial_UART2(9600);
	//DMA1_USART2_Init( );
	while(1)	
	{
		oled_show();

		buf_size = sizeof(buffer);
		sendmsg();
		//DMA_USART2_Tx_Data(buffer,buf_size);
		//data_receive2();
		//delay_ms(500); //20HZ����ʾƵ�ʣ���Ļ����ʱ��ˢ�¡�
		
		
		LED = !LED;
	}
}


void Tracking_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��PA�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;	           //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��PA�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;	           //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
} 
