 /**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
#include "sys.h"
/****************************ȫ�ֱ���*************************************/    
float Voltage;  															 //��ص�ѹ������صı���
float pitch,roll,yaw; 								  			 //ŷ����(��̬��)
short aacx,aacy,aacz;													 //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;											 //������ԭʼ����
float SR04_Distance;                 //���������

int   Encoder_Left,Encoder_Right;         		 //���ұ��������������
int 	Moto1=0,Moto2=0;												 //������������ո��������PWM

int Velocity=0,Turn=0;
u8 Mode=97; //97���������ϣ�98������99����ѭ����100 PS2
int Uart_Receive=0;
u8 key=0;								 									 //�����ļ�ֵ
u8 TkSensor=0;
/***********************************************************************/
int main(void)	
{ 
	LED_Init();                    //=====��ʼ���� LED ���ӵ�IO
	PlugIn_Init();										 //=====��ʼ���� USB ���ӵ�IO
	KEY_Init();                    //=====��ʼ���밴�����ӵ�IO
	delay_init();	    	           //=====��ʱ������ʼ��	
	uart1_init(115200);	          	 //=====����1��ʼ��
	uart2_init(9600);							 //=====����2��ʼ����������ʼ��
	delay_ms(100);
/*****************�޸�������Ĭ��ͨ�Ų������Լ�����Ĭ�ϵ�����******************
	Uart2SendStr("AT\r\n");
	Uart2SendStr("AT+NAME333DayuRobot\r\n");//��������ģ��ָ��--��������
	delay_ms(100);	
	Uart2SendStr("AT+BAUD8\r\n"); 		 //��������ģ��ָ��,�����������ó�115200
	delay_ms(100);		
//	uart2_init(115200);
*****************************************************************************/	
	NVIC_Configuration();					 //=====�ж����ȼ�����,���а��������е��ж����ȼ�������,��������һ�����޸ġ�
	Adc_Init();                    //=====��ʼ��ADC
	SR04_Configuration();
	Encoder_Init_TIM2();           //=====��ʼ��������2
	Encoder_Init_TIM3();
	OLED_Init();                   //=====OLED��ʼ��
	OLED_Clear();									 //=====OLED����
	MPU_Init();					    			 //=====��ʼ��MPU6050
	mpu_dmp_init();								 //=====��ʼ��MPU6050��DMPģʽ					 
	TIM1_PWM_Init(7199,0);   			 //=====��ʼ��PWM 10KHZ,������������� 
	delay_ms(1000);								 //=====��ʱ1s ���С���ϵ�������ת������
	Motor_Init();									 //=====��ʼ���������ӵ�Ӳ��IO�ӿ� 
	MPU6050_EXTI_Init();					 //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	oled_first_show();					   //ֻ��Ҫ��ʾһ�ε��ַ�,�ڴ�ˢ��һ�μ��ɡ�
	Timer4_Init(5000,7199);	    	 //=====��������ʱ����ʼ��
  while(1)	
	{
		oled_show();
		delay_ms(50); //20HZ����ʾƵ�ʣ���Ļ����ʱ��ˢ�¡�
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
