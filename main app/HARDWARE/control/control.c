#include "control.h"
#include "usart2.h"
/**************************************************************************
 ��  �� ���������
 �Ա���ַ��https://shop119207236.taobao.com
**************************************************************************/
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ�����ʱ��ͬ��	
				 ��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
				 ���߿���imv_mpu.h�ļ���26�еĺ궨������޸�(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/
#define SPEED_Y 40 //����(ǰ��)����趨�ٶ�
#define SPEED_Z 100//ƫ��(����)����趨�ٶ� 

int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;

float Mechanical_angle=0; 
float Target_Speed=0;	//�����ٶȣ������������ڿ���С��ǰ�����˼����ٶȡ�
float Turn_Speed=0;		//�����ٶȣ�ƫ����

//��Բ�ͬ���Ͳ�������sys.h������define�ĵ������
float balance_UP_KP=BLC_KP; 	 // С��ֱ����PD����
float balance_UP_KD=BLC_KD;

float velocity_KP=SPD_KP;     // С���ٶȻ�PI����
float velocity_KI=SPD_KI;

float Turn_Kd=TURN_KD;//ת��KP��KD
float Turn_KP=TURN_KP;

u8 SR04_Counter=0;
u8 Voltage_Counter=0;

u8 Send_Count,i;
float gyrox_f, gyroy_f, gyroz_f;	
float _gyrox_kal, gyrox_kal, _Px, Px=1, Kal_x, Q_x=0.1, R_x=5;
float roll_raw=0, roll_kal=0;
float PP[2][2] = {{1,0},{0,1}};
float Q_angle = 0.0001, Q_bias = -77;
float dt = 0.00061, K_0, K_1, R_angle = 1, R_gyro = 1;
void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)
	{
		EXTI->PR=1<<5;                                           //===���LINE5�ϵ��жϱ�־λ   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������
		Encoder_Left=Read_Encoder(2);                           //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=-Read_Encoder(3);                           //===��ȡ��������ֵ
		
		roll_raw += (gyrox+78)*dt;
		roll_kal += (gyrox - Q_bias)*dt;
		PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0])*dt;
		PP[0][1] = PP[0][1] - PP[1][1]*dt;
		PP[1][0] = PP[1][0] - PP[1][1]*dt;
		PP[1][1] = PP[1][1] + 0.003;
		K_0 = PP[0][0] / (PP[0][0] + R_angle);
		K_1 = PP[1][0] / (PP[0][0] + R_angle);
		roll_kal = roll_kal + K_0 * (roll - roll_kal);
		Q_bias = Q_bias + K_1 * (roll - roll_kal);
		PP[0][0] = PP[0][0] - K_0 * PP[0][0];
		PP[0][1] = PP[0][1] - K_0 * PP[0][1];
		PP[1][0] = PP[1][0] - K_1 * PP[0][0];
		PP[1][1] = PP[1][1] - K_1 * PP[0][1];

		/*
		gyrox_f = (float) gyrox;
		gyroy_f = (float) gyroy;
		gyroz_f = (float) gyroz;
		_gyrox_kal = gyrox_kal;
		_Px = Q_x + Px;
		Kal_x = _Px/(_Px + R_x);
		gyrox_kal = _gyrox_kal + Kal_x*(gyrox_f - _gyrox_kal);
		Px = (1-Kal_x)*_Px;
		*/
		DataScope_Get_Channel_Data(roll_kal, 1 );
		DataScope_Get_Channel_Data(roll, 2 );
		DataScope_Get_Channel_Data(gyrox, 3 );
		DataScope_Get_Channel_Data(Q_bias, 4 );
		Send_Count=DataScope_Data_Generate(4);
		for( i = 0 ; i < Send_Count; i++) 
		{
			while((USART1->SR&0X40)==0);  
			USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
		
		Voltage_Counter++;
		if(Voltage_Counter==20)									 //===100ms��ȡһ�γ�����������
		{
			Voltage_Counter=0;
			Voltage=Get_battery_volt();		                         //===��ȡ��ص�ѹ
		}
		/*ǰ��*/
		switch(Mode)
		{
			case 97:
				SR04_Counter++;
				if(SR04_Counter>=20)									 //===100ms��ȡһ�γ�����������
				{
					SR04_Counter=0;
					SR04_StartMeasure();												 //===��ȡ��������ֵ
				}
				break;
			case 98://����ģʽ
				if((Fore==0)&&(Back==0))Target_Speed=0;//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
				if(Fore==1)Target_Speed--;//ǰ��1��־λ����-->��Ҫǰ��
				if(Back==1)Target_Speed++;//
				/*����*/
				if((Left==0)&&(Right==0))Turn_Speed=0;
				if(Left==1)Turn_Speed-=30;	//��ת
				if(Right==1)Turn_Speed+=30;	//��ת
				/*ת��Լ��*/
				if((Left==0)&&(Right==0))Turn_Kd=-0.6;//��������ת��ָ�����ת��Լ��
				else if((Left==1)||(Right==1))Turn_Kd=0;//������ת��ָ����յ�����ȥ��ת��Լ��
				break;
			case 99://ѭ��ģʽ
				Tracking();
				switch(TkSensor)
				{
					case 15:
						Target_Speed=0;
						Turn_Speed=0;
						break;
					case 9:
						Target_Speed--;
						Turn_Speed=0;
						break;
					case 2://����ת
						Target_Speed--;
						Turn_Speed=15;
						break;
					case 4://����ת
						Target_Speed--;
						Turn_Speed=-15;
						break;
					case 8:
						Target_Speed=-10;
						Turn_Speed=-80;
						break;
					case 1:
						Target_Speed=-10;
						Turn_Speed=80;
						break;
				}
				break;
		}
			
		Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//�޷�
		Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//�޷�( (20*100) * 100)
			
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===ֱ����PID����	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,Target_Speed);       //===�ٶȻ�PID����	 
		Turn_Pwm =Turn_UP(gyroz,Turn_Speed);        //===ת��PID����
		Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;  	            //===�������ֵ������PWM
		Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
	  Xianfu_Pwm();  																					 //===PWM�޷�
		Turn_Off(pitch,12);																 //===���Ƕ��Լ���ѹ�Ƿ�����
		Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���  
	}
}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
��    �ߣ��������
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===���ƽ��ĽǶ���ֵ�ͻ�е���
	 balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI����
��ڲ����������������ֵ
����  ֵ���ٶȿ���PWM
��    �ߣ��������
**************************************************************************/
int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right);//-target;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ� 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-gyro_Z;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===�����޷�	
		Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //===�ٶȿ���	
	  if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===����رպ��������
	  return Velocity;
}
/**************************************************************************
�������ܣ�ת��PD����
��ڲ����������������ֵ��Z����ٶ�
����  ֵ��ת�����PWM
��    �ߣ��������
**************************************************************************/

int Turn_UP(int gyro_Z, int RC)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z + Turn_KP*RC;
	return PWM_out;
}

void Tracking()
{
	TkSensor=0;
	TkSensor+=(C1<<3);
	TkSensor+=(C2<<2);
	TkSensor+=(C3<<1);
	TkSensor+=C4;
}