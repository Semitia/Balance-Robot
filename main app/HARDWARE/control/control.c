#include "control.h"

/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
        5ms��ʱ�ж���MPU6050��INT���Ŵ���
        �ϸ�֤���������ݴ����ʱ��ͬ��	
				��MPU6050�Ĳ���Ƶ�������У����ó�100HZ�����ɱ�֤6050��������10ms����һ�Ρ�
				���߿���imv_mpu.h�ļ���26�еĺ궨������޸�(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/


int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;
int oled_v_pwm, oled_up_pwm, oled_turn_pwm;

float Mechanical_angle=MECHI; 
float Target_Speed=0;	//�����ٶȣ������������ڿ���С��ǰ�����˼����ٶȡ�
float Turn_Speed=0;		//�����ٶȣ�ƫ����
float Target_Yaw=0;

//��Բ�ͬ���Ͳ�������sys.h������define�ĵ������
float balance_UP_KP=BLC_KP; 	 // С��ֱ����PD����
float balance_UP_KD=BLC_KD;

float velocity_KP=SPD_KP;     // С���ٶȻ�PI����
float velocity_KI=SPD_KI;
float velocity_KD=SPD_KD;

float Turn_Kd=TURN_KD;//ת��KP��KD
float Turn_KP=TURN_KP;
float Turn_KI=TURN_KI;

u8 SR04_Counter=0;
u8 Voltage_Counter=0;
u8 rec_data[18];
short get_aacx, get_aacy, get_aacz;
int n_cnt=0,tim_cnt[100]={0};
u8 i;

void EXTI9_5_IRQHandler(void) 
{    
	if(PBin(5)==0)
	{
		EXTI->PR=1<<5;                                           //===���LINE5�ϵ��жϱ�־λ   
		mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===�õ�ŷ���ǣ���̬�ǣ�������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===�õ�����������
		MPU_Get_Accelerometer(&get_aacx, &get_aacy, &get_aacz);
		Encoder_Left=Read_Encoder(2);                           //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		Encoder_Right=-Read_Encoder(3);                           //===��ȡ��������ֵ
		
		/*
		//����жϺ���ʵ�ʵ���Ƶ��
		n_cnt++;
		tim_cnt[n_cnt] = TIM1->CNT;
		if(n_cnt >=50 )
		{
			n_cnt = 0;
			for(i=1;i<=50;i++) {printf("%d\r\n",tim_cnt[i]);}
		}
		*/
		
		Voltage_Counter++;
		if(Voltage_Counter>=200)									 //===100ms�Ҿ������Ƕ�ȡ��ص�ѹ
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
		//Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//�޷�( (20*100) * 100)
			
		Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===ֱ����PID����	
		Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,Target_Speed);       //===�ٶȻ�PID����	 
		//Turn_Pwm =Turn_UP(gyroz,Turn_Speed);        //===ת��PID����
		Turn_Pwm = Yaw_control(gyroz,Encoder_Left,Encoder_Right);
		Moto1=Balance_Pwm-Velocity_Pwm+Turn_Pwm;  	            //===�������ֵ������PWM
		Moto2=Balance_Pwm-Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
	  Xianfu_Pwm();  																					 //===PWM�޷�
		Turn_Off(pitch,12);																 //===���Ƕ��Լ���ѹ�Ƿ�����
		Set_Pwm(Moto1,Moto2);    //===��ֵ��PWM�Ĵ���  
		//kalman();
		//print();
		oled_v_pwm = Velocity_Pwm, oled_up_pwm = Balance_Pwm, oled_turn_pwm = Turn_Pwm;
		data_receive();
		data_receive3();
	}
}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ���еƽ��Ƕȣ���е��ֵ�������ٶ�
����  ֵ��ֱ������PWM
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
**************************************************************************/
int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
	static float Velocity,Encoder_Least,Encoder_error,last_error,d_error;
	static float Encoder_Integral;
	//=============�ٶ�PI������=======================//	
	Encoder_error = (Encoder_Left+Encoder_Right)-Target_Speed;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶ� 
	//Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
	//Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
	//Encoder = Encoder_Least;
	d_error = Encoder_error - last_error;
	Encoder_Integral += Encoder_error;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	//Encoder_Integral=Encoder_Integral-gyro_Z;                       //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===�����޷�
	if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===�����޷�	
	
	Velocity=Encoder_error*velocity_KP + Encoder_Integral*velocity_KI - d_error*velocity_KD;        //===�ٶȿ���	
	if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===����رպ��������
	
	last_error = Encoder_error;
	//OLED_Num3(0,5,Encoder_Left);
	return Velocity;
}
/**************************************************************************
�������ܣ�ת��PD����
��ڲ����������������ֵ��Z����ٶ�
����  ֵ��ת�����PWM
**************************************************************************/

int Turn_UP(int gyro_Z, int RC)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z + Turn_KP*RC;
	return PWM_out;
}

int Yaw_control(int gyro_Z, int encoder_left, int encoder_right)//encoder_left_right ��ת�٣������Ǳ������ۼ�ֵ
{
	int pwm_out;
	static int error = 0, tem_yaw=0;
	static int error_sum = 0;
	tem_yaw += (encoder_left - encoder_right);
	error = tem_yaw - Target_Yaw;
	error_sum += error;
	pwm_out = Turn_KP*error  + Turn_KI*error_sum;
	if(pitch<-30||pitch>30) 			error_sum=0;
	
	return -pwm_out;
}

void Tracking()
{
	TkSensor=0;
	TkSensor+=(C1<<3);
	TkSensor+=(C2<<2);
	TkSensor+=(C3<<1);
	TkSensor+=C4;
}


float roll_raw=0, pitch_raw=0;
float roll_kal, bias = -45;
float Q_angle = 0.02,Q_bias = 0.3;
float dt = 0.00061, K_0, K_1, R_angle = 1.5, R_gyro = 1.5;

void kalman(void)
{
	float tem_accx=get_aacx, tem_accy=get_aacy, tem_accz=get_aacz;//tem��¼acc��ֵ������֮��ᱻ����	
	static float K_0, K_1;	
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	//printf("aacy:%.1f, aacz:%.1f\r\n",tem_accy,tem_accz);
	
	if(aacx<32764) aacx=tem_accx/16384.0;
	else              aacx=1-(tem_accx-49152)/16384.0;
	if(aacy<32764) aacy=tem_accy/16384.0;
	else              aacy=1-(tem_accy-49152)/16384.0;
	if(aacz<32764) aacz=tem_accz/16384.0;
	else              aacz=(tem_accz-49152)/16384.0;
	pitch_raw=(atan(aacy/aacz))*180/3.14;
	roll_raw=(atan(aacx/aacz))*180/3.14;
	if(tem_accx<32764) roll_raw = +roll_raw;
	if(tem_accx>32764) roll_raw = -roll_raw;
	if(tem_accy<32764) pitch_raw = +pitch_raw;
	if(tem_accy>32764) pitch_raw = -pitch_raw;
	
	roll_kal += (gyroy - bias) * dt; 
	PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0])*dt;
	PP[0][1] = PP[0][1] - PP[1][1]*dt;
	PP[1][0] = PP[1][0] - PP[1][1]*dt;
	PP[1][1] = PP[1][1] + Q_bias;
	K_0 = PP[0][0] / (PP[0][0] + R_angle);
	K_1 = PP[1][0] / (PP[0][0] + R_angle);
	roll_kal = roll_kal + K_0 * (roll_raw - roll_kal);
	bias = bias + K_1 * (roll_raw - roll_kal);
	PP[0][0] = PP[0][0] - K_0 * PP[0][0];
	PP[0][1] = PP[0][1] - K_0 * PP[0][1];
	PP[1][0] = PP[1][0] - K_1 * PP[0][0];
	PP[1][1] = PP[1][1] - K_1 * PP[0][1];
	
}

void print(void)
{
	u8 Send_Count,i;
	DataScope_Get_Channel_Data(-roll_kal, 1);
	DataScope_Get_Channel_Data(-roll_raw, 2);
	DataScope_Get_Channel_Data(pitch, 3);	

	Send_Count=DataScope_Data_Generate(4);
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}	
}

void data_receive(void)
{
	if(USART_RX_STA&0x80)
	{					   
		//printf("%d, %d\r\n", start_time, end_time);
		balance_UP_KP = (float) ( 100*(USART_RX_BUF[0]-'0') + 10*(USART_RX_BUF[1]-'0') + (USART_RX_BUF[2]-'0') );
		printf("UP_KP:%.0f, ",balance_UP_KP);
		balance_UP_KD = (float)( (USART_RX_BUF[4]-'0') + 0.1*(USART_RX_BUF[5]-'0') + 0.01*(USART_RX_BUF[6]-'0') );
		printf("UP_KD:%.2f, ",balance_UP_KD);
		velocity_KP = (float)( 100*(USART_RX_BUF[8]-'0') + 10*(USART_RX_BUF[9]-'0') + (USART_RX_BUF[10]-'0') );
		printf("V_KP:%.0f, ",velocity_KP);
		velocity_KI = (float)( (USART_RX_BUF[12]-'0') + 0.1*(USART_RX_BUF[13]-'0') + 0.01*(USART_RX_BUF[14]-'0') );
		printf("V_KI:%.2f, ",velocity_KI);
		Mechanical_angle = (float)( (USART_RX_BUF[17]-'0') + 0.1*(USART_RX_BUF[18]-'0') + 0.01*(USART_RX_BUF[19]-'0') );
		if(USART_RX_BUF[16]-'0' == 1) {Mechanical_angle *= -1;}
		printf("Mechanical:%.2f, ",Mechanical_angle);
		velocity_KD = (float)( 10*(USART_RX_BUF[21]-'0') + (USART_RX_BUF[22]-'0') + 0.1*(USART_RX_BUF[23]-'0') );
		printf("V_KD:%.1f, ",velocity_KD);
		Turn_KP = (float)( (USART_RX_BUF[25]-'0') + 0.1*(USART_RX_BUF[26]-'0') + 0.01*(USART_RX_BUF[27]-'0') );
		printf("Turn_KP:%.2f, ",Turn_KP);
		Turn_KI = (float)( (USART_RX_BUF[29]-'0') + 0.1*(USART_RX_BUF[30]-'0') + 0.01*(USART_RX_BUF[31]-'0') );
		printf("Turn_KI:%.2f\r\n",Turn_KI);
		
		USART_RX_STA=0;
	}
	return;
}

void data_receive3(void)
{
	//u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		
	//len=USART3_RX_STA&0x3f;//�õ��˴ν��յ������ݳ���

		//printf("%d, %d\r\n", start_time, end_time);
		balance_UP_KP = (float) ( 100*(USART3_RX_BUF[0]-'0') + 10*(USART3_RX_BUF[1]-'0') + (USART3_RX_BUF[2]-'0') );
		printf("UP_KP:%.0f, ",balance_UP_KP);
		balance_UP_KD = (float)( (USART3_RX_BUF[4]-'0') + 0.1*(USART3_RX_BUF[5]-'0') + 0.01*(USART3_RX_BUF[6]-'0') );
		printf("UP_KD:%.2f, ",balance_UP_KD);
		velocity_KP = (float)( 100*(USART3_RX_BUF[8]-'0') + 10*(USART3_RX_BUF[9]-'0') + (USART3_RX_BUF[10]-'0') );
		printf("V_KP:%.0f, ",velocity_KP);
		velocity_KI = (float)( (USART3_RX_BUF[12]-'0') + 0.1*(USART3_RX_BUF[13]-'0') + 0.01*(USART3_RX_BUF[14]-'0') );
		printf("V_KI:%.2f, ",velocity_KI);
		Mechanical_angle = (float)( (USART3_RX_BUF[17]-'0') + 0.1*(USART3_RX_BUF[18]-'0') + 0.01*(USART3_RX_BUF[19]-'0') );
		if(USART3_RX_BUF[16]-'0' == 1) {Mechanical_angle *= -1;}
		printf("Mechanical:%.2f, ",Mechanical_angle);
		velocity_KD = (float)( 10*(USART3_RX_BUF[21]-'0') + (USART3_RX_BUF[22]-'0') + 0.1*(USART3_RX_BUF[23]-'0') );
		printf("V_KD:%.1f, ",velocity_KD);
		Turn_KP = (float)( (USART3_RX_BUF[25]-'0') + 0.1*(USART3_RX_BUF[26]-'0') + 0.01*(USART3_RX_BUF[27]-'0') );
		printf("Turn_KP:%.2f, ",Turn_KP);
		Turn_KI = (float)( (USART3_RX_BUF[29]-'0') + 0.1*(USART3_RX_BUF[30]-'0') + 0.01*(USART3_RX_BUF[31]-'0') );
		printf("Turn_KI:%.2f\r\n",Turn_KI);
		
		USART3_RX_STA=0;
	}
	return;
}
	



