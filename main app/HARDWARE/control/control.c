#include "control.h"

/**************************************************************************
函数功能：所有的控制代码都在这里面
        5ms定时中断由MPU6050的INT引脚触发
        严格保证采样和数据处理的时间同步	
				在MPU6050的采样频率设置中，设置成100HZ，即可保证6050的数据是10ms更新一次。
				读者可在imv_mpu.h文件第26行的宏定义进行修改(#define DEFAULT_MPU_HZ  (100))
**************************************************************************/


int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Turn_Kp;
int oled_up_pwm, oled_turn_pwm;
float oled_v,oled_p,oled_v_I;


float Mechanical_angle=MECHI; 
float target_speed=0;	//期望速度（俯仰）。用于控制小车前进后退及其速度。
float Turn_Speed=0;		//期望速度（偏航）
float Target_Yaw=0;
float target_angle = MECHI;
int target_position=0;

//针对不同车型参数，在sys.h内设置define的电机类型
float balance_UP_KP=BLC_KP; 	 // 小车直立环PD参数
float balance_UP_KD=BLC_KD;

float velocity_KP=SPD_KP;     // 小车速度环PI参数
float velocity_KI=SPD_KI;
float velocity_KD=SPD_KD;

float Turn_Kd=TURN_KD;//转向环KP、KD
float Turn_KP=TURN_KP;
float Turn_KI=TURN_KI;

float Position_KP=POSI_KP;

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
		EXTI->PR=1<<5;                                           //===清除LINE5上的中断标志位   
		//mpu_dmp_get_data(&pitch,&roll,&yaw);										 //===得到欧拉角（姿态角）的数据
		//MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);								 //===得到陀螺仪数据
		//MPU_Get_Accelerometer(&get_aacx, &get_aacy, &get_aacz);
		//Encoder_Left=Read_Encoder(2);                           //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		//Encoder_Right=-Read_Encoder(3);                           //===读取编码器的值
		
		state_update();

		/*
		//检测中断函数实际调用频率
		n_cnt++;
		tim_cnt[n_cnt] = TIM1->CNT;
		if(n_cnt >=50 )
		{
			n_cnt = 0;
			for(i=1;i<=50;i++) {printf("%d\r\n",tim_cnt[i]);}
		}
		*/
	
		Voltage_Counter++;
		if(Voltage_Counter>=200)									 //===100ms我觉得这是读取电池电压
		{
			Voltage_Counter=0;
			Voltage=Get_battery_volt();		                         //===读取电池电压
		}
		
		
		//target_speed=target_speed>SPEED_Y?SPEED_Y:(target_speed<-SPEED_Y?(-SPEED_Y):target_speed);//限幅
		//Turn_Speed=Turn_Speed>SPEED_Z?SPEED_Z:(Turn_Speed<-SPEED_Z?(-SPEED_Z):Turn_Speed);//限幅( (20*100) * 100)
		
		//target_speed = anya_position();
		target_angle = (float) anya_velocity() + MECHI;
		Balance_Pwm  = anya_balance();
		Turn_Pwm = anya_yaw();

		//Balance_Pwm =balance_UP(pitch,Mechanical_angle,gyroy);   //===直立环PID控制	
		//Velocity_Pwm=velocity(Encoder_Left,Encoder_Right,target_speed);       //===速度环PID控制	 
		//Turn_Pwm =Turn_UP(gyroz,Turn_Speed);        //===转向环PID控制
		//Turn_Pwm = Yaw_control(gyroz,Encoder_Left,Encoder_Right);
		Moto1=Balance_Pwm+Turn_Pwm;  	            //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Turn_Pwm;                 //===计算右轮电机最终PWM
	  Xianfu_Pwm();  																					 //===PWM限幅
		Turn_Off(now.pitch,Voltage);																 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);    //===赋值给PWM寄存器  
		//kalman();
		//print();
		oled_v = target_angle, oled_up_pwm = Balance_Pwm, oled_p = target_speed;
		data_receive();
		data_receive3();
	}
}


void int_limit(int *x, float range)
{
	if(*x>range)  {*x = range;}
	if(*x<-range) {*x = -range;}
	return;
}

void float_limit(float *x, float range)
{
	if(*x>range)  {*x = range;}
	if(*x<-range) {*x = -range;}
	return;
}

int anya_balance(void)
{
	/*
	float error;
	int output;
	error = now.pitch - target_angle;
	output = balance_UP_KP*error + balance_UP_KD*(now.gyroy);
	*/
	return balance_UP_KP*(now.pitch - target_angle) + balance_UP_KD*(now.gyroy);
}

float anya_velocity(void)
{
	static float last_filted=0, last_v_err=0, v_error_sum=0,last_speed=0;
	int raw_v = (int) now.v;
	float filted = 0.3*raw_v + 0.7*last_filted;
	float error  = filted - target_speed;
	float d_v_err;

	if(last_speed != target_speed) {v_error_sum = 0;}
	v_error_sum += error;
	d_v_err = error - last_v_err;
	
	if(v_error_sum>10000)		v_error_sum = 10000;             //===积分限幅
	if(v_error_sum<-10000)		v_error_sum = -10000;            //===积分限幅	
	if(pitch<-40||pitch>40)		v_error_sum = 0;     			 //===电机关闭后清除积分
	
	last_speed = target_speed;
	last_filted = filted;
	last_v_err = error;
	oled_v_I = v_error_sum;
	return (velocity_KP*error + velocity_KI*v_error_sum - velocity_KD*d_v_err)/200;
}

int anya_yaw(void)//encoder_left_right 是转速，而不是编码器累加值
{
	int pwm_out;
	static int error = 0, tem_yaw=0;
	static int error_sum = 0;
	tem_yaw += (now.v_left - now.v_right);
	error = tem_yaw - Target_Yaw;
	error_sum += error;
	pwm_out = Turn_KP*error  + Turn_KI*error_sum;
	if(pitch<-30||pitch>30) 			error_sum=0;
	
	return -pwm_out;
}

float anya_position(void)
{
	static int position = 0;
	float output;
	position += Encoder_Left + Encoder_Right;
	output = Position_KP*(target_position - position);
	output = output>SPEED_Y?SPEED_Y:(output<-SPEED_Y?(-SPEED_Y):output);//限幅
	return output;
}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、机械平衡角度（机械中值）、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle-Mechanical_balance;    							 //===求出平衡的角度中值和机械相关
	balance=balance_UP_KP*Bias+balance_UP_KD*Gyro;  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}

/**************************************************************************
函数功能：速度PI控制
入口参数：电机编码器的值
返回  值：速度控制PWM
**************************************************************************/
float filt_velocity;     //滤波后的速度
float last_filt_velocity;//上一次的滤波后的速度
float velocity_sum=0;    //速度的累加
float target_velocity=0, velocity_error, d_velocity_error, last_v_error;
int velocity(int encoder_left,int encoder_right,int gyro_Z)
{  
	/*
	static float Velocity,Encoder_Least,Encoder_error,last_error,d_error;
	static float Encoder_Integral;
	//=============速度PI控制器=======================//	
	Encoder_error = (Encoder_Left+Encoder_Right)-Target_Speed;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度 
	//Encoder *= 0.8;		                                                //===一阶低通滤波器       
	//Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
	//Encoder = Encoder_Least;
	d_error = Encoder_error - last_error;
	Encoder_Integral += Encoder_error;                                       //===积分出位移 积分时间：10ms
	//Encoder_Integral=Encoder_Integral-gyro_Z;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
	if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
	if(pitch<-40||pitch>40) 			Encoder_Integral=0;     						//===电机关闭后清除积分
	
	Velocity=Encoder_error*velocity_KP + Encoder_Integral*velocity_KI - d_error*velocity_KD;        //===速度控制	
	
	last_error = Encoder_error;
	//OLED_Num3(0,5,Encoder_Left);
	return Velocity;
	*/
	int raw_velocity = (encoder_left + encoder_right)/2;
	filt_velocity = 0.3*raw_velocity + 0.7*last_filt_velocity;
	velocity_error  = filt_velocity - target_velocity;
	velocity_sum += velocity_error;
	d_velocity_error = velocity_error - last_v_error;
	
	if(velocity_sum>10000)  	velocity_sum=10000;             //===积分限幅
	if(velocity_sum<-10000)		velocity_sum=-10000;            //===积分限幅	
	if(pitch<-40||pitch>40) 			velocity_sum=0;     						//===电机关闭后清除积分
	last_filt_velocity = filt_velocity;
	
	last_v_error = velocity_error;
	return velocity_KP*velocity_error + velocity_KI*velocity_sum - velocity_KD*d_velocity_error;
}
/**************************************************************************
函数功能：转向PD控制
入口参数：电机编码器的值、Z轴角速度
返回  值：转向控制PWM
**************************************************************************/

int Turn_UP(int gyro_Z, int RC)
{
	int PWM_out;
	
	PWM_out=Turn_Kd*gyro_Z + Turn_KP*RC;
	return PWM_out;
}

int Yaw_control(int gyro_Z, int encoder_left, int encoder_right)//encoder_left_right 是转速，而不是编码器累加值
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
/*
*/
void position_control()
{
	static int position = 0;
//	int error;
	position += Encoder_Left + Encoder_Right;
	target_velocity = Position_KP*(target_position - position);
	return;
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
	float tem_accx=get_aacx, tem_accy=get_aacy, tem_accz=get_aacz;//tem记录acc的值，后者之后会被更新	
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
		target_speed = (float)( 100*(USART_RX_BUF[34]-'0') + 10*(USART_RX_BUF[35]-'0') + (USART_RX_BUF[36]-'0') );
		if(USART_RX_BUF[33]-'0' == 1) {target_speed *= -1;}
		printf("speed:%.0f, ",target_speed);
		Position_KP = (float)( 1*(USART_RX_BUF[38]-'0') + 0.1*(USART_RX_BUF[39]-'0') + 0.01*(USART_RX_BUF[40]-'0') );
		printf("P_KP:%.2f\r\n",Position_KP);
		USART_RX_STA=0;
	}
	return;
}

void data_receive2(void)
{
	//u8 ACK_u8[200],i;
	char ACK[200];
	//u32 ACK_size;
	if(!USART2_RX_FLAG)
	{return;}
	USART2_RX_FLAG = 0;
	
	balance_UP_KP = (float) ( 100*(u1rxbuf[0]-'0') + 10*(u1rxbuf[1]-'0') + (u1rxbuf[2]-'0') );
	sprintf(ACK, "UP_KP:%.0f, ",balance_UP_KP);
	balance_UP_KD = (float)( (u1rxbuf[4]-'0') + 0.1*(u1rxbuf[5]-'0') + 0.01*(u1rxbuf[6]-'0') );
	sprintf(ACK, "UP_KD:%.2f, ",balance_UP_KD);
	velocity_KP = (float)( 100*(u1rxbuf[8]-'0') + 10*(u1rxbuf[9]-'0') + (u1rxbuf[10]-'0') );
	sprintf(ACK, "V_KP:%.0f, ",velocity_KP);
	velocity_KI = (float)( (u1rxbuf[12]-'0') + 0.1*(u1rxbuf[13]-'0') + 0.01*(u1rxbuf[14]-'0') );
	sprintf(ACK, "V_KI:%.2f, ",velocity_KI);
	Mechanical_angle = (float)( (u1rxbuf[17]-'0') + 0.1*(u1rxbuf[18]-'0') + 0.01*(u1rxbuf[19]-'0') );
	if(u1rxbuf[16]-'0' == 1) {Mechanical_angle *= -1;}
	sprintf(ACK, "Mechanical:%.2f, ",Mechanical_angle);
	velocity_KD = (float)( 10*(u1rxbuf[21]-'0') + (u1rxbuf[22]-'0') + 0.1*(u1rxbuf[23]-'0') );
	sprintf(ACK, "V_KD:%.1f, ",velocity_KD);
	Turn_KP = (float)( (u1rxbuf[25]-'0') + 0.1*(u1rxbuf[26]-'0') + 0.01*(u1rxbuf[27]-'0') );
	sprintf(ACK, "Turn_KP:%.2f, ",Turn_KP);
	Turn_KI = (float)( (u1rxbuf[29]-'0') + 0.1*(u1rxbuf[30]-'0') + 0.01*(u1rxbuf[31]-'0') );
	sprintf(ACK, "Turn_KI:%.2f\r\n",Turn_KI);
	printf("%s\r\n",ACK);
	//ACK_size = sizeof(ACK);
	//for(i=0;i<ACK_size;i++) ACK_u8[i] = (u8)ACK[i];
	//DMA_USART2_Tx_Data(ACK_u8,ACK_size);
	return;
}

void data_receive3(void)
{
	//u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		
		//len=USART3_RX_STA&0x3f;//得到此次接收到的数据长度
		//printf("%d, %d\r\n", start_time, end_time);
		/*
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
		*/
		target_speed = (float)( 10*(USART3_RX_BUF[1]-'0') + (USART3_RX_BUF[2]-'0') + 0.1*(USART3_RX_BUF[3]-'0') );
		if(USART3_RX_BUF[0]-'0' == 1) {target_speed *= -1;}
		//printf("%.1f\r\n",target_speed);
		USART3_RX_STA=0;
	}
	return;
}
	



