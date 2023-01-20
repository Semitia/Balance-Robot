#include "control.h"
//const u8 WARN_MSG=1,SPD_MSG=2,POS_MSG=3,PARA_MSG=4,DES_MSG=5,ACK_MSG=6;
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
u8 motion_mode=0;
bool ACK=0;
float Mechanical_angle=MECHI; 
float target_x=0, target_y=0;
//float target_yaw=0;
float target_speed=0;	//期望速度。用于控制小车前进后退及其速度。
float target_omiga=0; //期望角速度
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
		switch(Mode)
		{
			case 0:
			{
				Turn_Pwm = anya_yaw();
				break;
			}
			case 1:
			{
				Turn_Pwm = anya_omiga();
				break;
			}
			case 2:
			{
				Turn_Pwm = anya_yaw();
				break;
			}
		}
		//Turn_Pwm = anya_yaw();
		Moto1=Balance_Pwm+Turn_Pwm;  	            //===计算左轮电机最终PWM
		Moto2=Balance_Pwm-Turn_Pwm;                 //===计算右轮电机最终PWM
	    Xianfu_Pwm();  																					 //===PWM限幅
		Turn_Off(now.pitch,Voltage);																 //===检查角度以及电压是否正常
		Set_Pwm(Moto1,Moto2);    //===赋值给PWM寄存器  
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
	
	return pwm_out;
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

int anya_omiga(void)
{
	int pwm_out;
	float error = target_omiga - now.w;
	pwm_out = error*Turn_KP/5;
	return -pwm_out;
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

u8 warn=0;
u8 info_req=0;
const u8 for_bac=1, turn=2;
void data_receive(void)
{
	u8 *buf;
	u8 msg_type;
//#if USART1_DMA
	if(!USART1_RX_FLAG)
	{info_req=0;return;}
	USART1_RX_FLAG = 0;
	buf = usart1_rxbuf;
//#else
//	if(!(USART_RX_STA&0x80))
//	{return;}
//	rx_buf = USART_RX_BUF;
//#endif
	
	msg_type = buf[0]-'0';
	switch(msg_type)
	{
		case WARN_MSG:
		{
			Mode = 3;
			warn = tr_s(buf,1,3,2);//100*(rx_buf[1]-'0') + 10*(rx_buf[2]-'0') + (rx_buf[3]-'0');
			//info_req = WARN_MSG;
			break;
		}
		
		case SPD_MSG:
		{
			u8 mov_mod = tr(buf[1]);
			Mode = 1;
			switch(mov_mod)
			{
				case for_bac:
				{
					target_speed = tr_s(buf,2,3,1);//(float)( 10*(USART3_RX_BUF[1]-'0') + (USART3_RX_BUF[2]-'0') + 0.1*(USART3_RX_BUF[3]-'0') );
					target_omiga = 0;
					break;
				}
				case turn:
				{
					target_speed = 0;
					target_omiga = tr_s(buf,2,3,1);
					break;
				}
			}
			//printf("speed:%.2f, ",target_speed);
			//info_req = SPD_MSG;
			break;
		}
		
		case POS_MSG://要前往的坐标
		{
			Mode = 2;
			target_x = tr_s(buf,1,4,2);//100*(rx_buf[1]-'0') + 10*(rx_buf[2]-'0') + (rx_buf[3]-'0') + 0.1*(rx_buf[4]-'0');
			target_y = tr_s(buf,5,4,2);//100*(rx_buf[5]-'0') + 10*(rx_buf[6]-'0') + (rx_buf[7]-'0') + 0.1*(rx_buf[8]-'0');
			Target_Yaw = tr_s(buf,9,4,2);
			//info_req = POS_MSG;
			break;
		}
		
		case PARA_MSG:
		{
			balance_UP_KP = tr_s(buf,1,3,2);//(float) ( 100*(rx_buf[0]-'0') + 10*(rx_buf[1]-'0') + (rx_buf[2]-'0') );
			//printf("UP_KP:%.0f, ",balance_UP_KP);
			balance_UP_KD = tr_s(buf,5,3,0);//(float)( (rx_buf[4]-'0') + 0.1*(rx_buf[5]-'0') + 0.01*(rx_buf[6]-'0') );
			//printf("UP_KD:%.2f, ",balance_UP_KD);
			velocity_KP = tr_s(buf,9,3,2);//(float)( 100*(rx_buf[8]-'0') + 10*(rx_buf[9]-'0') + (rx_buf[10]-'0') );
			//printf("V_KP:%.0f, ",velocity_KP);
			velocity_KI = tr_s(buf,13,3,0);//(float)( (rx_buf[12]-'0') + 0.1*(rx_buf[13]-'0') + 0.01*(rx_buf[14]-'0') );
			//printf("V_KI:%.2f, ",velocity_KI);
			Mechanical_angle = tr_s(buf,18,3,0);//(float)( (rx_buf[17]-'0') + 0.1*(rx_buf[18]-'0') + 0.01*(rx_buf[19]-'0') );
			if(buf[17]-'0' == 1) {Mechanical_angle *= -1;}
			//printf("Mechanical:%.2f, ",Mechanical_angle);
			velocity_KD = tr_s(buf,22,3,1);//(float)( 10*(rx_buf[21]-'0') + (rx_buf[22]-'0') + 0.1*(rx_buf[23]-'0') );
			//printf("V_KD:%.1f, ",velocity_KD);
			Turn_KP = tr_s(buf,26,3,0);//(float)( (rx_buf[25]-'0') + 0.1*(rx_buf[26]-'0') + 0.01*(rx_buf[27]-'0') );
			//printf("Turn_KP:%.2f, ",Turn_KP);
			Turn_KI = tr_s(buf,30,3,0);//(float)( (rx_buf[29]-'0') + 0.1*(rx_buf[30]-'0') + 0.01*(rx_buf[31]-'0') );
			//printf("Turn_KI:%.2f\r\n",Turn_KI);
			target_speed = tr_s(buf,35,3,2);//(float)( 100*(rx_buf[34]-'0') + 10*(rx_buf[35]-'0') + (rx_buf[36]-'0') );
			if(buf[34]-'0' == 1) {target_speed *= -1;}
			//printf("speed:%.0f, ",target_speed);
			Position_KP = tr_s(buf,39,3,0);//(float)( 1*(rx_buf[38]-'0') + 0.1*(rx_buf[39]-'0') + 0.01*(rx_buf[40]-'0') );
			//printf("P_KP:%.2f\r\n",Position_KP);
			break;
		}
		case ACK_MSG:
		{
			ACK=1;
			break;
		}
		
		
	}
	
//#if !USART1_DMA
//USART_RX_STA=0;
//#endif
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

bool des_flag=0;
void data_receive3(void)
{
	//u8 len,t;
	if(USART3_RX_STA&0x80)
	{					   
		//len=USART3_RX_STA&0x3f;//得到此次接收到的数据长度
		int BTcmd= (int)tr_s(USART3_RX_BUF,0,3,2);
		switch(BTcmd)
		{
			case 666://destination
			{
				target_x = tr_s(USART3_RX_BUF,3,4,2);
				target_y = tr_s(USART3_RX_BUF,7,4,2);
				des_flag=1;
				break;
			}
			case 667://param
			{
				balance_UP_KP = tr_s(USART3_RX_BUF,0,3,2);
				balance_UP_KD = tr_s(USART3_RX_BUF,4,3,0);
				velocity_KP = tr_s(USART3_RX_BUF,8,3,2);
				velocity_KI = tr_s(USART3_RX_BUF,12,3,0);
				Mechanical_angle = tr_s(USART3_RX_BUF,17,3,0);
				if(USART3_RX_BUF[16]-'0' == 1) {Mechanical_angle *= -1;}
				velocity_KD = tr_s(USART3_RX_BUF,21,3,1);
				Turn_KP = tr_s(USART3_RX_BUF,25,3,0);
				Turn_KI = tr_s(USART3_RX_BUF,29,3,0);
				break;
			}
			case 668://speed cmd
			{
				target_speed = tr_s(USART3_RX_BUF,3,4,1);
				if(USART3_RX_BUF[0]-'0' == 1) {target_speed *= -1;}
				target_omiga = tr_s(USART3_RX_BUF,8,4,1);
				if(USART3_RX_BUF[7]-'0' == 1) {target_omiga *= -1;}
				Mode=1;
				break;
			}
		}
		USART3_RX_STA=0;
	}
	return;
}
	
void sendmsg(void)
{
	u8 i;
	char buf[20];
	u8 len;
	if(!des_flag) 
	{
		sprintf(buf,"Waitting DES_CMD\r\n");
		len=18;
	}
	else if(!ACK)//DES_MSG & POS_MSG
	{
		int s1=f_to_u(target_x,1), s2=f_to_u(target_y,1), s3=f_to_u(now.x,1), s4=f_to_u(now.y,1), s5=f_to_u(now.theta,3);
		//sprintf(buf,"%u%d%d\r\n",DES_MSG,s1,s2);
		buf[0]=DES_MSG+48;
		swrite(buf,s1,1);
		swrite(buf,s2,6);
		swrite(buf,s3,11);
		swrite(buf,s4,16);
		swrite(buf,s5,21);
		len=27;
	}
	else//POS_MSG
	{
		int s1=f_to_u(now.x,1), s2=f_to_u(now.y,1), s3=f_to_u(now.theta,3);
		buf[0]=POS_MSG+48;
		swrite(buf,s1,1);
		swrite(buf,s2,6);
		swrite(buf,s3,11);
		//sprintf(buf,"%u%d%d%d\r\n",POS_MSG,s1,s2,s3);
		len=17;
	}
	for(i=0;i<len;i++)
	{
		DMA_USART1_Tx_Data(&buf[i],1);
	}
	//DMA_USART1_Tx_Data(buf,len);
	//printf_s(buf,0);
	
	switch(info_req)
	{
		case 0:
		{
			break;
		}
		case WARN_MSG:
		{
			char str[4];
			sprintf(str,"%u",warn);
			DMA_USART1_Tx_Data("INFO:WARN= ",12);
			DMA_USART1_Tx_Data(str,4);
			break;
		}
		case SPD_MSG:
		{
			if(target_speed>0) {DMA_USART1_Tx_Data("Forward",8);}
			else if(target_speed<0) {DMA_USART1_Tx_Data("Back",6);}
			else if(target_omiga<0) {DMA_USART1_Tx_Data("TurnRight",10);}
			else {DMA_USART1_Tx_Data("TurnLeft",9);}
			break;
		}
		case POS_MSG:
		{
			break;
		}
		case PARA_MSG:
		{
			break;
		}
		case DES_MSG:
		{
			break;
		}
	}
	
	return;
}

void state_info()
{
	u8 i;
	char buf[20];
	u8 len;

	int s1=f_to_u(now.x,1), s2=f_to_u(now.y,1), s4=f_to_u(now.w,1), s5=f_to_u(now.theta,1);
	buf[0]=POS_MSG+48;
	swrite(buf,s1,1);
	swrite(buf,s2,6);
	swrite(buf,s4,11);
	swrite(buf,s5,16);
	//sprintf(buf,"%u%d%d%d\r\n",POS_MSG,s1,s2,s3);
	len=22;
	for(i=0;i<len;i++)
	{
		DMA_USART1_Tx_Data(&buf[i],1);
	}
	return;
}

