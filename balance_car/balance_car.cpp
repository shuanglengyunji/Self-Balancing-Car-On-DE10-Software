// ============================================================================
// Copyright (c) 2017 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development
//   Kits made by Terasic.  Other use of this code, including the selling
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use
//   or functionality of this code.
//
// ============================================================================
//
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// ============================================================================
//Date:  Wed Sep 20 14:19:17 2017
// ============================================================================
// ============================================================================
//
// Major Functions:     DE10-Nano Balance Car demo(HPS version software) 
//                      Main routine entrance
//
// ============================================================================

// Revision History :
// ============================================================================
//   Ver  :| Author              :| Mod. Date   :| Changes Made:
//   V1.0 :| Will                :| 2017/09/26  :| Initial Revision
// ============================================================================

//===================================include source files========================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include "car.h"
#include "fpga.h"
#include "kalman.h"
#include "hps.h"
#include "IrRx.h"
#include "MPU.h"

//=================================define bluetooth command=======================================

typedef enum{
	CMD_FORWARD=1,
	CMD_BACKWARD,
	CMD_LEFT,
	CMD_RIGHT,
	CMD_STOP,
	CMD_AKBT,
	CMD_ATDM,
	CMD_ATUTON,
	CMD_ATUTOFF,
}COMMAND_ID;

typedef struct{
	char szCommand[10];
	int CommandId;
	bool bParameter;
}COMMAND_INFO;

COMMAND_INFO gCommandList[] = {
		{"ATFW", CMD_FORWARD, false},
		{"ATBW", CMD_BACKWARD, false},
		{"ATTL", CMD_LEFT, false},
		{"ATTR", CMD_RIGHT, false},
		{"ATST", CMD_STOP, false},
		{"ATAB", CMD_AKBT, false},
		{"ATDM", CMD_ATDM, false},
		{"ATUTON", CMD_ATUTON, false},
		{"ATUTOFF", CMD_ATUTOFF, false},
};

//==========================define global variable================================================
CAR *pcar;		//point to CAR instance
MPU *pmpu;		//point to MPU instance
Kalman kalman;	//define a Kalman global instance
CIrRx *pIR;		//point to CIrRx instance

static bool stop_flag;
int IR_flag = 1; 		// use ir or not
float Angle_Balance=0,Gyro_Balance=0,Gyro_Turn=0;
float PI=3.141592;
float x_angle=0;
float balance_pwm=0, velocity_pwm=0, turn_pwm=0;
int	Movement = 0;			// 速度期望
float Position = 0;		// 水平方向位置
struct itimerval itv;	// for timer data setting
unsigned char led=0, led0=0, led1=0, led2=0, led3=0;  // for led status
													  // led3=led[3:0] car status
													  // led2=led[4] for volatile
													  // led1=led[6:5] bluetooth and demo run
													  // led0=led[7] car not balance(0) or keep balance(1)
float distance=0;	// for ultrasonic data
float vol=0;		// for power value
int mode=0;			// for car running mode
bool demo=false,env=true;

int cnt_NG;		//for car fall down
bool pick_up=false;	//for picking up car

//============================define and implement functions=====================================



/**************************************************************************
 Function     : Print infra red Input
 parameter    : Infra red data
 return value :
 **************************************************************************/

void Print_Infra_Red(unsigned char red)
{
	int data[8];

	data[0] = red & 1   ? 1 : 0;
	data[1] = red & 2   ? 1 : 0;
	data[2] = red & 4   ? 1 : 0;
	data[3] = red & 8   ? 1 : 0;
	data[4] = red & 16  ? 1 : 0;
	data[5] = red & 32  ? 1 : 0;
	data[6] = red & 64  ? 1 : 0;
	data[7] = red & 128 ? 1 : 0;

	for(int i=0;i<8;i++)
	{
		printf("%d ",data[i]);
	}

	printf(" -- %f -- ",Position);
	// printf("balance = %f velocity = %f turn = %f\n", balance_pwm,velocity_pwm,turn_pwm);
	printf("distance = %f \n", distance);
}

/**************************************************************************
 Function     : Print infra red Input
 parameter    : Infra red data
 return value :
 **************************************************************************/

void Process_Infra_Red(unsigned char red)
{
	int data[8];
	float sum = 0;
	int count = 0;

	data[0] = red & 1   ? 1 : 0;
	data[1] = red & 2   ? 1 : 0;
	data[2] = red & 4   ? 1 : 0;
	data[3] = red & 8   ? 1 : 0;
	data[4] = red & 16  ? 1 : 0;
	data[5] = red & 32  ? 1 : 0;
	data[6] = red & 64  ? 1 : 0;
	data[7] = red & 128 ? 1 : 0;

	for(int i=0;i<8;i++)
	{
		if (data[i])
		{
			sum = sum + i - 3.5;
			count ++;
		}
	}

	if (count)
	{
		Position = sum / count;
	}
	else
	{
		Position = 0;
	}

}

/**************************************************************************
 Function     : Bluetooth Command Parsing	解析蓝牙数据
 parameter    : Command Command ID
 return value : Command Parsing data
 **************************************************************************/
bool CommandParsing(char *pCommand, int *pCommandID, int *pParam)
{
	bool bFind = false;
	int nNum, i, j, x=0;
	bool find_equal = false;
	char Data[10]={0};
	nNum = sizeof(gCommandList)/sizeof(gCommandList[0]);

	for(i=0;i<nNum && !bFind;i++){
		if (strncmp(pCommand, gCommandList[i].szCommand, strlen(gCommandList[i].szCommand)) == 0){
			*pCommandID = gCommandList[i].CommandId;
			if (gCommandList[i].bParameter){
				for(j=0;pCommand[j]!=0x0d;j++){
					if(find_equal==true){
						Data[x] = pCommand[j];
						x++;
					}
					else if(pCommand[j]=='=')
						find_equal=true;
				}
				*pParam=atoi(Data);
			}
			bFind = true;
		}
	}
	return bFind;
}

/**************************************************************************
Function     : Get Angle value (Kalman filter)
parameter    :
return value :
**************************************************************************/
void Get_Angle(void)
{
	int16_t ax, ay, az, gx, gy, gz;
	pmpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	Gyro_Balance=-gy;
	x_angle=atan2(ax,az)*180/PI;
	gy=gy/16.4;
	Angle_Balance=kalman.getAngle(x_angle,-gy);
	Gyro_Turn=gz;
}

/**************************************************************************
Function     : LQR control of the system
parameter    : The Angle value、The angular speed value、 The encoder values of the wheel
return value : LQR output Control PWM
**************************************************************************/

float LQR(float Angle,float Gyro)
{
	static float Phi=0;
	float Phi_Dot; //record encoder data
	float Theta;
	float Theta_Dot;
	float out;

	// Phi_Dot
	Phi_Dot = (pcar->Speed_right - pcar->Speed_left) - 0;		// 计算速度之和（因为两个码盘是相对着装的，所以车轮同向运动时速度反馈是相反的数值）
	//printf("encoder least = %f\n", Encoder_Least);

	// Phi
	Phi += Phi_Dot;
	if(Phi>8000)  	Phi=8000;
	if(Phi<-8000)	Phi=-8000;

	// Theta
	Theta = Angle;
	Theta_Dot = Gyro;

	out = (Phi * -1 + Theta * -57 + Phi_Dot * 0.13 + Theta_Dot * -7.75);
	return out;
}


/**************************************************************************
Function     : Upright Closed-loop Control (PD)
parameter    : The Angle value、The angular speed value
return value : Upright Closed-loop Control PWM
**************************************************************************/
float balance(float Angle,float Gyro)
{
	 float Bias,kp=11.0,kd=0.020;
	 float balance;
	 Bias=Angle-1;				// 偏差
	 balance=kp*Bias+Gyro*kd;
	 return balance;
}

/**************************************************************************
Function     : Speed Closed-loop Control (PI)
parameter    : The encoder values of the wheel
return value : Speed Closed-loop Control PWM
**************************************************************************/
float speed(void)
{
	static float Encoder_Integral=0,Encoder=0; //record encoder data

	float Speed=0, Encoder_Least=0;
	float kp=2.2,ki=0.05;

	// 判断期望速度数值
	if( (pcar->driver_direction&CAR_DIRECTION_FORWARD) && distance >= 15)
	{
		Movement = -20;
	}
	else if(pcar->driver_direction&CAR_DIRECTION_BACKWARD)
	{
		Movement = 20;
	}
	else
	{
		Movement = 0;
	}

	// 读取速度
	Encoder_Least = (pcar->Speed_right - pcar->Speed_left) - 0;		// 计算速度之和（因为两个码盘是相对着装的，所以车轮同向运动时速度反馈是相反的数值）
	//printf("encoder least = %f\n", Encoder_Least);

	// 低通滤波器
	Encoder *= 0.8;
	Encoder += Encoder_Least*0.2;
	//printf("Encoder = %f\n", Encoder);

	// 积分
	Encoder_Integral += Encoder;
	Encoder_Integral = Encoder_Integral - Movement;
	//printf("Encoder_integral = %f\n", Encoder_Integral);

	// 积分限幅
	if(Encoder_Integral>8000)  	Encoder_Integral=8000;
	if(Encoder_Integral<-8000)	Encoder_Integral=-8000;

	// 停机状态处理（低通滤波器清零，积分清零）
	if(stop_flag)
	{
		Encoder=0;
		Encoder_Integral=0;
	}

	Speed = Encoder*kp + Encoder_Integral*ki;
	//printf("Speed = %f\n", Speed);

	return Speed;
}

/**************************************************************************
Function     : Turn Closed-loop Control (PI)
parameter    : The Z axis gyroscope value
return value : Turn Closed-loop Control PWM
**************************************************************************/
int turn(float Gyro)
{
	static float Position_last = 0;

	float Position_derivative_tmp;
	static float Position_derivative;
	static float Position_derivative_out;
	static float Position_derivative_last;

	// 更新 Position_derivative
	Position_derivative_tmp = Position - Position_last;		// 微分
	Position_last = Position;
	if (Position_derivative_tmp != 0)
	{
		Position_derivative_last = Position_derivative;	// 更新 Position_derivative_last
		Position_derivative = Position_derivative_tmp;	// 更新 Position_derivative

	}

	// 过低通滤波器
	Position_derivative_out = 0.95 * Position_derivative_last + 0.05 * Position_derivative;

	float kp_position,kd_position,ki_position;
	float out;
	static float Position_integration = 0;

	if( (pcar->driver_direction&CAR_DIRECTION_FORWARD) && distance >= 15)
	{
		// kp
		if (Position >= 2 || Position <= -2)
		{
			kp_position = 20;
		}
		else
		{
			// kp_position = 12;
			kp_position = 20;
		}

		// kd
		kd_position = 40;

		// ki
		ki_position = 10;

		// PID control
		Position_integration += Position * 0.001;
		if (Position_integration >= 8)
			Position_integration = 8;
		if (Position_integration <= -8)
			Position_integration = -8;

		out = kp_position * Position + \
				ki_position * Position_integration + \
				kd_position * Position_derivative;

	}
	else if (pcar->driver_direction&CAR_DIRECTION_BACKWARD)
	{
		// kp
		if (Position >= 2 || Position <= -2)
		{
			kp_position = 20;
		}
		else
		{
			// kp_position = 12;
			kp_position = 20;
		}

		// kd
		kd_position = 40;

		// ki
		ki_position = 10;

		// PID control
		Position_integration += Position * 0.001;
		if (Position_integration >= 8)
			Position_integration = 8;
		if (Position_integration <= -8)
			Position_integration = -8;

		out = - kp_position * Position - \
				ki_position * Position_integration - \
				kd_position * Position_derivative;
	}
	else
	{
		Position_integration = 0;
		out = 0;
	}

	float Bias;
	Bias = pcar->Speed_left+pcar->Speed_right;
	Bias += out;

	float turn_pwm;
	float kp,kd;

	kp = 0.3;
	kd = 0.015;
	turn_pwm = kp*Bias - kd*Gyro;

	return turn_pwm;
}

/**************************************************************************
 Function     : UnNormal State Check
 parameter    : Angle_Balance value and power voltage
 return value :
 **************************************************************************/
void UnNomal_Check()
{
	if(pick_up||Angle_Balance>70||Angle_Balance<-70){
 		pcar->Set_Led(0x00);
 		pcar->set_car_pause();
 		pcar->Motor_Fast_Stop();
 		stop_flag=true;
		pick_up=false;
 	}
}

/**************************************************************************
 Function     : Normal State Check and Restart Self-Balance
 parameter    :
 return value : 1:normal  0:UnNormal
 **************************************************************************/
bool Restart_Check()
{
	if(x_angle<10&&x_angle>-10)
		return true;
	else
		return false;
}
/**************************************************************************
 Function     : IrRx work thread
 parameter    : ptr a pointer for data
 return value :
 **************************************************************************/
static void *IrRx_thread(void *ptr)
{
	//printf("enter thread\n");
	while(IR_flag){
			pIR->irq_handler();
	}
}

/**************************************************************************
 Function     : Set timer for mpu6500, interrupt per 10ms
 parameter    :
 return value : 
 **************************************************************************/
void set_timer(void)
{  
    itv.it_interval.tv_sec  = 0;
    itv.it_interval.tv_usec = 10000;
    itv.it_value.tv_sec     = 0;
    itv.it_value.tv_usec    = 1000;
   
    setitimer(ITIMER_REAL,&itv,NULL);
}

/**************************************************************************
 Function     : Clear timer for mpu6500
 parameter    :
 return value : 
 **************************************************************************/
void clear_timer(void)
{
	itv.it_interval.tv_sec  = 0;
    itv.it_interval.tv_usec = 0;
    itv.it_value = itv.it_interval;

    setitimer(ITIMER_REAL,&itv,NULL);
}

/**************************************************************************
 Function     : IR Control
 parameter    : Infra red data
 return value :
 **************************************************************************/

void IR_control()
{
	//====IR remote control=====//

	int Command_IR=0;
	if (!pIR->IsEmpty())
	{
		Command_IR = pIR->Pop();
		switch(Command_IR)
		{
		case CIrRx::IR_NUM_5: //stop
			led3=0x00;
			pcar->set_car_pause();
			//printf("5\n");
			break;
		case CIrRx::IR_NUM_2: //forward
			led3=0x01;
			pcar->set_car_forward();
			//printf("2\n");
			break;
		case CIrRx::IR_NUM_8: // backward
			led3=0x02;
			pcar->set_car_backward();
			//printf("8\n");
			break;
		case CIrRx::IR_NUM_6: // turn right
			led3=0x08;
			pcar->set_car_turnright();
			//printf("6\n");
			break;
		case CIrRx::IR_NUM_4: // turn left
			led3=0x04;
			pcar->set_car_turnleft();
			//printf("4\n");
			break;
		case CIrRx::IR_NUM_A: //demo
			//printf("A\n");
			break;
		default:
			break;
		}
		pIR->Clear();
	}
}

/**************************************************************************
 Function     : Bluetooth Control
 parameter    : Infra red data
 return value :
 **************************************************************************/

void BT_control()
{
	//===Bluetooth control===//

	// define variable for bt_uart
	static char data[10]={0};
	static int i_bt=0;
	uint32_t bt_value, tmp=0;

	// get cmd from bluetooth
	pcar->get_bt_value(&bt_value);
	tmp = bt_value>>16;

	// judge the value is valid
	if (tmp!=0)
	{
		data[i_bt] = bt_value&0xff;	//get cmd
		i_bt++;
		if (i_bt==10) i_bt = 0; 	// error data
		if ((bt_value&0xff)==0x0a)	//get cmd finished
		{
			i_bt=0;
			// do work according to cmd
			int cmd, param;
			if (CommandParsing(data, &cmd, &param))
			{
				switch (cmd){
				case CMD_FORWARD:
					led3=0x01;
					pcar->set_car_forward();
					break;
				case CMD_BACKWARD:
					led3=0x02;
					pcar->set_car_backward();
					break;
				case CMD_LEFT:
					led3=0x04;
					pcar->set_car_turnleft();
					break;
				case CMD_RIGHT:
					led3=0x08;
					pcar->set_car_turnright();
					break;
				case CMD_STOP:
					led3=0x00;
					pcar->set_car_pause();
					break;
				case CMD_AKBT:	// upload power to blutooth app
					pcar->set_power_value();
					break;
				case CMD_ATUTON:    //Ultrosonic ON
					break;
				case CMD_ATUTOFF:   //Ultrosonic OFF
					break;
				case CMD_ATDM:	// run demo
					break;
				default:
					break;
				}/*switch (cmd)*/
			}/*if (CommandParsing(bt_data, &cmd, &param))*/
		}/*if ((bt_value&0xff)==0x0a)*/
	}/*if (cmd_flag)*/
}

/**************************************************************************
 Function     : The mpu interrupt handler for 10ms
 parameter    :
 return value : 
 **************************************************************************/
unsigned int global_counter = 0;
void alarm_handle(int sig)
{
	static int irq_count=0, NG_count=0;
	int cnt=0;
	const int nAdjust=15;
	float LQR_out;
	
	global_counter++;

	pcar->Motor_Get_Speed();
	Get_Angle();
	pcar->get_red_infra_red_sensor();
	
	Process_Infra_Red(pcar->infra_red);

	// set cnt value
	if(vol<10.0) 					cnt=100-nAdjust;
	else if (vol>=10.0&&vol<10.5)	cnt=110-nAdjust;
	else if (vol>=10.5&&vol<11.0)	cnt=115-nAdjust;
	else if (vol>=11.0&&vol<11.5)	cnt=120-nAdjust;
	else if (vol>=11.5&&vol<12.0)	cnt=130-nAdjust;
 	else if(vol>12.0) 				cnt=135-nAdjust;
 	
	// check the car is picked up or not 
	cnt_NG = abs(pcar->Speed_left)+abs(pcar->Speed_right);

	if(cnt_NG>cnt)
		NG_count++;
	else
		NG_count=0;

	if(NG_count==30)
	{
		pick_up=true;
		NG_count=0;
	}
	
	if(!stop_flag)
	{
		led0=0x01;
		UnNomal_Check();
	}
	
	if(stop_flag)
	{
		if(Restart_Check()) 
		{
			if (mode == 0)
				irq_count++;
		}
		else
		{
			led0=0x00;
			led1=0x00;
			led2=0x00;
			led3=0x00;
			irq_count = 0;
			pcar->Motor_Stop();
		}
		if(irq_count == 100)
		{
			led0=0x01;
			irq_count = 0;
			stop_flag = false;
			pcar->Motor_Start();
		}
		//printf("stop flag\n");
	}
	
	// 计算PID
	LQR_out = LQR(Angle_Balance,Gyro_Balance);
	balance_pwm = balance(Angle_Balance,Gyro_Balance);
	velocity_pwm = speed();
	turn_pwm = turn(Gyro_Turn);

	// 输出
	if(!stop_flag){
		//printf("balance = %f\nvelocity = %f\nturn = %f\n", balance_pwm,velocity_pwm,turn_pwm);
		pcar->Motor_Set_Speed(-balance_pwm-velocity_pwm+turn_pwm,
							  -balance_pwm-velocity_pwm-turn_pwm);
//		pcar->Motor_Set_Speed(-balance_pwm-velocity_pwm+turn_pwm+LQR_out,
//							  -balance_pwm-velocity_pwm-turn_pwm+LQR_out);
	}
}

/**************************************************************************
Function     : The main function
parameter    : 
return value :
**************************************************************************/
int main(int argc, char **argv)
{
	// hps instance
	HPS  my_hps;	
		
	// new a car instance
	pcar = new CAR;				
	
	// new a mpu instance with i2c address
	uint32_t mpu_i2c_addr;		
	pcar->get_mpu_addr_base(&mpu_i2c_addr);
	pmpu = new MPU((uint32_t)mpu_i2c_addr, 0x68);
	
	// new a IR instance 
	pthread_t id0;
	uint32_t IrRx_addr;
	pcar->get_IrRx_addr_base(&IrRx_addr);
	pIR = new CIrRx((uint32_t)IrRx_addr);
	
	// check parameters ok or not	
	if(argc > 1){				
		pcar->Motor_Stop();
		stop_flag = true;
		return 1;
	}	
	
	// blinking the leds to remind user
	int i=10;					
	while(i--)
	{
		pcar->Set_Led(0xff);
		usleep(50000);
		pcar->Set_Led(0x00);
		usleep(50000);
	}
	
	// initialize mpu
	pmpu->initialize();			
	
	//2, enable IR
	pIR->Enable();
	IR_flag = 1;
	
	// initialize car and set speed
	pcar->Motor_Stop();			
	pcar->Set_Total_Dur();
	pcar->Motor_Start();
	pcar->Motor_Set_Speed(0,0);
	stop_flag = false;
	
	// set interrupt for mpu
	signal(SIGALRM,alarm_handle);	
	set_timer();
	
	// create thread for IrRx
    int ret0;
	ret0 = pthread_create(&id0,NULL,IrRx_thread, (void *)NULL);
	if(ret0!=0){
		printf("Creat pthread-0 error!\n");
		IR_flag = 0;
	}

	// enter main loop
	while(1)
	{

		if(global_counter>=50)
		{
			global_counter = 0;

			// Display Binary
			Print_Infra_Red(pcar->infra_red);
			//printf("%f\n",out);
		}

		//pcar->get_key(&demo, &env);
		
		// get power and set led according to power value
	    pcar->get_power_value();
	    vol = pcar->power_value;
	    if(vol<10.5)
			led2=0x01;
	    else
		    led2=0x00;	
		
		// detecte distance
		pcar->Sonic_Get_Distance(&distance);
		
		// read the switch status to determine which mode to run
		pcar->get_mode();
		mode = pcar->mode;
		
		// mode
		if (mode == 0x00)
		{

		}
		else if (mode == 0x01)
		{

		}
		else if (mode == 0x02)
		{

		}
		else if (mode == 0x03)
		{
			pcar->Set_Led(0x00);
			pcar->set_car_pause();
 			pcar->Motor_Stop();
 			my_hps.LedSet(false);
 			stop_flag=true;
 			clear_timer();
 			IR_flag = 0;
 			pcar->release_mem();
 			
			break;	// 退出主循环，进而退出程序
		}

		BT_control();
		IR_control();

		my_hps.LedSet(true);
		led=((led0&0x01)<<7)|((led1&0x03)<<5)|((led2&0x01)<<4)|(led3&0x0f);
		pcar->Set_Led(led);
	} 
	free(pcar);
	free(pmpu);
	
	return 0;
}//end main

