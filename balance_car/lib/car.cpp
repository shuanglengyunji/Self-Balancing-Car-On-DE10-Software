
#include "car.h"
#include <stdio.h>
#include <string.h>

CAR::CAR( )
{
	m_CycleWidthMini=CYCLE_WIDTH_MINI;
	m_CycleWidthMaxi=CYCLE_WIDTH_MAX;
	car_start=false;
}

void CAR::Set_Led(uint8_t mask)
{
	Qsys_FPGA.LedSet(mask);	
}

void CAR::Set_Total_Dur(void )
{
	Qsys_FPGA.Left_Motor_Set_Total_Dur(CYCLE_WIDTH);	
	Qsys_FPGA.Right_Motor_Set_Total_Dur(CYCLE_WIDTH);	
}

void CAR::Motor_Start( )
{
	Qsys_FPGA.Left_Motor_Set_Start(true);	
	Qsys_FPGA.Right_Motor_Set_Start(true);	
	Qsys_FPGA.Left_Encoder_Enable(true);
	Qsys_FPGA.Right_Encoder_Enable(true);
	car_start=true;
}

void CAR::Motor_Stop( )
{
	Qsys_FPGA.Left_Motor_Set_Start(false);	
	Qsys_FPGA.Right_Motor_Set_Start(false);	
	Qsys_FPGA.Left_Encoder_Enable(false);
	Qsys_FPGA.Right_Encoder_Enable(false);
	car_start=false;
}

void CAR::Motor_Fast_Stop( )
{
	Qsys_FPGA.Motor_Fast_Stop();	
	car_start=false;
}

void CAR::Motor_Set_Speed(float fSpeed_left ,float fSpeed_right /* -100~100*/){
	int SpeedParam = 0;

	// bound speed
	if (fSpeed_left < -100.0)
		fSpeed_left = -100;
	else if (fSpeed_left > 100.0)
		fSpeed_left = 100.0;
	//
	if (fSpeed_left != 0.0){
		SpeedParam = m_CycleWidthMini + (int)(fabs(fSpeed_left) * (float)(m_CycleWidthMaxi-m_CycleWidthMini) / 100.0);
	}
	Qsys_FPGA.Left_Motor_Set_PWM(SpeedParam);
	Qsys_FPGA.Left_Motor_Set_Direction((fSpeed_left >= 0.0)?true:false);
	if (fSpeed_right < -100.0)
		fSpeed_right = -100;
	else if (fSpeed_right > 100.0)
		fSpeed_right = 100.0;
	//
	if (fSpeed_right != 0.0){
		SpeedParam = m_CycleWidthMini + (int)(fabs(fSpeed_right) * (float)(m_CycleWidthMaxi-m_CycleWidthMini) / 100.0);
	}
	Qsys_FPGA.Right_Motor_Set_PWM(SpeedParam);
	Qsys_FPGA.Right_Motor_Set_Direction((fSpeed_right >= 0.0)?true:false);
}

void CAR::Motor_Get_Speed( )
{
	 uint16_t  count;

	 // Left Wheel
	 Qsys_FPGA.Left_Encoder_Read(&count);
	 Qsys_FPGA.Left_Encoder_Set(0x8000);
	 Speed_left=count-0x8000;
	 //printf("Speed_left = %d\n", Speed_left);

	 // Right Wheel
	 Qsys_FPGA.Right_Encoder_Read(&count);
	 Qsys_FPGA.Right_Encoder_Set(0x8000);
	 Speed_right=count-0x8000;
	 //printf("Speed_right = %d\n", Speed_right);
}

void CAR::Sonic_Get_Distance(float *distance){
	uint32_t sonic_reg;
	Qsys_FPGA.Sonic_Distance_Read(&sonic_reg);
	if(sonic_reg==0x98){  //0x98 is the error data when read
			*distance=*distance;
			printf("error data\n");
	}
	else
		    *distance = (float)sonic_reg*34.0/100000.0;
}

void CAR::get_red_infra_red_sensor()
{
	uint32_t red;
	Qsys_FPGA.InfraRedRead(&red);
	infra_red = red;
}

void CAR::get_key(bool *enable,bool *env)
{
	uint32_t key_reg;
	Qsys_FPGA.KeyRead(&key_reg);
	if(!(key_reg & 0x01))
	{
		if(*enable)
			*enable=false;
		else
			*enable=true;
	}
	if(!(key_reg & 0x02))
	{
		if(*env)
			*env=false;
		else
			*env=true;
	}
}

void CAR::get_power_value()
{
	uint16_t data;
	if (!Qsys_FPGA.ADC_Ltc2308_Read(&data)){
		power_value = 0;
	}
	power_value = (float)data*18.0/4.7/1000.0;
}

void CAR::get_bt_value(uint32_t *pdata)
{
	Qsys_FPGA.Uart_Bluetooth_Read(pdata);
}

void CAR::set_power_value()
{
	char data[10]={0};
	sprintf(data,"%d\n",(uint32_t)(power_value*1000));
	//printf("vol = %s\n", data);
	int len = strlen(data);
	Qsys_FPGA.Uart_Bluetooth_Write(data, len);
}

void CAR::get_mode()
{
	uint32_t mask;
	if (!Qsys_FPGA.SwitchRead(&mask)){
		mode = 0;
	}
	mode = (uint8_t)(mask&0xf);
}

void CAR::get_mpu_addr_base(uint32_t *addr)
{
	uint32_t data;
	//printf("Qsys_fpga: get mpu_addr_base\n");
	Qsys_FPGA.get_mpu_addr_base(&data);
	//printf("data = %x\n", data);
	*addr = data;
}

void CAR::get_bt_addr_base(uint32_t *addr)
{
	uint32_t data;
	//printf("Qsys_fpga: get mpu_addr_base\n");
	Qsys_FPGA.get_bt_addr_base(&data);
	//printf("data = %x\n", data);
	*addr = data;
}

void CAR::get_IrRx_addr_base(uint32_t *addr)
{
	uint32_t data;
	//printf("Qsys_fpga: get mpu_addr_base\n");
	Qsys_FPGA.get_IrRx_addr_base(&data);
	//printf("data = %x\n", data);
	*addr = data;
}

void CAR::release_mem()
{
	if (Qsys_FPGA.release_mem())
		printf("release successfully!\n");
	
}

//-------------------------------------------

void CAR::Set_DriveDirection(unsigned char m_direction)
{
	driver_direction=m_direction;
	turn_direction=0x00;
}

void CAR::Set_TurnDirection(unsigned char m_direction)
{
	driver_direction=0x00;
	turn_direction=m_direction;
}

//-------------------------------------------

void CAR::set_car_forward()
{
	Set_DriveDirection(CAR_DIRECTION_FORWARD);
}

void CAR::set_car_backward()
{
	Set_DriveDirection(CAR_DIRECTION_BACKWARD);
}

void CAR::set_car_turnright()
{
	Set_TurnDirection(CAR_TURN_RIGHT);
}

void CAR::set_car_turnleft()
{
	Set_TurnDirection(CAR_TURN_LEFT);
}

void CAR::set_car_pause()
{
	Set_DriveDirection(0x00);
	Set_TurnDirection(0x00);
}
