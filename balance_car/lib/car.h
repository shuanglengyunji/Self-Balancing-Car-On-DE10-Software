#ifndef CAR_H
#define CAR_H

#include "fpga.h"
#include <sys/types.h>
#include "math.h"
#include <string.h>

#define CYCLE_WIDTH 		3500//5000
#define CYCLE_WIDTH_MAX 	3500//5000
#define CYCLE_WIDTH_MINI 	100 //100

#define CAR_DIRECTION_FORWARD   0x01
#define CAR_DIRECTION_BACKWARD  0x02
#define CAR_TURN_LEFT   	0x01
#define CAR_TURN_RIGHT  	0x02
class CAR{
	
	protected:
		FPGA Qsys_FPGA;
		int m_CycleWidthMini;
		int m_CycleWidthMaxi;
		bool car_start;
	public:
		CAR();
		int16_t   Speed_left, Speed_right;
		unsigned char driver_direction;
		unsigned char turn_direction;
		float power_value; 	// store power value
		uint8_t mode; 	// running mode
		uint32_t infra_red;
		
		void Set_Led(uint8_t mask);
		void get_key(bool *enable,bool *env);
		void get_red_infra_red_sensor();		//read switch status to confirm running mod
		void get_mode();		//read switch status to confirm running mod

		void Motor_Start();
		void Set_Total_Dur();
		void Motor_Stop();
		void Motor_Fast_Stop();
		void Motor_Set_Speed(float fSpeed_left ,float fSpeed_right /* -100~100*/);
		void Motor_Get_Speed();
		void Sonic_Get_Distance(float *distance);

		void get_power_value();			//get power value from ltc2308
		void get_bt_value(uint32_t *pdata);	//get bluetooth value from fpga
		void set_power_value(); 		//send power value to bluetooth app

		void get_mpu_addr_base(uint32_t *addr);
		void get_bt_addr_base(uint32_t *addr);
		void get_IrRx_addr_base(uint32_t *addr);
		void release_mem();
		
		void Set_DriveDirection(unsigned char m_direction);
		void Set_TurnDirection(unsigned char m_direction);

		void set_car_forward();
		void set_car_backward();
		void set_car_turnright();
		void set_car_turnleft();
		void set_car_pause();
};

#endif // CAR_H
