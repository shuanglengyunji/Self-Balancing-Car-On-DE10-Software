#include "fpga.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>


#include <sys/mman.h>
#include <sys/types.h>

#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
//#include "hps_0.h"

// QSyS dependent address
#define FPGA_LED_PIO_BASE        0x10040 //16 byte
#define FPGA_KEY_PIO_BASE   	 0x100c0 //16 byte
#define FPGA_SW_PIO_BASE    	 0x10080 //16 byte
#define FPGA_RED_PIO_BASE    	 0x10090 //16 byte
#define FPGA_MOTOR_LEFT_BASE     0x70	 //16 byte
#define FPGA_MOTOR_RIGHT_BASE    0x60 	 //16 byte
#define	FPGA_ENCODER_LEFT_BASE 	 0x40  	 //32 byte
#define	FPGA_ENCODER_RIGHT_BASE  0x20	 //32 byte
#define FPGA_ADC_LTC2308_BASE    0x90 	 //8  byte
#define	FPGA_SONIC_DISTANCE_BASE 0x98 	 //8  byte
#define FPGA_UART_BT_BASE        0xa0 	 //8  byte
#define FPGA_IR_RX_BASE			 0xa8 	 //4  byte
#define FPGA_MPU_I2C_BASE		 0x00    //32 byte
	

#define IORD(base, index)                             (*( ((uint32_t *)base)+index))
#define IOWR(base, index, data)                       (*(((uint32_t *)base)+index) = data)
// ///////////////////////////////////////
// memory map

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )



// end
// ///////////////////////////////////////////////////

FPGA::FPGA()
{
    m_bInitSuccess = Init();
}

FPGA::~FPGA()
{
	printf("exit ~FPGA\n");
    //close(m_file_mem);
}

bool FPGA::Init()
{
    bool bSuccess = false;

    m_file_mem = open( "/dev/mem", ( O_RDWR | O_SYNC ) );
    if (m_file_mem != -1){
        virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, HW_REGS_BASE );
        if (virtual_base == MAP_FAILED){
        }else{
            m_led_base			  = (uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_LED_PIO_BASE)  	   & (unsigned long)(HW_REGS_MASK));
            m_key_base			  = (uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_KEY_PIO_BASE) 	   & (unsigned long)(HW_REGS_MASK));
            m_red_base			  = (uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_RED_PIO_BASE) 	   & (unsigned long)(HW_REGS_MASK));
            m_sw_base             = (uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_SW_PIO_BASE)         & (unsigned long)(HW_REGS_MASK));
			m_motor_left_base     =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_MOTOR_LEFT_BASE)     & (unsigned long)(HW_REGS_MASK));
			m_motor_right_base    =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_MOTOR_RIGHT_BASE)    & (unsigned long)(HW_REGS_MASK));
			m_encoder_left_base   =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_ENCODER_LEFT_BASE)   & (unsigned long)(HW_REGS_MASK));
			m_encoder_right_base  =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_ENCODER_RIGHT_BASE)  & (unsigned long)(HW_REGS_MASK));
			m_sonic_distance_base =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_SONIC_DISTANCE_BASE) & (unsigned long)(HW_REGS_MASK));
			m_uart_bt_base		  =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_UART_BT_BASE) 	   & (unsigned long)(HW_REGS_MASK));
			m_ir_rx_base		  =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_IR_RX_BASE) 	   	   & (unsigned long)(HW_REGS_MASK));
			m_adc_ltc2308_base	  =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_ADC_LTC2308_BASE)    & (unsigned long)(HW_REGS_MASK));
			m_mpu_i2c_base	  	  =	(uint8_t *)virtual_base + ((unsigned long)(ALT_LWFPGASLVS_OFST + FPGA_MPU_I2C_BASE)    	   & (unsigned long)(HW_REGS_MASK));

            bSuccess = true;
        }
        close(m_file_mem);
    }
	printf("memory map success\n");

    return bSuccess;
}


bool FPGA::LedSet(int mask){
    if (!m_bInitSuccess)
        return false;
	IOWR(m_led_base,0,mask);
    return true;
}

bool FPGA::InfraRedRead(uint32_t *mask){
    if (!m_bInitSuccess)
        return false;

    *mask = IORD(m_red_base,0);
    return true;

}

bool FPGA::KeyRead(uint32_t *mask){
    if (!m_bInitSuccess)
        return false;

    *mask = IORD(m_key_base,0);
    return true;

}

bool FPGA::SwitchRead(uint32_t *mask){
    if (!m_bInitSuccess)
        return false;
    *mask = IORD(m_sw_base,0);
    return true;

}


bool FPGA::Left_Encoder_Read(uint16_t *left_encoder){
	if (!m_bInitSuccess)
        return false;

    *left_encoder = IORD(m_encoder_left_base,0);
    return true;
}

bool FPGA::Left_Encoder_Set(uint16_t left_encoder){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_encoder_left_base,measure_cnt_write_reg,left_encoder);
	return true;
}

bool FPGA::Left_Encoder_Enable(bool enable){
	if (!m_bInitSuccess)
	return false;
	if(enable)
		IOWR(m_encoder_left_base,measure_cnt_enable_reg,0x01);  
	else IOWR(m_encoder_left_base,measure_cnt_enable_reg,0x00); 
	return true;
}

bool FPGA::Right_Encoder_Read(uint16_t *right_encoder){
	if (!m_bInitSuccess)
	return false;
	*right_encoder = IORD(m_encoder_right_base,0);
	return true;
}
bool FPGA::Right_Encoder_Set(uint16_t right_encoder){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_encoder_right_base,measure_cnt_write_reg,right_encoder);
	return true;
}
bool FPGA::Right_Encoder_Enable(bool enable){
	if (!m_bInitSuccess)
	return false;
	if(enable)
			IOWR(m_encoder_right_base,measure_cnt_enable_reg,0x01);  
	else IOWR(m_encoder_right_base,measure_cnt_enable_reg,0x00); 
	return true;
}
bool FPGA::Sonic_Distance_Read(uint32_t *Sonic_distance){
	if (!m_bInitSuccess)
		return false;
	*Sonic_distance = IORD(m_sonic_distance_base,0);
	//printf("Sonic_distance=%x\n",*Sonic_distance);
	return true;
}
bool FPGA::Left_Motor_Set_PWM(uint32_t left_pwm){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_motor_left_base,REG_HIGH_DUR,left_pwm);  
	return true;
}
bool FPGA::Left_Motor_Set_Start(bool enable){
	uint32_t Status,Control;
	if (!m_bInitSuccess)
        return false;
	Status = IORD(m_motor_left_base, REG_CTS);
	if(enable){
		Control = Status | RUN_BIT;
		IOWR(m_motor_left_base, REG_CTS, Control);
		IOWR(m_encoder_left_base, measure_cnt_enable_reg, 0x01);
	}else{
		Control = Status & (~RUN_BIT);
		IOWR(m_motor_left_base, REG_CTS, Control);
		IOWR(m_encoder_left_base, measure_cnt_enable_reg, 0x00);
	}
	return true;
}
bool FPGA::Left_Motor_Set_Direction(bool bFordward){
	if (!m_bInitSuccess)
        return false;
	uint32_t Status,Control;
	Status = IORD(m_motor_left_base, REG_CTS);
	if (bFordward)
		Control = Status | FORWARD_BIT;
	else
		Control = Status & ~FORWARD_BIT;
	IOWR(m_motor_left_base, REG_CTS, Control);
	return true;
}
bool FPGA::Left_Motor_Set_Total_Dur(uint32_t Total_Dur){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_motor_left_base, REG_TOTAL_DUR, Total_Dur);
	return true;
}
bool FPGA::Right_Motor_Set_PWM(uint32_t right_pwm){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_motor_right_base,REG_HIGH_DUR,right_pwm); 
	return true;
}
bool FPGA::Right_Motor_Set_Start(bool enable){
		uint32_t Status,Control;
	if (!m_bInitSuccess)
        return false;
	Status = IORD(m_motor_right_base, REG_CTS);
	if(enable){
		Control = Status | RUN_BIT;
		IOWR(m_motor_right_base, REG_CTS, Control);
		IOWR(m_encoder_right_base, measure_cnt_enable_reg, 0x01);
	}else{
		Control = Status & (~RUN_BIT);
		IOWR(m_motor_right_base, REG_CTS, Control);
		IOWR(m_encoder_right_base, measure_cnt_enable_reg, 0x00);
	}
	return true;
}
bool FPGA::Right_Motor_Set_Direction(bool bFordward){
	uint32_t Status,Control;
	if (!m_bInitSuccess)
        return false;
	Status = IORD(m_motor_right_base, REG_CTS);
	if (bFordward)
		Control = Status | FORWARD_BIT;
	else
		Control = Status & ~FORWARD_BIT;
	IOWR(m_motor_right_base, REG_CTS, Control);
	return true;
}
bool FPGA::Right_Motor_Set_Total_Dur(uint32_t Total_Dur){
	if (!m_bInitSuccess)
        return false;
	IOWR(m_motor_right_base, REG_TOTAL_DUR, Total_Dur);
	return true;
}

bool FPGA::Uart_Bluetooth_Read(uint32_t *bt_data)
{	
	uint32_t tmp;
	
	if (!m_bInitSuccess)
        return false;
	tmp = IORD(m_uart_bt_base,0);
	*bt_data = tmp;
	
	return true;
}

bool FPGA::Uart_Bluetooth_Write(char *string, int len)
{
	int i;
	uint32_t temp;

	if (!m_bInitSuccess)
        return false;
    //printf("uart write\n");
	for(i=0; i<len; i++)
	{
		temp=IORD(m_uart_bt_base, 1);
		temp=(temp>>16)&0xff;
		if(temp>0)
			IOWR(m_uart_bt_base, 0, string[i]);
		else 
			i--;
	}
	return true;
}

bool FPGA::ADC_Ltc2308_Read(uint16_t *vol_data)
{
	int  power_value;
	
	if (!m_bInitSuccess)
        return false;
	
	// set measure number for ADC convert
	IOWR(m_adc_ltc2308_base, 0x01, 1);
	
	// start measure
	IOWR(m_adc_ltc2308_base, 0x00, (0x01 << 1) | 0x00);
	IOWR(m_adc_ltc2308_base, 0x00, (0x01 << 1) | 0x01);
	IOWR(m_adc_ltc2308_base, 0x00, (0x01 << 1) | 0x00);
	
	usleep(1);
	while ((IORD(m_adc_ltc2308_base,0x00) & 0x01) == 0x00);
	power_value = IORD(m_adc_ltc2308_base, 0x01);

	*vol_data= power_value;
	return true;
}

bool FPGA::get_mpu_addr_base(uint32_t *addr)
{
	if (!m_bInitSuccess)
        return false;
    
	*addr = (uint32_t)m_mpu_i2c_base;
	return true;
	
}

bool FPGA::get_bt_addr_base(uint32_t *addr)
{
	if (!m_bInitSuccess)
        return false;
    
	*addr = (uint32_t)m_uart_bt_base;
	return true;	
}

bool FPGA::get_IrRx_addr_base(uint32_t *addr)
{
	if (!m_bInitSuccess)
        return false;
    
	*addr = (uint32_t)m_ir_rx_base;
	return true;
}

bool FPGA::release_mem()
{
	if (!m_bInitSuccess)
        return false;
    munmap(virtual_base, HW_REGS_SPAN);
    return true;
}

bool FPGA::Motor_Fast_Stop()
{
	
	if (!m_bInitSuccess)
        return false;

	IOWR(m_motor_left_base, REG_CTS, 0x00);
	IOWR(m_encoder_left_base, measure_cnt_enable_reg, 0x00);
	
	IOWR(m_motor_right_base, REG_CTS, 0x00);
	IOWR(m_encoder_right_base, measure_cnt_enable_reg, 0x00);
	
	return true;
}
