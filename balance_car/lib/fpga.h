#ifndef FPGA_H
#define FPGA_H

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#define     measure_cnt_enable_reg 0x01
#define     measure_cnt_write_reg 0x02
#define     measure_cnt_read_reg 0x00

#define REG_TOTAL_DUR	0
#define REG_HIGH_DUR	1
#define REG_CTS			2

#define RUN_BIT			0x01
#define FORWARD_BIT		0x02
#define FAST_DECAY_BIT	0x04

class FPGA
{
public:
    FPGA();
    ~FPGA();

    bool LedSet(int mask);
    bool KeyRead(uint32_t *mask);
    bool InfraRedRead(uint32_t *mask);
	bool SwitchRead(uint32_t *mask);
    bool Left_Encoder_Read(uint16_t *left_encoder);
	bool Left_Encoder_Set(uint16_t left_encoder);
	bool Left_Encoder_Enable(bool enable);
	bool Right_Encoder_Read(uint16_t *right_encoder);
	bool Right_Encoder_Set(uint16_t right_encoder);
	bool Right_Encoder_Enable(bool enable);
	bool Sonic_Distance_Read(uint32_t *Sonic_distance);
	bool Left_Motor_Set_PWM(uint32_t left_pwm);
	bool Left_Motor_Set_Start(bool enable);
	bool Left_Motor_Set_Direction(bool bFordward);
	bool Left_Motor_Set_Total_Dur(uint32_t  Total_Dur);
	bool Right_Motor_Set_PWM(uint32_t right_pwm);
	bool Right_Motor_Set_Start(bool enable);
	bool Right_Motor_Set_Direction(bool bFordward);
	bool Right_Motor_Set_Total_Dur(uint32_t Total_Dur);
	bool Uart_Bluetooth_Read(uint32_t *bt_data);
	bool Uart_Bluetooth_Write(char *string, int len);
	bool ADC_Ltc2308_Read(uint16_t *vol_data);
	bool get_mpu_addr_base(uint32_t *addr);
	bool get_bt_addr_base(uint32_t *addr);
	bool get_IrRx_addr_base(uint32_t *addr);
	bool release_mem();
	bool Motor_Fast_Stop();

protected:
    bool m_bInitSuccess;
    int  m_file_mem;
    void *virtual_base;

    uint8_t *m_led_base;
    uint8_t *m_key_base;
    uint8_t *m_red_base;
    uint8_t *m_sw_base;
    uint8_t *m_motor_left_base;
	uint8_t *m_motor_right_base;
    uint8_t *m_encoder_left_base;
	uint8_t *m_encoder_right_base;
	uint8_t *m_sonic_distance_base;
	uint8_t *m_uart_bt_base;
	uint8_t *m_ir_rx_base;
	uint8_t *m_adc_ltc2308_base;
	uint8_t *m_mpu_i2c_base;
	
    bool Init();

};

#endif // FPGA_H
