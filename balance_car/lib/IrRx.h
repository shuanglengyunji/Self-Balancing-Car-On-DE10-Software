/*
 * IrRx.h
 *
 *  Created on: 2016/3/23
 *      Author: User
 */

#ifndef IRRX_H_
#define IRRX_H_

#include "Queue.h"
class CIrRx : public CQueue{
public:
	uint32_t m_IRM_BASE;

	typedef enum{
		IR_POWER = 		0xed126b86,
		IR_CH_UP =		0xe51a6b86,
		IR_CH_DOWN = 	0xe11e6b86,
		IR_VOL_UP = 	0xe41b6b86,
		IR_VOL_DOWN = 	0xe01f6b86,
		IR_MUTE = 		0xf30c6b86,
		IR_ADJ_LEFT = 	0xeb146b86,
		IR_ADJ_RIGHT = 	0xe7186b86,
		IR_PLAY_PAUSE = 0xe9166b86,
		IR_NUM_0 = 		0xff006b86,
		IR_NUM_1 = 		0xfe016b86,
		IR_NUM_2 = 		0xfd026b86,
		IR_NUM_3 = 		0xfc036b86,
		IR_NUM_4 = 		0xfb046b86,
		IR_NUM_5 = 		0xfa056b86,
		IR_NUM_6 = 		0xf9066b86,
		IR_NUM_7 = 		0xf8076b86,
		IR_NUM_8 = 		0xf7086b86,
		IR_NUM_9 = 		0xf6096b86,
		IR_NUM_A = 		0xf00f6b86,
		IR_NUM_B = 		0xec136b86,
		IR_NUM_C = 		0xef106b86,
		IR_RETURN = 	0xe8176b86,
		IR_MENU = 		0xee116b86
	};

public:
	CIrRx(const uint32_t IRM_BASE);
	virtual ~CIrRx();

	bool Enable(void);
	void Disable(void);
	void Test(void);
	void irq_handler(void);

	//CQueue m_Queue;
};

#endif /* IRRX_H_ */
