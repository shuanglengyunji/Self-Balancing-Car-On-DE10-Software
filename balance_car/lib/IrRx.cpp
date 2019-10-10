/*
 * IrRx.cpp
 *
 *  Created on: 2016/3/23
 *      Author: User
 */

#include "IrRx.h"

void CIrRx::irq_handler(void)
{
    uint32_t ScanCode=0;
    static uint32_t temp=0xffffffff;
    if (temp == 0xffffffff)
    	temp = IORD(m_IRM_BASE, 0);
    ScanCode = IORD(m_IRM_BASE, 0);
    //printf("ScanCode : %x\n", ScanCode);
    if (ScanCode!=temp){
   	 	printf("ScanCode in : %x\n", ScanCode);
   	 	temp = ScanCode;
   	 	if (!IsFull())
    		Push(ScanCode);
	}
}


CIrRx::CIrRx(const uint32_t IRM_BASE):
CQueue(8),
m_IRM_BASE(IRM_BASE)
{
	// TODO Auto-generated constructor stub
}

CIrRx::~CIrRx() {
	// TODO Auto-generated destructor stub
}


bool CIrRx::Enable(void){
	CQueue::Clear();
	return true;
}


void CIrRx::Disable(void){
   	CQueue::Clear();
}

void CIrRx::Test(void){
	int nIndex = 1;
	if (!Enable()){
		printf("failed to enable IR\r\n");
	}else{
		printf("Enable ir\r\n");
		while(1){
			if (!IsEmpty()){
				printf("[%d]%04xh\r\n", nIndex++, Pop());
			}
		}
	}
}


