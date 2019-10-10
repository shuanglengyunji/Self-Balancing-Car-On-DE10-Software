/*
 * Queue.cpp
 *
 *  Created on: 2016/3/23
 *      Author: User
 */

#include "Queue.h"

CQueue::CQueue(const int nQueueSize):
	m_nNum(nQueueSize)
{
	// TODO Auto-generated constructor stub
	m_szQueue = (uint32_t *)malloc(nQueueSize*sizeof(uint32_t));
	Clear();

}

CQueue::~CQueue() {
	// TODO Auto-generated destructor stub
	free(m_szQueue);
}


bool CQueue::IsEmpty(void){
    if (m_nFront == m_nRear)
        return true;
    return false;
}

bool CQueue::IsFull(void){
    if (((m_nFront+1)%m_nNum) == m_nRear)
        return true;
    return false;
}

bool CQueue::Push(uint32_t data32){
    if (IsFull())
        return false;
    m_szQueue[m_nFront] = data32;
    m_nFront = (m_nFront+1)%m_nNum;
    return true;
}

uint32_t CQueue::Pop(void){
    uint32_t data32;
    if (IsEmpty())
        return 0;
    data32 = m_szQueue[m_nRear];
    m_nRear = (m_nRear+1)%m_nNum;
    return data32;
}

void CQueue::Clear(void){
	m_nFront = 0;
    m_nRear = 0;
}

