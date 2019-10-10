/*
 * Queue.h
 *
 *  Created on: 2016/3/23
 *      Author: User
 */

#ifndef QUEUE_H_
#define QUEUE_H_
#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#define IORD(base, index)                             (*( ((uint32_t *)base)+index))
#define IOWR(base, index, data)                       (*(((uint32_t *)base)+index) = data)
class CQueue {
protected:
	uint32_t m_nNum;
	uint32_t m_nFront;
	uint32_t m_nRear;
	uint32_t *m_szQueue;

public:
	CQueue(const int nQueueSize = 8);
	virtual ~CQueue();

	bool IsEmpty(void);
	bool IsFull(void);
	bool Push(uint32_t data32);
	uint32_t Pop(void);
	void Clear(void); // clear queue
};

#endif /* QUEUE_H_ */
