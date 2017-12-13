/*
 * dev_memory.h
 *
 *  Created on: 26 нояб. 2017 г.
 *      Author: eric

 == persistent memory functions pseudo device

 */

#ifndef INC_DEV_MEMORY_H_
#define INC_DEV_MEMORY_H_

#include "device.h"

typedef struct{
	int16_t	nArraySize;
	int32_t	*pMemoryArray;
} memory_data_t;

int32_t 	MemoryDevInitStruct(sDevice* pDev, int16_t nSize);
int16_t	getMemoryDevQty(sDevice* pDev);
int32_t 	getMemoryDevValue(sDevice* dev, uint8_t nValCode, int32_t* nDevValue);
int32_t 	setMemoryDevValue(sDevice* pDev, uint8_t nValCode, int32_t nDevValue);

int32_t 	restoreMemDevs(sDevice* pDev);
int32_t 	storeMemDevs(void* pBuf, uint16_t nBufSize);
int32_t 	serializeMemDevs(sDevice* pDev, void **pBuffer, uint16_t *nBufSize);

int32_t 	storeFlashData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr);
int32_t 	storePersistentData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr);
int32_t 	restorePersistentData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr);

#endif /* INC_DEV_MEMORY_H_ */
