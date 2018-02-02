/*
 * dev_memory.c
 *
 *  Created on: 26 нояб. 2017 г.
 *      Author: eric

 == persistent memory functions pseudo device

 */


#include "dev_memory.h"
#include "ssn.h"
#include <libopencm3/stm32/flash.h>
#include "commands.h"

#ifdef PERSIST_EEPROM_I2C
#include "i2c_eeprom.h"
#endif

int32_t MemoryDevInitStruct(sDevice* pDev, int16_t nSize) {
	int32_t nRes = pdFAIL;
//	pDev->pGroup->iDevQty = nSize;
	// allocate memory for memory elements array:
	if (pDev) {
		pDev->pDevStruct = (void*) pvPortMalloc(sizeof(memory_data_t));
		((memory_data_t*) (pDev->pDevStruct))->pMemoryArray = pvPortMalloc(
				sizeof(uint32_t) * nSize);
		if (pDev->pDevStruct
				&& ((memory_data_t*) (pDev->pDevStruct))->pMemoryArray) {
			((memory_data_t*) (pDev->pDevStruct))->nArraySize = nSize;
			pDev->pGroup->iDevQty = 1; // only one memory device in group
			nRes = pdPASS;
		} else
			nRes = pdFAIL;

		if (nRes) {
			xprintfMsg(
					"\r\nOk. Memory device initialized: [%d], Elements = %d ",
					pDev->nId, nSize);
		} else {
			xprintfMsg("\r\nError! Memory device error  %d ", pDev->nId);
		}
	}
	return nRes;
}

int16_t getMemoryDevQty(sDevice* pDev) {
	if (pDev) {
		return ((memory_data_t*) (pDev->pDevStruct))->nArraySize;
	}
	return 0;
}

int32_t getMemoryDevValue(sDevice* pDev, uint8_t nValCode, int32_t* nDevValue)
{

	if (pDev->pDevStruct) {
		if (nValCode < ((memory_data_t*)(pDev->pDevStruct))->nArraySize) {
			*nDevValue = *(((memory_data_t*)(pDev->pDevStruct))->pMemoryArray+nValCode);
			return pdPASS;
		}
	}
	return pdFAIL;
}

int32_t setMemoryDevValue(sDevice* pDev, uint8_t nValCode, int32_t nDevValue)
{
	if (pDev->pDevStruct) {
		if (nValCode < ((memory_data_t*)(pDev->pDevStruct))->nArraySize) {
			*(((memory_data_t*)(pDev->pDevStruct))->pMemoryArray+nValCode) = nDevValue;
			 pDev->uiLastUpdate = rtc_get_counter_val();
			return pdPASS;
		}
	}
	return pdFAIL;
}

// serialize memory device and return pointer to created buffer and buffer size
// external procedure must free memory!
int32_t serializeMemDevs(sDevice* pDev, void **pBuffer, uint16_t *nBufSize)
{
	int32_t nRes = pdFAIL;
//	uint16_t nIndex;

	if (!pDev)
		goto serializeMemDevsEnd;

	*nBufSize = 2+2+sizeof(uint32_t)*((memory_data_t*)(pDev->pDevStruct))->nArraySize;
	*pBuffer = pvPortMalloc(*nBufSize);

//	int32_t nValue;

	if (*pBuffer) {
/* Full memDev data structure:

   NUMDEVS  DEV1  e1  Val_1    Val_2    Val_n    DEV2  e2   Val_1     Val_2     CRC    Addr
   -------|------|--|--------|--------|--------|------|--|---------|----------|------|-------|
      2b    2b    2b   4b        4b       4b      2b    2b    4b       4b       2b     2b

   This procedure fill only one device part: (DEV1+e1+Val_1...Val_x)
*/
//		puiBuf[nIndex] = mem_devs_counter; // number of memory devices
		*(uint16_t*)(*pBuffer+0)=pDev->nId;
		*(uint16_t*)(*pBuffer+2)=((memory_data_t*)(pDev->pDevStruct))->nArraySize;
		memcpy((uint16_t*)(*pBuffer+4), ((memory_data_t*)(pDev->pDevStruct))->pMemoryArray, sizeof(uint32_t)*((memory_data_t*)(pDev->pDevStruct))->nArraySize);
		nRes = pdPASS;

	}

	serializeMemDevsEnd:
	return nRes;
}

// store memory device into persistent storage:
int32_t storeMemDevs(void* pBuf, uint16_t nBufSize)
{
	uint16_t calcCRC;
	uint16_t nAddr;
	int32_t nRes = pdPASS;

// calculate EEPROM address, memory devices data save into top EEPROM:
// top EEPROM memory - nBufSize
	nAddr = EEPROM_MAX_SIZE - nBufSize;
	calcCRC = crc16((uint8_t*) pBuf, nBufSize-4);
	*(uint16_t*)(pBuf+nBufSize-4) = calcCRC;
	*(uint16_t*)(pBuf+nBufSize-2) = nAddr;

// TO DO: check with preferences overlapping..

	nRes = storePersistentData(pBuf, nBufSize, nAddr);

	return nRes;
}

// restore memory device from persistent storage into RAM:
int32_t restoreMemDevs(sDevice* pDev)
{
	int32_t nRes = pdPASS;
	uint16_t nBufSize = 0;
	uint8_t* puiBuf;
	uint16_t calcCRC, bufCRC;
	int32_t nValue;
//	uint16_t nMemDevID;
	uint8_t	 nMemDevElements;
	uint16_t nIndex = 2;
	uint16_t nAddr;
	uint8_t sBuf1[2]; // array for address value storing
	uint8_t nNumMemDevs = 0;

	nAddr = EEPROM_MAX_SIZE - 2; // last 2 bytes EEPROM - address of memoryDev block

	nRes = restorePersistentData((uint8_t*) &sBuf1, 2, nAddr);

	if (nRes) {

		nAddr = (sBuf1[1] << 8) + sBuf1[0]; // calculate address of data block
		nBufSize = EEPROM_MAX_SIZE - nAddr;

		puiBuf = pvPortMalloc(nBufSize);

			if (puiBuf) {
				nRes = restorePersistentData(puiBuf, nBufSize, nAddr);
				calcCRC = crc16((uint8_t*) puiBuf, nBufSize-4);
				bufCRC = *(uint16_t*)(puiBuf+nBufSize-4); // last 2 bytes = CRC

				if (calcCRC == bufCRC) {
					nNumMemDevs = *(uint8_t*)(puiBuf);

					for (uint8_t i = 0; i < nNumMemDevs; i++)
					{
//						nMemDevID = *(uint16_t*)(puiBuf+nIndex); // to do - ???
						nIndex += 2;
						nMemDevElements = *(uint8_t*)(puiBuf+nIndex);
						nIndex += 2;

						for (uint8_t j = 0; j < nMemDevElements; j++)
						{
							nValue = *(int32_t*)(puiBuf+nIndex);
							nIndex += 4;
//							setDevValueByID(nValue, j, nMemDevID, eElmInteger);
							setDevValue(nValue, j, pDev, eElmInteger);
						}
					}

				} else {
					nRes = pdFAIL;
				}
				vPortFree((void*)puiBuf);
			}
	} else {
		nRes = pdFAIL;
	}

//restoreMemDevsEnd:
	return nRes;
}


// Store buffer data into persistent memory:
int32_t storePersistentData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr)
{
	int32_t nRes = pdFAIL;

#ifdef PERSIST_EEPROM_I2C
	// to do!!!
// save into EEPROM
	sDevice* pDev = getDevByNo(0); // get virtual i2c device
	if (pDev) {
		nRes = eeprom_write(&pDev->pGroup->GrpDev, EEPROM_ADDRESS, nBeginAddr, (uint8_t*)pBuf, nBufSize);	// res == 1 -> good!
	}
#endif
#ifdef PERSIST_STM32FLASH
// save into STM32 flash
		nRes = storeFlashData(pBuf, nBufSize, nBeginAddr);
#endif
		return nRes;
}

// Restore buffer data from persistent memory:
int32_t restorePersistentData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr)
{
	int32_t nRes = pdFAIL;
	if (pBuf) {
		if ((nBufSize > (nBeginAddr + STM32FLASH_PREF_SIZE)) || (nBeginAddr >= STM32FLASH_PREF_SIZE)) {
			xprintfMsg("\r\nError: request data > EEPROM size!");
			nRes = pdFAIL;
		} else {

#ifdef PERSIST_EEPROM_I2C
			sDevice* pDev = getDevByNo(0); // get virtual i2c device
			if (pDev) {
				nRes = eeprom_read(&pDev->pGroup->GrpDev, EEPROM_ADDRESS, nBeginAddr, (uint8_t*)pBuf, nBufSize);
				xprintfMsg("\r\nData loaded from EEPROM: from %d, %d bytes",nBeginAddr, nBufSize);
			}
#endif
#ifdef PERSIST_STM32FLASH
			// simply copy into buffer data from flash memory where stored preferences
			uint32_t i = STM32FLASH_BEGIN_ADDR + nBeginAddr;
			memcpy(pBuf, (void*) (i), nBufSize);
			nRes = pdPASS;
			xprintfMsg("\r\nData loaded from FLASH: from %d, %d bytes",nBeginAddr, nBufSize);
#endif

		}
	}
	return nRes;
}

// Store buffer data into MC Flash memory:
int32_t storeFlashData(void* pBuf, uint16_t nBufSize, uint32_t nBeginAddr)
{
	uint32_t i;
	uint16_t j;
	uint32_t data;
	int32_t nRes = pdPASS;
	uint32_t nStartAddress = STM32FLASH_BEGIN_ADDR + nBeginAddr;

	if (nBufSize > (nBeginAddr + STM32FLASH_PREF_SIZE)) {
		xprintfMsg("\r\nError: data > EEPROM size");
		nRes = pdFAIL;
	}
	else {
		flash_unlock();
		// erase necessary flash pages
		for (i = nStartAddress; i < (nStartAddress + nBufSize); i+=STM32FLASH_PAGE_SIZE )
		{
			flash_erase_page(i);
		}
		// write data into flash

		i = nStartAddress;
		for (j = 0; j < nBufSize; j+=4)
		{
			data = *(uint32_t*)(pBuf+j);

			flash_program_word(i, data);
			i += 4;
		}

		nRes = pdPASS;
		flash_lock();
	}
	return nRes;
}
