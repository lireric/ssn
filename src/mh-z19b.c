/*
 * mh-z19b.c
 *
 *  Created on: 25 февр. 2018 г.
 *      Author: eric
 */


/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2017 Ernold Vasiliev <ericv@mail.ru>
 *

    SSN project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSN project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SSN project.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * bmp180.c
 *
 * BMP180 atmosphere pressure and temperature sensor
 *
 *  Created on: 14 dec. 2017 г.
 *      Author: Eric
 */


#include "../inc/mh-z19b.h"

#include "FreeRTOS.h"
#include "../inc/ssn.h"
#include "commands.h"
#include "utils.h"
#include <stdlib.h>

#define ser_CHAR_DELAY		( 10 / portTICK_RATE_MS )


// ********************************************************************
int8_t calcCrc(char* pBuf, int8_t nSize) {
	int8_t nCrc = 0;

	for (int8_t nCnt = 0; nCnt < nSize; nCnt++) {
		nCrc+=pBuf[nCnt];
	}

	nCrc = 255 - nCrc;
	nCrc += 1;
	return (nCrc-1);
}

static void prvMHZ19USARTReadTask( void *pvParameters )
{
signed char cChar = 0;
char mhz19Buff[9];
uint8_t nCrc;
uint8_t nCnt = 0;
uint16_t nPpm;
int32_t nRes;
xSensorMessage sSensorMsg;

	sDevice *pDev = (sDevice *) pvParameters; // get dev pointer
	sSensorMsg.pDev = pDev;
	sSensorMsg.nDevCmd = 0;

	if (!pDev) {
		vTaskDelay(portMAX_DELAY);	// wait forever
	}

	while (1) {
		/* Block to wait for a first character to be received on UART */
		xSerialGetChar( pDev->pGroup->GrpDev.nReserve, &cChar, portMAX_DELAY);
		mhz19Buff[nCnt] = cChar;
		// get other 8 bytes:
		for (nCnt = 1; nCnt <= 8; nCnt++) {
			nRes = xSerialGetChar( pDev->pGroup->GrpDev.nReserve, &cChar, ser_CHAR_DELAY);
			if (nRes) {
				mhz19Buff[nCnt] = cChar;
			} else {
				break;
			}
		}

		if (nRes) {
			nCrc = calcCrc(mhz19Buff, 8);

			// check CRC and response data:
			if (mhz19Buff[0] == 0xFF && mhz19Buff[1] == 0x86 && mhz19Buff[8] == nCrc) {
				nPpm = (256*(uint16_t)mhz19Buff[2]) + (uint16_t)mhz19Buff[3];
				pDev->pDevStruct = (void*)(uint32_t)nPpm; // instead void* in this attribute store actual value!

				if (abs((uint32_t)pDev->pDevStruct - pDev->nLastPinValue) >= pDev->uiDeltaValue) {
					pDev->nLastPinValue = (uint32_t)pDev->pDevStruct;
					pDev->uiLastUpdate = rtc_get_counter_val();

					nRes = xQueueSend(xSensorsQueue, &sSensorMsg, 0);
				}
			}
		}

		nCnt = 0;
	}
}

// ********************************************************************
int32_t mhz19DeviceSendCommandReadCO2(sDevice *pDev) {
	const char data_cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; // command read sensor CO2 data

	return lSerialPutString( pDev->pGroup->GrpDev.nReserve, data_cmd, 9 );
}

// sGrpDev.nReserve - UART port number (0..5)

int32_t mhz19DeviceInit(sDevice *pDev) {

	int32_t nRes = 0;

	xprintfMsg("\r\nMH-Z19B CO2 sensor start initialing [%d]", pDev->nId);

	if (!pDev) {
		nRes = -1;
		goto mhz19InitEnd;
		// error
	}
// check UART port: if it equal base port, then error:
	if (pDev->pGroup->GrpDev.nReserve == mainBASECOM) {
		nRes = -2; // UART port number error
		xprintfMsg("\r\ninit. MH-Z19B UART=%d", (pDev->pGroup->GrpDev.nReserve + 1));
		goto mhz19InitEnd;
	}

	// initialize UART port at 9600:
	nRes = lCOMPortInit( pDev->pGroup->GrpDev.nReserve, 9600, mainBASECOM_Priority );
	if (!pDev) {
		nRes = -3;
		goto mhz19InitEnd;
		// UART port init error
	}

	const char setrangeA_cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB}; // command set range 0 - 5000ppm

	nRes = lSerialPutString( pDev->pGroup->GrpDev.nReserve, setrangeA_cmd, 9 );

//	int8_t nCnt = 0;
//	signed char cChar;
//	char mhz19Buff[9];
//	for (nCnt = 0; nCnt <= 9; nCnt++) {
//		nRes = xSerialGetChar( pDev->pGroup->GrpDev.nReserve, &cChar, ser_CHAR_DELAY);
//		if (nRes) {
//			mhz19Buff[nCnt] = cChar;
//		} else {
//			break;
//		}
//	}

	if (!nRes) {
		nRes = -4;
		goto mhz19InitEnd;
		// Get sensor response error
	}

	// calculate CRC
//	int8_t nCrc = calcCrc(mhz19Buff, 8);
//
//	// check CRC and response data:
//	if ( !(mhz19Buff[0] == 0xFF && mhz19Buff[1] == 0x99 && mhz19Buff[8] == nCrc) ) {
//		nRes = -5;
//		goto mhz19InitEnd;
//		// response data or CRC error
//	}

	nRes = xTaskCreate( prvMHZ19USARTReadTask, ( char * ) "MHZ19USART", 250, pDev, mainECHO_TASK_PRIORITY, NULL );
	if (!nRes) {
		nRes = -6;
		goto mhz19InitEnd;
		// Sensor serial task create error
	}


	mhz19InitEnd: if (nRes > 0) {
		xprintfMsg("\r\nOk. MH-Z19B CO2 sensor initialized: %d result code=%d", pDev->nId, nRes);
	} else {
		xprintfMsg("\r\nError! MH-Z19B CO2 sensor error code=%d: %d ", nRes, pDev->nId);
	}

	return nRes;

}

void deviceProcAttributes_mhz19(sDevice* pDev, char* sName, char* sValue) {

	// delta temperature store in pDev->uiDeltaValue!
	if (strcmp(sName, "uart") == 0) {
		pDev->pGroup->GrpDev.nReserve = conv2d(sValue) - 1; // UART numeration in SSN start from 0, but in MC UARTs start from 1 number
	}
}

void 	mhz19_device_delete(sDevice *pDev) {
	vPortFree((void*)pDev->pDevStruct);
}


