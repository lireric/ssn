/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2016 Ernold Vasiliev <ericv@mail.ru>
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
 *  Created on: 29 сент. 2016 г.
 *      Author: eric
 */


#include "i2c_eeprom.h"
#include "FreeRTOS.h"
#include "../inc/ssn.h"

BMP180_data_t* bmp180_device_init(sGrpDev* pGrpDev, uint8_t nAddr) {
	BMP180_data_t* pBMP180Dev = NULL;
	int32_t nRes;
	uint8_t buf[22];

//	owi_device_init_i2c(pGrpDev);
	soft_i2c_init(pGrpDev);

	nRes = soft_i2c_ReadBufferAddress(pGrpDev, nAddr, BMP180_REG_CHIP_ID, buf, 1); // read chip id. it must be 0x55

	if (nRes && (buf[0] == 0x55)) {
		buf[0] = BMP180_SOFT_RESET_CMD;
		nRes = soft_i2c_WriteBufferAddress(pGrpDev, pBMP180Dev->I2C_Addr, BMP180_REG_SOFT_RESET, buf, 1); // reset
		delay_ms(pGrpDev, 10);

		nRes = soft_i2c_ReadBufferAddress(pGrpDev, nAddr, BMP180_REG_AC1_H, buf, 22);


		if (nRes) {
			pBMP180Dev = (BMP180_data_t*) pvPortMalloc(sizeof(BMP180_data_t));
			devArray[all_devs_counter]->pDevStruct = (void*) pBMP180Dev;
			if (pBMP180Dev) {

				((BMP180_data_t*) devArray[all_devs_counter]->pDevStruct)->I2C_Addr =
						nAddr;

				pBMP180Dev->AC1 = (buf[0] << 8) | buf[1];
				pBMP180Dev->AC2 = (buf[2] << 8) | buf[3];
				pBMP180Dev->AC3 = (buf[4] << 8) | buf[5];
				pBMP180Dev->AC4 = (buf[6] << 8) | buf[7];
				pBMP180Dev->AC5 = (buf[8] << 8) | buf[9];
				pBMP180Dev->AC6 = (buf[10] << 8) | buf[11];
				pBMP180Dev->B1 = (buf[12] << 8) | buf[13];
				pBMP180Dev->B2 = (buf[14] << 8) | buf[15];
				pBMP180Dev->MB = (buf[16] << 8) | buf[17];
				pBMP180Dev->MC = (buf[18] << 8) | buf[19];
				pBMP180Dev->MD = (buf[20] << 8) | buf[21];

				pBMP180Dev->uiLastUpdate = 0;
				pBMP180Dev->iTemperature = 0;
				pBMP180Dev->iPrevTemperature = 0;
				pBMP180Dev->uiPressure = 0;
				pBMP180Dev->uiPrevPressure = 0;
			}
		}
	}
	return pBMP180Dev;

}

void 	bmp180_device_delete(sDevice *pDev) {
	vPortFree((void*) (BMP180_data_t*)pDev->pDevStruct);
}

int32_t bmp180_get_data(sDevice *pDev) {
	int32_t nRes;
	int32_t X1, X2, X3, B3, B5, B6;
	uint32_t B4, B7;

	BMP180_data_t* pBMP180Dev = pDev->pDevStruct;

	// save previous values
	pBMP180Dev->iPrevTemperature = pBMP180Dev->iTemperature;
	pBMP180Dev->uiPrevPressure = pBMP180Dev->uiPressure;
	// read and calculate temperature:

	nRes = bmp180_read_raw_temperature(pDev);
	nRes = bmp180_read_raw_pressure(pDev);

	X1 = (pBMP180Dev->UT - pBMP180Dev->AC6) * pBMP180Dev->AC5 >> 15;
	X2 = (pBMP180Dev->MC << 11) / (X1 + pBMP180Dev->MD);
	B5 = X1 + X2;
	pBMP180Dev->iTemperature = ((B5 + 8) >> 4);

	// read and calculate pressure, result in pascal:

/*	X1 = ((pBMP180Dev->UT - pBMP180Dev->AC6) * pBMP180Dev->AC5) >> 15;
	X2 = (pBMP180Dev->MC << 11) / (X1 + pBMP180Dev->MD);
	B5 = X1 + X2;
*/


	B6 = B5 - 4000;

	X1 = (pBMP180Dev->B2 * (B6 * B6 >> 12)) >> 11;
	X2 = pBMP180Dev->AC2 * B6 >> 11;
	X3 = X1 + X2;

	B3 = (((pBMP180Dev->AC1 * 4 + X3) << pBMP180Dev->P_Oversampling) + 2) >> 2;
	X1 = pBMP180Dev->AC3 * B6 >> 13;
	X2 = (pBMP180Dev->B1 * (B6 * B6 >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;


	B4 = (pBMP180Dev->AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t) pBMP180Dev->UP - B3) * (50000 >> pBMP180Dev->P_Oversampling);

	if(B7 < 0x80000000) {
		pBMP180Dev->uiPressure = (B7 * 2) / B4;
	} else {
		pBMP180Dev->uiPressure = (B7 / B4) * 2;
	}

	X1 = (pBMP180Dev->uiPressure >> 8) * (pBMP180Dev->uiPressure >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * pBMP180Dev->uiPressure) >> 16;

	pBMP180Dev->uiPressure = pBMP180Dev->uiPressure + ((X1 + X2 + 3791)>>4);

	pBMP180Dev->uiLastUpdate = rtc_get_counter_val();

	return nRes;
}

/*
 * Returns the raw measured temperature value of this BMP180 sensor.
 *
 * @param bmp180 sensor
 */
int32_t bmp180_read_raw_temperature(sDevice *pDev) {
	BMP180_data_t* pBMP180Dev = pDev->pDevStruct;
	int32_t nRes;
	uint8_t buf[2];

	buf[0] = BMP180_TMP_READ_CMD;

	nRes = soft_i2c_WriteBufferAddress(&pDev->pGroup->GrpDev, pBMP180Dev->I2C_Addr, BMP180_CTRL, buf, 1);

//	delay_nus(&pDev->pGroup->GrpDev, BMP180_TMP_READ_WAIT_US); // 5 ms
	delay_ms(&pDev->pGroup->GrpDev, BMP180_TMP_READ_WAIT_US/1000);
	nRes = soft_i2c_ReadBufferAddress(&pDev->pGroup->GrpDev, pBMP180Dev->I2C_Addr, BMP180_REG_TMP, buf, 2);

	pBMP180Dev->UT = (buf[0] << 8) + buf[1];

	return nRes;
}

/*
 * Returns the raw measured pressure value of this BMP180 sensor.
 *
 * @param bmp180 sensor
 */
int32_t bmp180_read_raw_pressure(sDevice *pDev) {
	BMP180_data_t* pBMP180Dev = pDev->pDevStruct;
	int32_t nRes;
	uint8_t buf[3];

	uint16_t wait;
//	uint8_t cmd;

	switch(pBMP180Dev->P_Oversampling) {
		case BMP180_PRE_OSS0:
		default:
			wait = BMP180_PRE_OSS0_WAIT_US; buf[0] = BMP180_PRE_OSS0_CMD;
			break;

		case BMP180_PRE_OSS1:
			wait = BMP180_PRE_OSS1_WAIT_US; buf[0] = BMP180_PRE_OSS1_CMD;
			break;

		case BMP180_PRE_OSS2:
			wait = BMP180_PRE_OSS2_WAIT_US; buf[0] = BMP180_PRE_OSS2_CMD;
			break;

		case BMP180_PRE_OSS3:
			wait = BMP180_PRE_OSS3_WAIT_US; buf[0] = BMP180_PRE_OSS3_CMD;
			break;
	}

	nRes = soft_i2c_WriteBufferAddress(&pDev->pGroup->GrpDev, pBMP180Dev->I2C_Addr, BMP180_CTRL, buf, 1);

//	delay_nus(&pDev->pGroup->GrpDev, wait);
	delay_ms(&pDev->pGroup->GrpDev, wait/100);

//	int32_t msb, lsb, xlsb, data;
	nRes = soft_i2c_ReadBufferAddress(&pDev->pGroup->GrpDev, pBMP180Dev->I2C_Addr, BMP180_REG_PRE, buf, 3);
	pBMP180Dev->UP = ((buf[0] << 16)  + (buf[1] << 8)  + buf[3]) >> (8 - pBMP180Dev->P_Oversampling);

	return nRes;
}


