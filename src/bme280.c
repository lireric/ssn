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


#include "bme280.h"
#include "bme280dev.h"
#include "FreeRTOS.h"
#include "../inc/ssn.h"
#include "commands.h"
#include "utils.h"

s8 BME280_I2C_bus_write(sDevice* pDev, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	int8_t nRes = pdFALSE;
	if (pDev)
		if ((bme280_t*)pDev->pDevStruct)
			nRes = soft_i2c_WriteBufferAddress(&pDev->pGroup->GrpDev, ((bme280_t*)pDev->pDevStruct)->dev_addr, reg_addr, (uint8_t*)reg_data, cnt);
	return nRes;
}

s8 BME280_I2C_bus_read(sDevice* pDev, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	int8_t nRes = pdFALSE;
	if (pDev)
		if ((bme280_t*)pDev->pDevStruct)
			nRes = soft_i2c_ReadBufferAddress(&pDev->pGroup->GrpDev, ((bme280_t*)pDev->pDevStruct)->dev_addr, reg_addr, (uint8_t*)reg_data, cnt);
	return nRes;
}

void BME280_delay_msek(sDevice* pDev, u32 msek)
{
	//	delay_nus(&pDev->pGroup->GrpDev, wait);
		delay_ms(&pDev->pGroup->GrpDev, msek);
}

// ********************************************************************

bme280_t* bme280DeviceInitStruct(sDevice *pDev)
{
	bme280_t* pBME280Dev = NULL;

	pBME280Dev = (bme280_t*) pvPortMalloc(sizeof(bme280_t));
	if (pBME280Dev) {
		pBME280Dev->pDevice = pDev;
	}
	return pBME280Dev;
}

void deviceProcAttributes_bme280(sDevice* pDev, char* sName, char* sValue) {

	// delta temperature store in pDev->uiDeltaValue!
	if (strcmp(sName, "addr") == 0) {
		if (pDev->pDevStruct)
			((bme280_t*) pDev->pDevStruct)->dev_addr = conv2d(sValue);
	} else if (strcmp(sName, "oss") == 0) {
		// oversampling: "0" - normal, "1" - force
		if (pDev->pDevStruct)
			((bme280_t*) pDev->pDevStruct)->oss = conv2d(sValue);
	} else if (strcmp(sName, "dlp") == 0) {
		// delta pressure
		if (pDev->pDevStruct)
			((bme280_t*) pDev->pDevStruct)->uiDeltaPressure = conv2d(
					sValue);
	} else if (strcmp(sName, "dlh") == 0) {
		// delta Humidity
		if (pDev->pDevStruct)
			((bme280_t*) pDev->pDevStruct)->uiDeltaHumidity = conv2d(
					sValue);
	}
}


int32_t bme280DeviceInit(sDevice *pDev) {
	sGrpDev* pGrpDev = &pDev->pGroup->GrpDev;
	int32_t nRes = 0;

	if (!pDev) {
		nRes = -100;
		goto bmp280InitEnd;
		// error
	}

	if (!pGrpDev->pPort || !pGrpDev->pTimer) {
		nRes = -2;
		goto bmp280InitEnd;
		// error
	}

	bme280_t* pBMP280Dev = (bme280_t*) pDev->pDevStruct;
	if (!pBMP280Dev) {
		nRes = -3;
		goto bmp280InitEnd;
		// error
	}

	if (nRes) {
		pBMP280Dev = (bme280_t*) pDev->pDevStruct;

		if (pBMP280Dev) {

			pBMP280Dev->uiLastUpdate = 0;
			pBMP280Dev->iTemperature = 0;
			pBMP280Dev->iPrevTemperature = 0;
			pBMP280Dev->uiPressure = 0;
			pBMP280Dev->uiPrevPressure = 0;
		}
	}

	soft_i2c_init(&pDev->pGroup->GrpDev);

	pBMP280Dev->bus_read = BME280_I2C_bus_read;
	pBMP280Dev->bus_write = BME280_I2C_bus_write;
	pBMP280Dev->delay_msec = BME280_delay_msek;

	nRes = bme280_init(pDev);
	if (nRes) {

		if (((bme280_t*) pDev->pDevStruct)->oss == 0) {
			// normal mode
			nRes += bme280_set_oversamp_pressure(pDev, BME280_OVERSAMP_16X);
			nRes += bme280_set_oversamp_temperature(pDev, BME280_OVERSAMP_2X);
			nRes += bme280_set_oversamp_humidity(pDev, BME280_OVERSAMP_8X);
//			nRes += bme280_set_oversamp_humidity(pDev, BME280_OVERSAMP_1X);

			nRes += bme280_set_standby_durn(pDev, BME280_STANDBY_TIME_125_MS);
//			nRes += bme280_set_standby_durn(pDev, BME280_STANDBY_TIME_1_MS);
			nRes += bme280_set_filter(pDev, BME280_FILTER_COEFF_16);

			nRes += bme280_set_power_mode(pDev, BME280_NORMAL_MODE);

		} else {
			// forced mode
			nRes += bme280_set_oversamp_pressure(pDev, BME280_OVERSAMP_1X);
			nRes += bme280_set_oversamp_temperature(pDev, BME280_OVERSAMP_1X);
			nRes += bme280_set_oversamp_humidity(pDev, BME280_OVERSAMP_1X);

			nRes += bme280_set_filter(pDev, BME280_FILTER_COEFF_OFF);
			nRes += bme280_set_power_mode(pDev, BME280_FORCED_MODE);

		}
	}
	bmp280InitEnd: if (nRes > 0) {
		xprintfMsg("\r\nOk. BME280 device initialized: %d result code=%d", pDev->nId, nRes);
	} else {
		xprintfMsg("\r\nError! BME280 device error code=%d: %d ", nRes,
				pDev->nId);
	}

	return nRes;

}

void 	bme280_device_delete(sDevice *pDev) {
	vPortFree((void*)pDev->pDevStruct);
}


