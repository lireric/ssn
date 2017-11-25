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
 * bmp180.h
 *
 *BMP180 atmosphere pressure and temperature sensor
 *
 *  Created on: 29 сент. 2016 г.
 *      Author: eric
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "device.h"
#include "soft_i2c.h"
#include "stdint.h"

/*
 * AC register
 */
#define BMP180_REG_AC1_H 0xAA
#define BMP180_REG_AC2_H 0xAC
#define BMP180_REG_AC3_H 0xAE
#define BMP180_REG_AC4_H 0xB0
#define BMP180_REG_AC5_H 0xB2
#define BMP180_REG_AC6_H 0xB4

/*
 * B1 register
 */
#define BMP180_REG_B1_H 0xB6

/*
 * B2 register
 */
#define BMP180_REG_B2_H 0xB8

/*
 * MB register
 */
#define BMP180_REG_MB_H 0xBA

/*
 * MC register
 */
#define BMP180_REG_MC_H 0xBC

/*
 * MD register
 */
#define BMP180_REG_MD_H 0xBE

/*
 * Chip ID register
 */
#define BMP180_REG_CHIP_ID 0xD0

/*
 * AC register
 */
#define BMP180_CTRL 0xF4

/*
 * Soft reset register
 */
#define BMP180_REG_SOFT_RESET 0xE0
#define BMP180_SOFT_RESET_CMD 0xB6

/*
 * Temperature register
 */
#define BMP180_REG_TMP 0xF6

/*
 * Pressure register
 */
#define BMP180_REG_PRE 0xF6

/*
 * Temperature read command
 */
#define BMP180_TMP_READ_CMD 0x2E

/*
 *  Waiting time in us for reading temperature values
 */
#define BMP180_TMP_READ_WAIT_US 5000

/*
 * Pressure read commands
 */
#define BMP180_PRE_OSS0_CMD 0x34
#define BMP180_PRE_OSS1_CMD 0x74
#define BMP180_PRE_OSS2_CMD 0xB4
#define BMP180_PRE_OSS3_CMD 0xF4

/*
 * Waiting times in us for reading pressure values
 */
#define BMP180_PRE_OSS0_WAIT_US 5000
#define BMP180_PRE_OSS1_WAIT_US 8000
#define BMP180_PRE_OSS2_WAIT_US 14000
#define BMP180_PRE_OSS3_WAIT_US 26000

/*
 * Pressure oversampling modes
 */
/*
 * Pressure oversampling modes
 */
#define BMP180_PRE_OSS0 0 // ultra low power
#define BMP180_PRE_OSS1 1 // standard
#define BMP180_PRE_OSS2 2 // high resolution
#define BMP180_PRE_OSS3 3 // ultra high resoultion


typedef struct{
	int16_t 	iTemperature;
	int16_t 	iPrevTemperature;
	uint32_t	uiPressure;
	uint32_t	uiPrevPressure;
    uint32_t 	uiLastUpdate;		// last update device value
    uint32_t 	uiDeltaPressure;	// Humidity delta for change value tolerance

	/* Settings */
    uint8_t I2C_Addr;				//I2c address. Default value 0xEE
    uint8_t P_Oversampling;

	/* Internal data */
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	uint32_t UT;
	uint32_t UP;
} BMP180_data_t;

BMP180_data_t* 	bmp180DeviceInitStruct();
BMP180_data_t* 	bmp180DeviceInit(sDevice *pDev);
void 			bmp180_device_delete(sDevice *pDev);
int32_t 		bmp180_get_data(sDevice *pDev);
int32_t 		bmp180_read_raw_temperature(sDevice *pDev);
int32_t 		bmp180_read_raw_pressure(sDevice *pDev);


#endif /* INC_BMP180_H_ */
