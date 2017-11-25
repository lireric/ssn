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
 * bme280.h
 *
 *BME280 atmosphere pressure, humidity and temperature sensor
 *
 *  Created on: 6.02.2017
 *      Author: eric
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

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
 * read value 0x60
 */
#define BMP280_REG_CHIP_ID 0xD0

/*
 * Ctrl_meas register (0xF4):
 *
 * The ctrl_meas register sets the pressure and temperature data acquisition options of the
device. The register needs to be written after changing “ctrl_hum” for the changes to become
effective.

Bit 7, 6, 5
Controls oversampling of temperature data. See "Temperature oversampling modes"

Bit 4, 3, 2
Controls oversampling of pressure data. See "Pressure oversampling modes"

Bit 1, 0
Controls the sensor mode of the device. See "Register settings mode"

 */
#define BMP280_CTRL 0xF4

/*
 Pressure oversampling modes (osrs_p[2:0])
 */
#define BMP280_PRE_OSS0  0 // skipped (output set to 0x8000)
#define BMP280_PRE_OSS1  1 // oversampling * 1
#define BMP280_PRE_OSS2  2 // oversampling * 2
#define BMP280_PRE_OSS4  3 // oversampling * 4
#define BMP280_PRE_OSS8  4 // oversampling * 8
#define BMP280_PRE_OSS16 5 // oversampling * 16

/*
 Temperature oversampling modes (osrs_t[2:0])
 */
#define BMP280_TMP_OSS0  0 // skipped (output set to 0x8000)
#define BMP280_TMP_OSS1  1 // oversampling * 1
#define BMP280_TMP_OSS2  2 // oversampling * 2
#define BMP280_TMP_OSS4  3 // oversampling * 4
#define BMP280_TMP_OSS8  4 // oversampling * 8
#define BMP280_TMP_OSS16 5 // oversampling * 16

/*
 Register settings mode (mode[1:0])
 */
#define BMP280_SET_0  0 // Sleep mode
#define BMP280_SET_1  1 // Forced mode
#define BMP280_SET_3  2 // Normal mode

/*
 * Status register:
 *
 * Bit 3
measuring[0]
Automatically set to ‘1’ whenever a conversion is running
and back to ‘0’ when the results have been transferred
to the data registers.
 *
 * Bit 0
im_update[0]
Automatically set to ‘1’ when the NVM data are being
copied to image registers and back to ‘0’ when the
copying is done. The data are copied at power-on-reset
and before every conversion.
 *
 */
#define BMP280_CTRL 0xF3

/*
 * Soft reset register
 */
#define BMP280_REG_SOFT_RESET 0xE0
#define BMP280_SOFT_RESET_CMD 0xB6

/*
 * Pressure register
 *

Register 0xF7...0xF9 "press" (_msb, _lsb, _xlsb)
The "press" register contains the raw pressure measurement output data up[19:0].

0xF7 - press_msb[7:0]
Contains the MSB part up[19:12] of the raw pressure measurement output data.

0xF8 - press_lsb[7:0]
Contains the LSB part up[11:4] of the raw pressure measurement output data.

0xF9 (bit 7, 6, 5, 4)
press_xlsb[3:0]
Contains the XLSB part up[3:0] of the raw pressure measurement output data.
Contents depend on temperature resolution.

 */
#define BMP280_REG_PRESS 0xF7

/*
 * Temperature register
 *

Register 0xFA...0xFC "temp" (_msb, _lsb, _xlsb)
The "temp" register contains the raw temperature measurement output data ut[19:0].

0xFA - temp_msb[7:0]
Contains the MSB part ut[19:12] of the raw temperature measurement output data.

0xFB - temp_lsb[7:0]
Contains the LSB part ut[11:4] of the raw temperature measurement output data.

0xFC (bit 7, 6, 5, 4) - temp_xlsb[3:0]
Contains the XLSB part ut[3:0] of the raw temperature measurement output data.
Contents depend on pressure resolution.

 */
#define BMP280_REG_TEMP 0xFA

/*
 * Humidity register
 *

Register 0xFD...0xFE "hum" (_msb, _lsb)
The "hum" register contains the raw humidity measurement output data uh[19:0].

0xFD - hum_msb[7:0]
Contains the MSB part uh[15:8] of the raw humidity measurement output data.

0xFE - temp_lsb[7:0]
Contains the LSB part uh[7:0] of the raw humidity measurement output data.

 */
#define BMP280_REG_HUM 0xFD

/*
 Config register (0xF5)

The "config" register sets the rate, filter and interface options of the device. Writes to the "config"
register in normal mode may be ignored. In sleep mode writes are not ignored.

Bit 7, 6, 5
Controls inactive duration t(standby) in normal mode. See t_sb[2:0]

Bit 4, 3, 2
Controls the time constant of the IIR filter. See filter[2:0]

Bit 0
Enables 3-wire SPI interface when set to "1". See spi3w_en[0]

 */
#define BMP280_REG_CONFIG 0xF5

/*
 * t_sb[2:0] settings (ms)
 */
#define BMP280_TSB_0 0 // 0.5
#define BMP280_TSB_1 1 // 62
#define BMP280_TSB_2 2 // 125
#define BMP280_TSB_3 3 // 250
#define BMP280_TSB_4 4 // 500
#define BMP280_TSB_5 5 // 1000
#define BMP280_TSB_6 6 // 10
#define BMP280_TSB_7 7 // 20

/*
 * filter[2:0] settings
 */
#define BMP280_FILTER_0 0 // filter off
#define BMP280_FILTER_1 1 // 2
#define BMP280_FILTER_2 2 // 4
#define BMP280_FILTER_3 3 // 8
#define BMP280_FILTER_4 4 // 16

/*
 * Humidity register
 */
#define BMP280_REG_PRE 0xF2

/*
 * Humidity oversampling modes
 */
#define BMP280_HUM_OSS0 0 // skipped (output set to 0x8000)
#define BMP280_HUM_OSS1 1 // oversampling * 1
#define BMP280_HUM_OSS2 2 // oversampling * 2
#define BMP280_HUM_OSS4 3 // oversampling * 4
#define BMP280_HUM_OSS8 4 // oversampling * 8
#define BMP280_HUM_OSS16 5 // oversampling * 16

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

BMP180_data_t* 	bmp180_device_init(sDevice *pDev, uint8_t nAddr);
void 			bmp180_device_delete(sDevice *pDev);
int32_t 		bmp180_get_data(sDevice *pDev);
int32_t 		bmp180_read_raw_temperature(sDevice *pDev);
int32_t 		bmp180_read_raw_pressure(sDevice *pDev);


#endif /* INC_BME280_H_ */
