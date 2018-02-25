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
#include "bme280dev.h"

void 			deviceProcAttributes_bme280(sDevice* pDev, char* sName, char* sValue);
bme280_t* 		bme280_device_init(sDevice *pDev, uint8_t nAddr);
void 			bme280_device_delete(sDevice *pDev);
int32_t 		bme280_get_data(sDevice *pDev);
int32_t 		bme280DeviceInit(sDevice *pDev);
bme280_t* 		bme280DeviceInitStruct(sDevice *pDev);

//int32_t 		bmp180_read_raw_temperature(sDevice *pDev);
//int32_t 		bmp180_read_raw_pressure(sDevice *pDev);


#endif /* INC_BME280_H_ */
