/*
 * This file is part of the SSN project.
 *
 * Copyright (C) 2014-2015 Ernold Vasiliev <ericv@mail.ru>
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
 * DHT22.h
 *
 *  Created on: 08 окт. 2014 г.
 *      Author: eric
 */

#ifndef DHT22_H_
#define DHT22_H_

#include "device.h"
#include "bb_device.h"

/* Exported constants --------------------------------------------------------*/
#define MAX_TICS 10000
#define DHT_OK 0
#define DHT_NO_CONN 1
#define DHT_CS_ERROR 2
#define DHT_COM_ERROR 3

/* -- DH22 parameters --- */
#define DHT_TBE 1000		// ms, Host the start signal down time
#define DHT_TGO	30			// us, Bus master has released time
// Commented value - from datasheet
#define DHT_TREL_MIN	50 // 75	// us, Response to low time
#define DHT_TREL_MAX	90 // 85
#define DHT_TREH_MIN	65 // 75	// us, In response to high time
#define DHT_TREH_MAX	90 // 85
#define DHT_TLOW_MIN	38 // 48	// us, Signal "0", "1" low time
#define DHT_TLOW_MAX	75 // 55
#define DHT_TH0_MIN		18 // 22	// us, Signal "0" high time
#define DHT_TH0_MAX		35 // 30
#define DHT_TH1_MIN		58 // 68	// us, Signal "1" high time
#define DHT_TH1_MAX		80 // 75
#define DHT_TEN_MIN		38 // 45	// us, Sensor to release the bus time
#define DHT_TEN_MAX		60 // 55

#define DHT_TIMEOUT 500		// us

typedef struct{
	int16_t 	temperature;
	uint16_t	humidity;
    uint32_t 	uiLastUpdate;	// last update device value
} DHT_data_t;

DHT_data_t* 	dht_device_init(sGrpDev* pGrpDev);
void 			dht_device_delete(DHT_data_t* pDHTDev);
uint32_t 		measure_period_nus(sGrpDev* pGrpDev, uint32_t nTimeout);
uint8_t 		dht_get_data(sGrpDev* pGrpDev, DHT_data_t* dht_data);

#endif /* DHT22_H_ */
