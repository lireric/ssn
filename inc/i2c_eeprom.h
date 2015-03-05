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
 * i2c_eeprom.h
 *
 *  Created on: 05 окт. 2014 г.
 *      Author: eric
 */

#ifndef I2C_EEPROM_H_
#define I2C_EEPROM_H_

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "device.h"
#include "bb_device.h"
#include "soft_i2c.h"

//uint8_t i2c_eeprom_read_data_single(uint32_t i2c, uint8_t device, uint8_t addr);
uint8_t eeprom_read(sGrpDev* pGrpDev, uint8_t eeprom_addr, uint16_t addr, uint8_t* buf, uint32_t size);
uint8_t eeprom_write(sGrpDev* pGrpDev, uint8_t eeprom_addr, uint16_t addr, uint8_t* buf, uint32_t size);

#endif /* I2C_EEPROM_H_ */
