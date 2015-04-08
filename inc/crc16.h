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

/* crc.h
 *
 *  Created on: 06 февр. 2015 г.
*/

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include "stdint.h"

#define	CRC16CCITT
/* available algorithms (select one):
#define	CRC16CCITT
#define	CRC16CCITT_XMODEM
#define	CRC16CCITT_KERMIT
#define	CRC16CCITT_1D0F
#define	CRC16IBM
#define	CRC16X25
*/

typedef uint16_t bit_order_16(uint16_t value);
typedef uint8_t bit_order_8(uint8_t value);

uint16_t crc16(const uint8_t *data, uint16_t size);


#endif /* INC_CRC16_H_ */
