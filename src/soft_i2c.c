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
 * soft_i2c.c
 *
 *  Created on: 05 окт. 2014 г.
 *      Author: eric
 */

#include "device.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "soft_i2c.h"

// SCL и SDA must be configured in GPIO_Mode_Out_OD

//uint8_t ack = 0;

//static const uint8_t I2CSWM_DIRECTION_TX = 0;
//static const uint8_t I2CSWM_DIRECTION_RX = 1;

//static const uint8_t I2CSWM_NACK = 1;
static const uint8_t I2CSWM_ACK = 0;

#define DELAY_TICS                          (12)

void soft_i2c_init(sGrpDev* pGrpDev)
{
	if (!pGrpDev->pPort || !pGrpDev->pTimer) {
		return; // error
	}
	rcc_periph_clock_enable(pGrpDev->pPort);
	switch (pGrpDev->pTimer) {
	case TIM1:
		rcc_periph_clock_enable(RCC_TIM1);
		break;
	case TIM2:
		rcc_periph_clock_enable(RCC_TIM2);
		break;
	case TIM3:
		rcc_periph_clock_enable(RCC_TIM3);
		break;
	case TIM4:
		rcc_periph_clock_enable(RCC_TIM4);
		break;
	case TIM5:
		rcc_periph_clock_enable(RCC_TIM5);
		break;
	}

	gpio_set_mode(pGrpDev->pPort, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGrpDev->ucPin);
	gpio_set_mode(pGrpDev->pPort, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGrpDev->ucPin2);

/* Reset timer peripheral. */
//	timer_reset(pGrpDev->pTimer);
	timer_disable_counter(pGrpDev->pTimer);
	timer_set_mode(pGrpDev->pTimer, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
/* Reset prescaler value. */
	timer_set_prescaler(pGrpDev->pTimer, 72);
/* Enable preload. */
	timer_disable_preload(pGrpDev->pTimer);
/* Timer mode. */
	timer_one_shot_mode(pGrpDev->pTimer);
/* Period */
	timer_set_period(pGrpDev->pTimer, 1);
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	soft_i2c_set_wire_SCL(pGrpDev);		// SETSCL
	timer_enable_counter(pGrpDev->pTimer);
}

/* clear pGrpDev->ucPin2
*
*/
void soft_i2c_clear_wire_SDA(sGrpDev* pGrpDev)
{
	gpio_clear(pGrpDev->pPort, 1<<pGrpDev->ucPin);
}

/* set pGrpDev->ucPin2
*
*/
void soft_i2c_set_wire_SDA(sGrpDev* pGrpDev)
{
	gpio_set(pGrpDev->pPort, 1<<pGrpDev->ucPin);
}

/* clear pGrpDev->ucPin2
*
*/
void soft_i2c_clear_wire_SCL(sGrpDev* pGrpDev)
{
	gpio_clear(pGrpDev->pPort, 1<<pGrpDev->ucPin2);
}

/* set pGrpDev->ucPin2
*
*/
void soft_i2c_set_wire_SCL(sGrpDev* pGrpDev)
{
	gpio_set(pGrpDev->pPort, 1<<pGrpDev->ucPin2);
}

// Read input data bit from SDA
uint8_t soft_i2c_read_SDA(sGrpDev* pGrpDev)
{
	volatile uint8_t result = 0;
	result = GPIO_IDR(pGrpDev->pPort) & (1 << pGrpDev->ucPin);

	if (result != 0)
		result = 1;
	else
		 result = 0;
	return result;
}


uint8_t soft_i2c_start(sGrpDev* pGrpDev)
{
// GENERATION START BIT

	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, DELAY_TICS);
	if (!soft_i2c_read_SDA(pGrpDev))
			return pdFALSE;  // error is line down by slave
	soft_i2c_clear_wire_SDA (pGrpDev); 	// SDA_LOW
	delay_nus (pGrpDev, DELAY_TICS);
	if (soft_i2c_read_SDA(pGrpDev))
			return pdFALSE;	// bus error
	delay_nus (pGrpDev, DELAY_TICS);
//	soft_i2c_clear_wire_SCL(pGrpDev); //  SCL_LOW
//	delay_nus (pGrpDev, 3);
	return pdTRUE;
}

void soft_i2c_stop(sGrpDev* pGrpDev)
{
	soft_i2c_clear_wire_SCL (pGrpDev);  // SCL_LOW
	delay_nus (pGrpDev, DELAY_TICS);
	soft_i2c_clear_wire_SDA (pGrpDev); 	//  SDA_LOW
	delay_nus (pGrpDev, DELAY_TICS);
	soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, DELAY_TICS);
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	delay_nus (pGrpDev, DELAY_TICS);
}


void soft_i2c_reset(sGrpDev* pGrpDev)
{
	soft_i2c_set_wire_SDA(pGrpDev);			// SETSDA
		uint8_t i;
		for (i = 0; i < 15; i++) {
			soft_i2c_set_wire_SCL(pGrpDev);	// SETSCL
			delay_nus(pGrpDev, DELAY_TICS*2);
			soft_i2c_clear_wire_SCL(pGrpDev); //  SCL_LOW
			delay_nus(pGrpDev, DELAY_TICS*2);
		}
		soft_i2c_set_wire_SDA(pGrpDev);			// SETSDA
		soft_i2c_set_wire_SCL(pGrpDev);		// SETSCL
}

void soft_i2c_ack(sGrpDev* pGrpDev)
{
	soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_clear_wire_SDA (pGrpDev); 	// SDA_LOW
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_clear_wire_SCL (pGrpDev); //  SCL_LOW
//	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
//	delay_nus (pGrpDev, 5);
}

void soft_i2c_NoAck(sGrpDev* pGrpDev)
{
	soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_clear_wire_SCL (pGrpDev); //  SCL_LOW
	delay_nus (pGrpDev, DELAY_TICS);
}

uint8_t soft_i2c_WaitAck(sGrpDev* pGrpDev)
{
	soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	delay_nus (pGrpDev, DELAY_TICS*2);
	soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
	delay_nus (pGrpDev, DELAY_TICS*2);
	if (soft_i2c_read_SDA(pGrpDev)) {
		soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
		return pdFALSE;
	}
	soft_i2c_clear_wire_SCL (pGrpDev); //  SCL_LOW
	return pdTRUE;
}

uint8_t soft_i2c_clock(sGrpDev* pGrpDev)
{
    uint8_t l;
    soft_i2c_set_wire_SCL (pGrpDev);		// SETSCL
    delay_nus (pGrpDev, DELAY_TICS);
    l = soft_i2c_read_SDA(pGrpDev);
	soft_i2c_clear_wire_SCL (pGrpDev);		// SCL_LOW
    delay_nus (pGrpDev, DELAY_TICS);
	return l;


}

// *******************************************************
uint8_t soft_i2c_write (sGrpDev* pGrpDev, uint8_t data)
{
	uint8_t db = 0x80;
	uint8_t i;
	soft_i2c_clear_wire_SCL (pGrpDev);		// SCL_LOW
	for (i = 0; i < 8; i++) {
		if (data & db) {
			soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
		} else {
			soft_i2c_clear_wire_SDA (pGrpDev); 	//  SDA_LOW
		}
		soft_i2c_clock(pGrpDev);
		db >>= 1;
	}

	/* Получим ACK */
	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
	/* отпускаем линию SDA */
	return soft_i2c_clock(pGrpDev);
}

uint8_t soft_i2c_read(sGrpDev* pGrpDev, uint8_t ack )
	{
		uint8_t rb = 0;
		uint8_t db = 0x80;
		uint8_t i;
		for (i = 0; i < 8; i++) {
			if (soft_i2c_clock(pGrpDev))
				rb |= db;
			db >>= 1;
		}

		if (ack == I2CSWM_ACK) {
			/* шлем ацк */
			soft_i2c_clear_wire_SDA (pGrpDev); 	//  SDA_LOW
			soft_i2c_clock(pGrpDev);
			soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
		} else {
			/* шлем нацк */
			soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
			soft_i2c_clock(pGrpDev);
		}
		return rb;
	}
// ****************************************************

void soft_i2c_PutByte(sGrpDev* pGrpDev, uint8_t data)
{
    uint8_t i = 8;
    while ( i-- ) {
    	soft_i2c_clear_wire_SCL (pGrpDev);		// SCL_LOW
    	delay_nus (pGrpDev, 10);
        if ( data & 0x80 )
        	soft_i2c_set_wire_SDA(pGrpDev);		// SETSDA
        else
        	soft_i2c_clear_wire_SDA (pGrpDev); 	// SDA_LOW
        data <<= 1;
        delay_nus (pGrpDev, DELAY_TICS*2);
        soft_i2c_set_wire_SCL (pGrpDev);		// SETSCL
        delay_nus (pGrpDev, DELAY_TICS*2);
    }
    soft_i2c_clear_wire_SCL (pGrpDev);			// SCL_LOW
}

uint8_t soft_i2c_GetByte(sGrpDev* pGrpDev)
{
    volatile uint8_t i = 8;
    uint8_t data = 0;

    soft_i2c_set_wire_SDA(pGrpDev);			// SETSDA (slave control bus)
    while ( i-- ) {
        data <<= 1;
        soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
        delay_nus (pGrpDev, DELAY_TICS);
        soft_i2c_set_wire_SCL (pGrpDev);	// SETSCL
        delay_nus (pGrpDev, DELAY_TICS);
        if ( soft_i2c_read_SDA(pGrpDev) ) {
            data |= 0x01;
        }
    }
    soft_i2c_clear_wire_SCL (pGrpDev);	// SCL_LOW
    return data;
}

// try to connect device by given address for pDevice. Return True or False
int32_t soft_i2c_TryAddress (sGrpDev* pGrpDev, uint8_t chipAddress)
{
    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress);
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }
    soft_i2c_stop(pGrpDev);
    delay_nus (pGrpDev, 20);
    return pdTRUE;

}

uint32_t soft_i2c_ReadBuffer (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer)
{
    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress + 1 );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
        *buffer = soft_i2c_GetByte(pGrpDev);

        buffer++;
        nCnt--;
        if ( nCnt == 0 ) {
        	soft_i2c_NoAck(pGrpDev);
            break;
        }
       else
        	soft_i2c_ack(pGrpDev);
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;
}

uint32_t soft_i2c_ReadBufferAddress (sGrpDev* pGrpDev, uint8_t chipAddress, uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer)
{


    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }
    soft_i2c_PutByte(pGrpDev, registerAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

	if ( !soft_i2c_start(pGrpDev) )
		return pdFALSE;

	soft_i2c_PutByte(pGrpDev, chipAddress + 1 );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
        *buffer = soft_i2c_GetByte(pGrpDev);

        buffer++;
        nCnt--;
        if ( nCnt == 0 ) {
        	soft_i2c_NoAck(pGrpDev);
            break;
        }
       else
        	soft_i2c_ack(pGrpDev);
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;
}

uint32_t soft_i2c_ReadBufferAddress16 (sGrpDev* pGrpDev, uint8_t chipAddress, uint16_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer)
{

    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

    // 16 bit address:
    soft_i2c_PutByte(pGrpDev, (registerAddress >> 8) & 0xff);
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
    }

   	soft_i2c_PutByte(pGrpDev, registerAddress & 0xff);
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

	if ( !soft_i2c_start(pGrpDev) )
		return pdFALSE;

	soft_i2c_PutByte(pGrpDev, chipAddress + 1 );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
        *buffer = soft_i2c_GetByte(pGrpDev);

        buffer++;
        nCnt--;
        if ( nCnt == 0 ) {
        	soft_i2c_NoAck(pGrpDev);
            break;
        }
       else
        	soft_i2c_ack(pGrpDev);
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;


}

int soft_i2c_WriteBuffer (sGrpDev* pGrpDev,  uint8_t chipAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{
    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
    	soft_i2c_PutByte(pGrpDev, *buffer );
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
        }

        buffer++;
        nCnt--;
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;
}

int soft_i2c_WriteBufferAddress (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint8_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{
    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }
        // 8 bit address:
        soft_i2c_PutByte(pGrpDev, registerAddress & 0xff);
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
        }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
    	soft_i2c_PutByte(pGrpDev, *buffer );
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
        }

        buffer++;
        nCnt--;
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;
}

int soft_i2c_WriteBufferAddress16 (sGrpDev* pGrpDev,  uint8_t chipAddress,  uint16_t registerAddress, uint8_t *buffer, uint32_t sizeOfBuffer )
{
    if ( !soft_i2c_start(pGrpDev) ) {
    	// try once more
    	soft_i2c_reset(pGrpDev);
    	if ( !soft_i2c_start(pGrpDev) )
    		return pdFALSE;
    }

    soft_i2c_PutByte(pGrpDev, chipAddress );
    if ( !soft_i2c_WaitAck(pGrpDev) ) {
    	soft_i2c_stop(pGrpDev);
        return pdFALSE;
    }
        // 16 bit address:
        soft_i2c_PutByte(pGrpDev, (registerAddress >> 8) & 0xff);
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
            	soft_i2c_stop(pGrpDev);
                return pdFALSE;
        }
        soft_i2c_PutByte(pGrpDev, registerAddress & 0xff);
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
        }

    uint32_t nCnt = sizeOfBuffer;
    while ( nCnt != 0 ) {
    	soft_i2c_PutByte(pGrpDev, *buffer );
        if ( !soft_i2c_WaitAck(pGrpDev) ) {
        	soft_i2c_stop(pGrpDev);
            return pdFALSE;
        }

        buffer++;
        nCnt--;
    }
    soft_i2c_stop(pGrpDev);
    return pdTRUE;

}


