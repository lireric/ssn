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
 * DHT22.c
 *
 *  Created on: 08 окт. 2014 г.
 *      Author: eric
 */


#include "FreeRTOS.h"
#include "DHT22.h"
#include "../inc/ssn.h"

/* -- Measuring pulse period on pGrpDev->ucPin, us */
uint32_t measure_period_nus(sGrpDev* pGrpDev, uint32_t nTimeout) {
	uint32_t TIMCounter = 0;
	uint8_t pinStartValue;
	// Fix start pin value
	pinStartValue = (GPIO_IDR(pGrpDev->pPort) & (1 << pGrpDev->ucPin)) ? 1 : 0;

	bb_set_wire2 (pGrpDev);	// SETSCL
	/* Counter enable. */
	timer_direction_up(pGrpDev->pTimer);
	timer_enable_counter(pGrpDev->pTimer);
	timer_set_counter(pGrpDev->pTimer, 0);
	/* Start timer. */
	TIM_CR1(pGrpDev->pTimer) |= TIM_CR1_CEN;
	// Wait changing pin value or timeout:
	while ((TIMCounter < nTimeout) && (((GPIO_IDR(pGrpDev->pPort) & (1 << pGrpDev->ucPin)) ? 1 : 0) == pinStartValue)){
		TIMCounter = timer_get_counter(pGrpDev->pTimer);
	}
	timer_disable_counter(pGrpDev->pTimer);
	bb_clear_wire2 (pGrpDev); //  SCL_LOW
	return TIMCounter;
}

DHT_data_t* dht_device_init(sGrpDev* pGrpDev) {
	DHT_data_t* pDHTDev;

	pDHTDev = (DHT_data_t*) pvPortMalloc(sizeof(DHT_data_t));
	if (!pDHTDev) return pdFAIL;
	if (!pGrpDev->pTimer) return pdFAIL;

	pDHTDev->uiLastUpdate = 0;

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

	gpio_set_mode(pGrpDev->pPort, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, 1 << pGrpDev->ucPin);
	gpio_set_mode(pGrpDev->pPort, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, 1 << pGrpDev->ucPin2);

	/* Reset timer peripheral. */
	timer_reset(pGrpDev->pTimer);
	timer_set_mode(pGrpDev->pTimer, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);

	/* Reset prescaler value. */
	timer_set_prescaler(pGrpDev->pTimer, 72);

	/* Enable preload. */
	timer_disable_preload(pGrpDev->pTimer);

	/* Timer mode. */
	timer_one_shot_mode(pGrpDev->pTimer);

	/* Period */
	return pDHTDev;
}

void dht_device_delete(DHT_data_t* pDHTDev)
{
	vPortFree((void*)pDHTDev);
}

uint8_t dht_get_data (sGrpDev* pGrpDev, DHT_data_t* dht_data) {
	uint64_t data = 0;
	uint64_t bitmask = 0x0000008000000000;
	uint8_t pos = 40;	// read 40 bit
	uint8_t check_sum, currentbit;
	uint32_t nPulsePeriod;
	//reset DHT22
	bb_wire_out(pGrpDev);
	bb_clear_wire(pGrpDev);
	delay_nus(pGrpDev, DHT_TBE);	// 1ms
	bb_set_wire(pGrpDev);
	delay_nus(pGrpDev, DHT_TGO);
	bb_wire_in(pGrpDev);
	// Response to low time
	nPulsePeriod = measure_period_nus(pGrpDev, DHT_TIMEOUT);
	if ((nPulsePeriod < DHT_TREL_MIN) || (nPulsePeriod > DHT_TREL_MAX) || (nPulsePeriod >= DHT_TIMEOUT)) return DHT_COM_ERROR;

	// In response to high time
	delay_nus(pGrpDev, 3);  // some delay
	nPulsePeriod = measure_period_nus(pGrpDev, DHT_TIMEOUT);
	if ((nPulsePeriod < DHT_TREH_MIN) || (nPulsePeriod > DHT_TREH_MAX) || (nPulsePeriod >= DHT_TIMEOUT)) return DHT_COM_ERROR;
	// start reading
	while(pos > 0)
	{
		//  Signal "0", "1" low time
		delay_nus(pGrpDev, 1);  // some delay
		nPulsePeriod = measure_period_nus(pGrpDev, DHT_TIMEOUT);
		if (((nPulsePeriod < DHT_TLOW_MIN) || (nPulsePeriod > DHT_TLOW_MAX)) || (nPulsePeriod >= DHT_TIMEOUT)) return DHT_COM_ERROR;
		nPulsePeriod = measure_period_nus(pGrpDev, DHT_TIMEOUT);
		if  (nPulsePeriod >= DHT_TIMEOUT) return DHT_NO_CONN;
		else
			if ((nPulsePeriod > DHT_TH0_MIN) && (nPulsePeriod < DHT_TH0_MAX)) currentbit = 0;
			else
				if ((nPulsePeriod > DHT_TH1_MIN) && (nPulsePeriod < DHT_TH1_MAX)) currentbit = 1;
				else return DHT_COM_ERROR;

			if (currentbit) data += bitmask;
			pos--; bitmask  >>= 1;
			delay_nus(pGrpDev, 3);  // some delay
	}

	//release line
	bb_wire_out(pGrpDev);
	bb_set_wire(pGrpDev);

	check_sum = (uint8_t)(data>>8)+(uint8_t)(data>>16)+(uint8_t)(data>>24)+(uint8_t)(data>>32);
	dht_data->nPrevTemperature = dht_data->temperature;
	dht_data->temperature = (uint16_t)(data>>8);
	dht_data->nPrevHumidity = dht_data->humidity;
	dht_data->humidity = (uint16_t)(data>>24);
	dht_data->uiLastUpdate = rtc_get_counter_val();

	if ((uint8_t)data != check_sum) return DHT_CS_ERROR;

	return DHT_OK;
}

