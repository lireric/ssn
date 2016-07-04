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
 * Functions for work with device and actions data elements
 *
 */

#include "xprintf.h"
#include <string.h>
#include "commands.h"
#include "utils.h"
#include "rtc_func.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#define M_DS18B20
#define M_LCD
#define M_DHT
//#define M_GSM

#include "device.h"
#ifdef  M_DS18B20
	#include "DS18B20.h"
#endif
#ifdef  M_LCD
	#include "lcd-320x240.h"
	#include "touchscreen.h"
#endif
#ifdef  M_DHT
	#include "DHT22.h"
#endif
#ifdef  M_GSM
	#include "gsm.h"
#endif

int8_t  getTIM_OC(int8_t nChannel)
{
	int8_t nRet = 0;

	switch (nChannel) {
		case 1:	nRet = TIM_OC1;	break;
		case 2:	nRet = TIM_OC2;	break;
		case 3:	nRet = TIM_OC3;	break;
		case 4:	nRet = TIM_OC4;	break;
		}
	return nRet;
}

int8_t  getTIM_Pin(int32_t nTimer, int32_t nPort, int8_t nChannel)
{
	int8_t nRet = -1;

	if (nTimer == TIM1) {
		if (nPort == GPIOA) {
			switch (nChannel) {
				case 1:	nRet = 8;	break;
				case 2:	nRet = 9;	break;
				case 3:	nRet = 10;	break;
				case 4:	nRet = 11;	break;
				}
		} else if (nPort == GPIOE) { // remapping
			switch (nChannel) {
				case 1:	nRet = 9;	break;
				case 2:	nRet = 11;	break;
				case 3:	nRet = 13;	break;
				case 4:	nRet = 14;	break;
				}
		}
	}
	else if (nTimer == TIM2) {
		if (nPort == GPIOA) {
			switch (nChannel) {
				case 1:	nRet = 0;	break;
				case 2:	nRet = 1;	break;
				case 3:	nRet = 2;	break;
				case 4:	nRet = 3;	break;
				}
		} else if (nPort == GPIOB) { // remapping
			switch (nChannel) {
				case 1:	nRet = 0;	break; // ???
				case 2:	nRet = 3;	break;
				case 3:	nRet = 10;	break;
				case 4:	nRet = 11;	break;
				}
		}
	}
	else if (nTimer == TIM3) {
		if (nPort == GPIOA) {
			switch (nChannel) {
				case 1:	nRet = 6;	break;
				case 2:	nRet = 7;	break;
				case 3:	nRet = 0;	break; // PB0!
				case 4:	nRet = 1;	break; // PB1!
				}
		} else if (nPort == GPIOC) { // remapping
			switch (nChannel) {
				case 1:	nRet = 6;	break;
				case 2:	nRet = 7;	break;
				case 3:	nRet = 8;	break;
				case 4:	nRet = 9;	break;
				}
		}
	}
	else if (nTimer == TIM4) {
		if (nPort == GPIOB) {
			switch (nChannel) {
				case 1:	nRet = 6;	break;
				case 2:	nRet = 7;	break;
				case 3:	nRet = 8;	break;
				case 4:	nRet = 9;	break;
				}
		} else if (nPort == GPIOD) { // remapping
			switch (nChannel) {
				case 1:	nRet = 12;	break;
				case 2:	nRet = 13;	break;
				case 3:	nRet = 14;	break;
				case 4:	nRet = 15;	break;
				}
		}
	}
	else if (nTimer == TIM5) {
		if (nPort == GPIOA) {
			switch (nChannel) {
				case 1:	nRet = 0;	break;
				case 2:	nRet = 1;	break;
				case 3:	nRet = 2;	break;
				case 4:	nRet = 3;	break;
				}
		}
	}
	else if (nTimer == TIM8) {
		if (nPort == GPIOC) {
			switch (nChannel) {
				case 1:	nRet = 6;	break;
				case 2:	nRet = 7;	break;
				case 3:	nRet = 8;	break;
				case 4:	nRet = 9;	break;
				}
		}
	}
	return nRet;
}

int32_t pwm_setup(sDevice* pDev, char* psChannels)
{
	int32_t nRes = pdPASS;
	sGrpInfo* pGrp = pDev->pGroup;

	pGrp->iDevQty = parseCommaString(psChannels, ((sPWM_data_t*)pDev->pDevStruct)->nChannelArray, 4);

	if (pGrp->iDevQty == 0 || ((sPWM_data_t*)pDev->pDevStruct)->nFreq == 0) {
		nRes = pdFAIL;
		goto pwm_setup_ret;
	}

	rcc_periph_clock_enable(get_rcc_by_port(pGrp->GrpDev.pPort));
	rcc_periph_clock_enable(get_rcc_by_port(pGrp->GrpDev.pTimer));
	rcc_periph_clock_enable(RCC_AFIO);

	// check remapping or not:
	if (((pGrp->GrpDev.pTimer == TIM1) && (pGrp->GrpDev.pPort == GPIOE)) || ((pGrp->GrpDev.pTimer == TIM2) && (pGrp->GrpDev.pPort == GPIOB))
			|| ((pGrp->GrpDev.pTimer == TIM3) && (pGrp->GrpDev.pPort == GPIOC)) || ((pGrp->GrpDev.pTimer == TIM4) && (pGrp->GrpDev.pPort == GPIOD)))
	{
		gpio_primary_remap(0, AFIO_MAPR_TIM3_REMAP_FULL_REMAP); // to do - auto define remap or not!
	}
	timer_reset(pGrp->GrpDev.pTimer);
	timer_set_mode(pGrp->GrpDev.pTimer, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Reset prescaler value. */
    timer_set_prescaler(pGrp->GrpDev.pTimer, ((sPWM_data_t*)pDev->pDevStruct)->nFreq); // to do: calculate from pref freq. value
	timer_set_period(pGrp->GrpDev.pTimer, 65535);

    /* Continuous mode. */
    timer_continuous_mode(pGrp->GrpDev.pTimer);

	for (int8_t i = 0; i < pGrp->iDevQty; i++)
	{
		int8_t nTimCC = getTIM_OC(((sPWM_data_t*)pDev->pDevStruct)->nChannelArray[i]);
		int8_t nPin = getTIM_Pin(pGrp->GrpDev.pTimer, pGrp->GrpDev.pPort, ((sPWM_data_t*)pDev->pDevStruct)->nChannelArray[i]);
		if (nPin < 0) { break; } // may be error

		gpio_set_mode(pGrp->GrpDev.pPort, GPIO_MODE_OUTPUT_50_MHZ,
				GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 1 << nPin);
		/* Disable outputs. */
		timer_disable_oc_output(pGrp->GrpDev.pTimer, nTimCC);

		/* Configure global mode of line 1. */
		timer_disable_oc_clear(pGrp->GrpDev.pTimer, nTimCC);
		timer_enable_oc_preload(pGrp->GrpDev.pTimer, nTimCC);
		timer_set_oc_slow_mode(pGrp->GrpDev.pTimer, nTimCC);
		timer_set_oc_mode(pGrp->GrpDev.pTimer, nTimCC, TIM_OCM_PWM1);

		/* Set the capture compare value for OC1. */
		timer_set_oc_value(pGrp->GrpDev.pTimer, nTimCC, 32000 ); // default - 50%

		/* Reenable outputs. */
		timer_enable_oc_output(pGrp->GrpDev.pTimer, nTimCC);
	}

    /* ARR reload enable. */
    timer_enable_preload(pGrp->GrpDev.pTimer);

	timer_enable_counter(pGrp->GrpDev.pTimer);

pwm_setup_ret:
	return nRes;
}


int32_t adc_setup(sDevice* pDev, char* psChannels)
{
	int i=0;
	int32_t nRes = pdPASS;
	sGrpInfo* pGrp = pDev->pGroup;

	pGrp->iDevQty = parseCommaString(psChannels, ((sADC_data_t*)pDev->pDevStruct)->nChannelArray, 16);

	if (pGrp->iDevQty == 0) {
		nRes = pdFAIL;
		goto adc_setup_ret;
	}

	for (i = 0; i < pGrp->iDevQty; i++)
	{
		gpio_set_mode(pGrp->GrpDev.pPort, GPIO_MODE_INPUT,
					GPIO_CNF_INPUT_ANALOG, 1 << ((sADC_data_t*)pDev->pDevStruct)->nChannelArray[i]);
	}

	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_enable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);


/* We want to read the temperature sensor, so we have to enable it. */
adc_enable_temperature_sensor(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

adc_setup_ret:
	return nRes;
}


sGrpInfo* getGroupByID (uint8_t nGrpID)
{
	uint8_t i;
	sGrpInfo* pGrp = NULL;

	for (i = 0; i <= grp_counter; i++) {
			if (grpArray[i]->uiGroup == nGrpID) {
				pGrp = grpArray[i];
				break;
			}
	}

	return pGrp;
}

sDevice* getDeviceByID (uint16_t nDevID)
{
	uint16_t i;
	sDevice* pDev = 0;

	for (i = 0; i <= all_devs_counter; i++) {
			if (devArray[i]->nId == nDevID) {
				pDev = devArray[i];
				break;
			}
	}
	return pDev;
}

int16_t	getDeviceObjByID (uint16_t nDevID)
{
	uint16_t uiObj = 0;
	sDevice* pDev = getDeviceByID(nDevID);
	if (pDev) {
		uiObj = pDev->pGroup->uiObj;
	}
	return uiObj;
}

int32_t getDevValueByID(uint8_t nValCode, uint16_t nDevID)
{
	uint16_t i;
	int32_t res = 0;
	for (i = 0; i <= all_devs_counter; i++) {
			if (devArray[i]->nId == nDevID) { res = getDevValue(nValCode, devArray[i]); break;}
	}
	return res;
}

int32_t getDevValue(uint8_t nValCode, sDevice* dev)
{
	int32_t nDevValue = 0;
	uint32_t nDevLastUpdate;

	getDevData(dev, nValCode, &nDevValue, &nDevLastUpdate);

	return nDevValue;
}

uint32_t getDevLastUpdate(uint8_t nValCode, sDevice* dev)
{
	int32_t nDevValue = 0;
	uint32_t nDevLastUpdate = 0;

	getDevData(dev, nValCode, &nDevValue, &nDevLastUpdate);

	return nDevLastUpdate;
}

int32_t getDevData(sDevice* dev, uint8_t nValCode, int32_t* nDevValue, uint32_t* nDevLastUpdate)
{
	int32_t nValue;
	uint32_t nLastUpdate = 0;
	int32_t *e;

	if (!dev) return pdFALSE;

	nLastUpdate = dev->uiLastUpdate;

	switch (dev->ucType) {
	case device_TYPE_DS18B20:
		nValue = (int32_t)((ds18b20_device*) dev->pDevStruct)->iDevValue;
		nLastUpdate = ((ds18b20_device*) dev->pDevStruct)->uiLastUpdate;
		break;
	case device_TYPE_DHT22:
// if nValCode==0 return temperature, else - humidity:
		nValue = (nValCode==0)?(int32_t)(((DHT_data_t*) dev->pDevStruct)->temperature):((int32_t)((DHT_data_t*) dev->pDevStruct)->humidity);
		nLastUpdate = ((DHT_data_t*) dev->pDevStruct)->uiLastUpdate;
		break;
	case device_TYPE_BB1BIT_IO_PP:
	case device_TYPE_BB1BIT_IO_OD:
	case device_TYPE_BB1BIT_IO_INPUT:
		// nValue = (int32_t) bb_read_wire_data_bit(&dev->pGroup->GrpDev);
		nValue = dev->nLastPinValue;
		//nLastUpdate = rtc_get_counter_val();
		//nLastUpdate = dev->uiLastUpdate;
		break;
	case device_TYPE_MEMORY:
		e = (int32_t*)dev->pDevStruct;
		nValue = e[nValCode];
		break;
	case device_TYPE_BB1BIT_IO_AI:
		nValue = ((sADC_data_t*)dev->pDevStruct)->nADCValueArray[nValCode];
		//nLastUpdate = ((sADC_data_t*)dev->pDevStruct)->uiLastUpdate;
		//nLastUpdate = dev->uiLastUpdate;
		break;
	}

	*nDevValue = nValue;
	*nDevLastUpdate = nLastUpdate;

	return pdTRUE;
}
/* return number available value codes for device type */
uint8_t getNumDevValCodes(uint8_t ucType)
{
	uint8_t n;
	switch (ucType) {
	case device_TYPE_DHT22:
		n = 2;
		break;
	case device_TYPE_DS18B20:
	case device_TYPE_BB1BIT_IO_PP:
	case device_TYPE_BB1BIT_IO_OD:
	case device_TYPE_BB1BIT_IO_INPUT:
		n = 1;
		break;
	default:
		n = 0;
	}
	return n;
}

void setDevValueByID(int32_t nValue, uint8_t nDevCmd, uint16_t nDevID, uint8_t nDataType)
{
	uint16_t i;

// if DevID == 0 then search action with ActID == nValue, else search device:

	if (nDevID == 0) {
		for (i=0; i<act_counter;i++) {
			if (actArray[i]->nActId == nValue) {
				vMainStartTimer(actArray[i]);
			}
		}
	} else {
		for (i = 0; i <= all_devs_counter; i++) {
			if (devArray[i]->nId == nDevID) {
				if (nDataType == eElmDevValue) {
					nValue = getDevValueByID(nDevCmd, (uint16_t) nValue); // for eElmDevValue datatype nValue = devID
				} else {
					setDevValue(nValue, nDevCmd, devArray[i], nDataType);
				}
				break;
			}
		}
		// if local device array not contain this device, send to input queue for routing
		if (i > all_devs_counter)
		{
			char msg[mainMAX_MSG_LEN];
			if (nDataType == eElmString) {
				xsprintf(msg, "{\"ssn\":{\"v\":1,\"obj\":0,\"cmd\":\"sdv\", \"data\": {\"adev\":%d,\"acmd\":%d,\"aval\":\"%s\"}}}", nDevID, nDevCmd, (char*)nValue);
				vPortFree((void*)nValue);
			} else {
				xsprintf(msg, "{\"ssn\":{\"v\":1,\"obj\":0,\"cmd\":\"sdv\", \"data\": {\"adev\":%d,\"acmd\":%d,\"aval\":\"%d\"}}}", nDevID, nDevCmd, nValue);
			}
			char* buf = pvPortMalloc(strlen(msg));
			if (buf)
			{
				memcpy(buf, &msg, strlen(msg) + 1);
				vSendInputMessage(1, 0, mainJSON_MESSAGE, 0, 0, nDevID, (void*) buf, strlen(buf), 0);
			}
		}
	}
}

void setDevValue(int32_t nValue, uint8_t nDevCmd, sDevice* dev, uint8_t nDataType)
{
	int32_t *e;
	switch (dev->ucType) {
// ignore:
	case device_TYPE_DS18B20:
	case device_TYPE_DHT22:
	case device_TYPE_BB1BIT_IO_INPUT:
		break;
	case device_TYPE_BB1BIT_IO_PP:
	case device_TYPE_BB1BIT_IO_OD:
		if (nDataType == eElmInteger) {
			if (nValue > 0) { bb_set_wire(&dev->pGroup->GrpDev); } else { bb_clear_wire (&dev->pGroup->GrpDev);}
		} else {
			// to do: process error and free string resource
		}
		break;
	case device_TYPE_MEMORY:
		e = (int32_t*)dev->pDevStruct;
		if (e) {
			e[nDevCmd] = nValue;
			dev->uiLastUpdate = rtc_get_counter_val();
		//= nValue;
		}
		break;
	case device_TYPE_PWM:
		timer_set_oc_value(dev->pGroup->GrpDev.pTimer, getTIM_OC(nDevCmd), nValue); // channel = nDevCmd, % = nValue
		break;
	case device_TYPE_GSM:
#ifdef  M_GSM
		setGSMDevValue(nValue, nDevCmd, dev, nDataType);
#endif
		break;
	}
	// log device value changing for local device:
//	logAction(0, dev->nId, nDevCmd, nValue);

}


void	clearActionsDeviceCash (sDevice* pDev)
{
	if (pDev) {
		vPortFree(pDev->pActionsCash);
		pDev->pActionsCash = NULL;
	}
}


/* clear old cash info, reallocate memory and fill new arrays */
int32_t	refreshActions2DeviceCash ()
{
	uint16_t nActIndex;
	uint16_t nDevIndex;
	sDevice* pDev;
	sAction* pAct;
	sEvtElm* pEvtElm;
	sAction* pTmpActArray[devMAX_ACTIONS_CASH];
	uint8_t nTmpArrayIndex;
	uint8_t i;
	uint32_t res = pdTRUE;

	vTaskSuspendAll();

	for (nDevIndex = 0; nDevIndex <= all_devs_counter; nDevIndex++)
	{
		pDev = devArray[nDevIndex];
		if (pDev) {
			clearActionsDeviceCash(pDev);
			nTmpArrayIndex = 0;
			// scan all actions for this device
			for (nActIndex = 0; nActIndex < act_counter; nActIndex++)
			{
				pAct = actArray[nActIndex];
				if (pAct) {
					pEvtElm = pAct->pStartElmEvents;
					do {
						if (pEvtElm) {
							if (pEvtElm->nElmType == eElmInterval) {
								// skip action if interval type
								break;
							}
							if ((pEvtElm->nElmType == eElmDevValue) && (pEvtElm->nElmData1 == pDev->nId)) {
								// minimum once use this device in action -> add to array and break scan thru other elements
								pTmpActArray[nTmpArrayIndex++] = pAct;
								break;
							} else
								// search time events for abstract device[0]
								if (pDev->nId == 0)
								{
								   	  switch(pEvtElm->nElmType)
								   	  {
											case  eElmTimeValue:
											case  eElmDateYear:
											case  eElmDateMonth:
											case  eElmDateDay:
											case  eElmDateDayWeek:
											{
												pTmpActArray[nTmpArrayIndex++] = pAct;
												break;
											}
								   	  }
								}
							// scan left to right
							pEvtElm = pEvtElm->pPrevElm;
						}
					} while (pEvtElm);
				}
			}
			// if find any action for this device, than add it to cash
			if (nTmpArrayIndex > 0)
			{
				pDev->pActionsCash = pvPortMalloc(nTmpArrayIndex * sizeof(sAction*));
				pDev->nActionsCashSize = nTmpArrayIndex;
				for (i=0; i < nTmpArrayIndex; i++) {
					((sAction**)pDev->pActionsCash)[i] = pTmpActArray[i];
				}
			}
		}
	}

	xTaskResumeAll();

	return res;
}


/*
 * Scan actions associated with device and call action processing if needed
 * return pdTRUE if success
 */
int32_t	scanDevActions (sDevice* dev)
{
	int32_t res = pdTRUE;
	sAction* pAct;

	for (uint8_t i=0; i<dev->nActionsCashSize; i++) {
		pAct = ((sAction**)dev->pActionsCash)[i];
		if (pAct) {
			res = calcAndDoAction (pAct);
			if (!res) {
				// to do: process error
				res = pdFAIL;
			}
		}
	}
	return res;
}

