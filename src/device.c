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
#include "dev_memory.h"

#define M_DS18B20
#define M_LCD
#define M_DHT
//#define M_GSM
#define M_BMP180

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
#ifdef  M_BMP180
	#include "bmp180.h"
#endif
#ifdef  M_BME280
	#include "bme280.h"
	#include "bme280dev.h"
#endif

#ifdef  M_GSM
	#include "gsm.h"
#endif
#ifdef  M_STEPMOTOR
	#include "stepmotor.h"
#endif
#ifdef  M_MHZ19
	#include "mh-z19b.h"
#endif

/* The devices initialization task */
static void pDevInitTask( void *pvParameters ) {
	uint16_t nDevId;
	/* Process all requests from queue */
	while(xQueueReceive( xDevInitQueue, &nDevId, 0 ) == pdPASS) {

		deviceInit(getDeviceByID(nDevId));

	}
	vTaskDelete( NULL ); // destroy it task after complete
}

/*
 * Add request to init device
 */
int32_t addInitDevRequest(uint16_t 	nDevId) {

	return xQueueSend( xDevInitQueue, &nDevId, 0 );
}

/*
 * Start initialization task for some devices:
 * task processes xDevInitQueue queue and perform special init method
 */
void completeAllInit() {
	int32_t nRet;

	nRet = xTaskCreate( pDevInitTask, ( char * ) "DevInitTask", 200, NULL, tskIDLE_PRIORITY + 1, NULL );
	(void) nRet;
}

/*
 * Perform parsing initialization parameters from preferences processor handler
 * take dev pointer, parameter name and parameter value
 */
int32_t devicePreInit(sDevice* pDev, char* sName, char* sValue) {
//	int32_t nRes = pdPASS;
	if (pDev) {
		if (strcmp(sName, "devid") == 0) {
			pDev->nId = conv2d(sValue);

		} else if (strcmp(sName, "dl") == 0) {
			pDev->uiDeltaValue = conv2d(sValue);

		} else if (strcmp(sName, "devtype") == 0) {
			// section "dev:devtype" --------------------------------------------
			pDev->ucType = conv2d(sValue);

			if (pDev->ucType == device_TYPE_DS18B20) {
				pDev->pDevStruct = NULL;

			} else if (pDev->ucType == device_TYPE_DHT22) {
#ifdef  M_DHT
				pDev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag before initialization
				pDev->pDevStruct = (void*) DHTInitStruct();
				if (!addInitDevRequest(pDev->nId))
					return pdFAIL; // add request for delayed initialization
#endif
			} else if (pDev->ucType == device_TYPE_BB1BIT_IO_OD) {
				gpio_set_mode(pDev->pGroup->GrpDev.pPort,
						GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN,
						1 << pDev->pGroup->GrpDev.ucPin);
				gpio_set(pDev->pGroup->GrpDev.pPort,
						1 << pDev->pGroup->GrpDev.ucPin); // set high value line

			} else if (pDev->ucType == device_TYPE_BB1BIT_IO_PP) {
				gpio_set_mode(pDev->pGroup->GrpDev.pPort,
						GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
						1 << pDev->pGroup->GrpDev.ucPin);

			} else if (pDev->ucType == device_TYPE_BB1BIT_IO_INPUT) {
				gpio_set_mode(pDev->pGroup->GrpDev.pPort, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, 1 << pDev->pGroup->GrpDev.ucPin);

			} else if (pDev->ucType == device_TYPE_BB1BIT_IO_AO) {
				// to do:

			} else if (pDev->ucType == device_TYPE_PWM) {

				pDev->pDevStruct = pvPortMalloc(sizeof(sPWM_data_t));

			} else if (pDev->ucType == device_TYPE_BB1BIT_IO_AI) {

				pDev->pDevStruct = pvPortMalloc(sizeof(sADC_data_t));
				if (!pDev->pDevStruct) {
					return pdFAIL;
				}

			} else if (pDev->ucType == device_TYPE_GSM) {
#ifdef  M_GSM
				dpDev->pDevStruct = pvPortMalloc(sizeof(sGSMDevice));
				if (!pDev->pDevStruct) {
					return pdFAIL;
				}
				//gsm_preinit_ini ((sGSMDevice*) devArray[all_devs_counter]->pDevStruct, xBaseOutQueue);
				nRes = vMainStartGSMTask((void*)pDev);
				if (!nRes)
				return pdFAIL;
#endif

			} else if (pDev->ucType == device_TYPE_BMP180) {
#ifdef  M_BMP180
				pDev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag before initialization
				pDev->pDevStruct = (void*) bmp180DeviceInitStruct();
				if (!addInitDevRequest(pDev->nId))
					return pdFAIL; // add request for delayed initialization
#endif
			} else if (pDev->ucType == device_TYPE_BME280) {
#ifdef  M_BME280
				pDev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag before initialization
				pDev->pDevStruct = (void*) bme280DeviceInitStruct(pDev);
				if (!addInitDevRequest(pDev->nId))
					return pdFAIL; // add request for delayed initialization
#endif
			}
// ***************** step motor init structure:
#ifdef  M_STEPMOTOR
			else if (pDev->ucType == device_TYPE_STEPMOTOR) {
				pDev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag before initialization
				StepMotorInitStruct(pDev);
				if (!pDev->pDevStruct)
					return pdFAIL;
				if (!addInitDevRequest(pDev->nId))
					return pdFAIL; // add request for delayed initialization
			}
#endif
#ifdef  M_MHZ19
			 else if (pDev->ucType == device_TYPE_MHZ19) {
						pDev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag before initialization
						pDev->pDevStruct = NULL; // not use any structures
						if (!addInitDevRequest(pDev->nId))
							return pdFAIL; // add request for delayed initialization
#endif
					}
		}
// ---- other dev attributes: -----------------------------------------
//---------------------------------------------------------------------
		else if (pDev->ucType == device_TYPE_MEMORY) {
			// memory element pseudo device
			if (strcmp(sName, "e") == 0) {
				// allocate memory for memory elements array:
				if (!MemoryDevInitStruct(pDev, conv2d(sValue)))
					return pdFAIL;
			}
		} else if (pDev->ucType == device_TYPE_PWM) {
			if (strcmp(sName, "ch") == 0) {
				if (!pwm_setup(pDev, sValue)) {
					return pdFAIL;
				}
			} else if (strcmp(sName, "freq") == 0) {
				((sPWM_data_t*) pDev->pDevStruct)->nFreq = conv2d(sValue);
			}

		} else if (pDev->ucType == device_TYPE_BB1BIT_IO_AI) {
			if (strcmp(sName, "ch") == 0) {
				if (!adc_setup(pDev, sValue)) {
					return pdFAIL;
				}
			}
		}
// ***************** DHT22 sensor
		else if (pDev->ucType == device_TYPE_DHT22) {
#ifdef  M_DHT
			// delta humidity
			if (strcmp(sName, "dlh") == 0) {
				if (!pDev->pDevStruct)
					return pdFAIL;
				else {
					((DHT_data_t*) pDev->pDevStruct)->uiDeltaHumidity = conv2d(
							sValue);
				}
			}
#endif
		}		// ***************** MHZ19 sensor
		else if (pDev->ucType == device_TYPE_MHZ19) {
#ifdef  M_MHZ19
			deviceProcAttributes_mhz19(pDev, sName, sValue);
#endif
		}		// ***************** BMP180 sensor
		else if (pDev->ucType == device_TYPE_BMP180) {
#ifdef  M_BMP180
			deviceProcAttributes_bmp180(pDev, sName, sValue);
#endif
		} else if (pDev->ucType == device_TYPE_BME280) {
#ifdef  M_BME280
			deviceProcAttributes_bme280(pDev, sName, sValue);
#endif
		}		// ***************** step motor
		else if (pDev->ucType == device_TYPE_STEPMOTOR) {
#ifdef  M_STEPMOTOR
			deviceProcAttributes_StepMotor(pDev, sName, sValue);

#endif
		} else if (pDev->ucType == device_TYPE_DS18B20) {
#ifdef  M_DS18B20
			if (strcmp(sName, "romid") == 0) {
				pDev->pDevStruct = (void*) ds18b20_init(pDev->pGroup, sValue);
			}
#endif
		} else if (pDev->ucType == device_TYPE_GSM) {
#ifdef  M_GSM
			deviceProcAttributes_gsm(pDev, sName, sValue);
#endif
		}
		return pdPASS;
	} else {
		return pdFAIL;
	}
}

/*
 * Perform device initialization procedure.
 * Usually it long time or complex and cannot be completed at apply preferences time
 */
void deviceInit(sDevice* dev) {
	int32_t nRes = pdPASS;

	if (dev) {
		switch (dev->ucType) {
		case device_TYPE_DS18B20:
			if (dev->pDevStruct) {
// to do...
			}
			break;
		case device_TYPE_DHT22:
			if (dev->pDevStruct) {
				nRes = dht_device_init(dev);
			}
			break;
		case device_TYPE_BMP180:
#ifdef  M_BMP180
			if (dev->pDevStruct) {
				nRes = bmp180DeviceInit(dev);
			}
#endif
			break;
		case device_TYPE_BME280:
#ifdef  M_BME280
			if (dev->pDevStruct) {
				nRes = bme280DeviceInit(dev);
			}
#endif
			break;
		case device_TYPE_MHZ19:
#ifdef  M_MHZ19
				nRes = mhz19DeviceInit(dev);
#endif
			break;
		case device_TYPE_BB1BIT_IO_PP:
		case device_TYPE_BB1BIT_IO_OD:
		case device_TYPE_BB1BIT_IO_INPUT:
// not need
			break;
		case device_TYPE_MEMORY:
			if (dev->pDevStruct) {
// to do ...
			}
			break;
		case device_TYPE_BB1BIT_IO_AI:
			if (dev->pDevStruct) {
// to do...
			}
			break;
		case device_TYPE_STEPMOTOR:
#ifdef  M_STEPMOTOR
			if (dev->pDevStruct) {
				nRes = StepMotorInitHW(dev);
			}
#endif
		}
		if (nRes > 0) {
			dev->nFlag &= 0x7F; // reset device disable flag
		} else {
			dev->nFlag |= DEV_DISABLE_FLAG; // set device disable flag
		}
	}
}

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
//	timer_reset(pGrp->GrpDev.pTimer);
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

uint8_t getAdcDevQty(sDevice* pDev) {
	if (pDev) {
		return pDev->pGroup->iDevQty;
	}
	return 0;
}

int32_t adc_setup(sDevice* pDev, char* psChannels)
{
	int i=0;
	uint32_t adcPortsArray[] = {GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA, GPIOB,GPIOB, GPIOC,GPIOC,GPIOC,GPIOC,GPIOC,GPIOC};
	int32_t nRes = pdPASS;
	sGrpInfo* pGrp = pDev->pGroup;

	pGrp->iDevQty = parseCommaString(psChannels, ((sADC_data_t*)pDev->pDevStruct)->nChannelArray, 16);

	if (pGrp->iDevQty == 0) {
		nRes = pdFAIL;
		goto adc_setup_ret;
	}

	for (i = 0; i < pGrp->iDevQty; i++)
	{
		uint8_t nIndex = ((sADC_data_t*)pDev->pDevStruct)->nChannelArray[i];
		if (nIndex < 16) {
			gpio_set_mode(adcPortsArray[nIndex], GPIO_MODE_INPUT,
					GPIO_CNF_INPUT_ANALOG, 1 << nIndex);
		}
	}

	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
//	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_enable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);


/* We want to read the temperature sensor, so we have to enable it. */
adc_enable_temperature_sensor();
//adc_enable_temperature_sensor(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
//	adc_calibration(ADC1);
	adc_calibrate_async(ADC1);

adc_setup_ret:
	return nRes;
}


// -------------------------------- Device array operations:
sDevice* newDev()
{
	sDevice * pDev = NULL;
	if (++all_devs_counter > mainMAX_ALL_DEVICES) {
		return NULL;
	}
	pDev = (sDevice*)pvPortMalloc(sizeof(sDevice));
//	*(&pDev) = (sDevice*)pvPortMalloc(sizeof(sDevice));
//	**(&devArray + all_devs_counter) = pDev;
	*(sDevice**)(devArray + all_devs_counter) = pDev;
	memset (pDev,0,sizeof(sDevice)); 					// reset all device attributes

	return pDev;
}

sDevice* addDev(sDevice* pnewDev)
{
	sDevice* pDev = newDev();

	memcpy(pDev, pnewDev, sizeof(sDevice));

	return pDev;
}

// Get device by number in array:
sDevice* getDevByNo(uint16_t nDevNo)
{
	if (nDevNo <= all_devs_counter)
		return *(sDevice**)(devArray+nDevNo);
	else
		return NULL;
}

sDevice* getCurrentDev()
{
	return *(sDevice**)(devArray+all_devs_counter);
}


// Get device by ID (from preferences):
sDevice* getDeviceByID (uint16_t nDevID)
{
	uint16_t i;
	sDevice* pDev = NULL;

	for (i = 0; i <= all_devs_counter; i++) {
			pDev = getDevByNo(i);
			if (pDev->nId == nDevID) {
				return pDev;
				break;
			}
	}
	return NULL;
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

// -------------------------------- Group array operations:
sGrpInfo* newGrpInfo()
{
	sGrpInfo * pGrp = NULL;
	if (++grp_counter > mainMAX_DEV_GROUPS) {
		return NULL;
	}
	pGrp = (sGrpInfo*)pvPortMalloc(sizeof(sGrpInfo));
//	*(&pGrp) = ;
	*(sGrpInfo**)(grpArray+grp_counter) = pGrp;

	return pGrp;
}

sGrpInfo* addGrpInfo(sGrpInfo* pnewGrpInfo)
{
	sGrpInfo* pGrpInfo = newGrpInfo();
	memcpy(pGrpInfo, pnewGrpInfo, sizeof(sGrpInfo));

//	pnewGrpInfo->GrpDev = pnewGrpInfo->GrpDev;
//	pnewGrpInfo->iDevQty = pnewGrpInfo->iDevQty;
//	pnewGrpInfo->uiGroup = pnewGrpInfo->uiGroup;
//	pnewGrpInfo->uiObj = pnewGrpInfo->uiObj;

	return pGrpInfo;
}

sGrpInfo* getGrpInfo (uint8_t nGrpID)
{
	uint8_t i;
	sGrpInfo* pGrp = NULL;

	for (i = 0; i <= grp_counter; i++) {
//			if (grpArray[i]->uiGroup == nGrpID) {
		pGrp = *(sGrpInfo**)(grpArray+i);
			if (pGrp->uiGroup == nGrpID) {
				return pGrp;
			}
	}
	return NULL;
}

sGrpInfo* getCurrentGrp()
{
	return *(sGrpInfo**)(grpArray+grp_counter);
}


// -------------------------------- Device values operations:

int32_t getDevValueByID(uint8_t nValCode, uint16_t nDevID)
{
	uint16_t i;
	int32_t res = 0;
	sDevice * pDev;

	for (i = 1; i <= all_devs_counter; i++) {
		pDev = getDevByNo(i);
		if (pDev->nId == nDevID) { res = getDevValue(nValCode, pDev); break;}
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

int32_t getDevData(sDevice* dev, uint8_t nValCode, int32_t* nDevValue,
		uint32_t* nDevLastUpdate) {
	int32_t nValue;
	uint32_t nLastUpdate = 0;
//	int32_t *e;
	int32_t res = pdTRUE;

	if (!dev) {
		res = pdFALSE;
		goto finishGetDevData;;
	}

	nLastUpdate = dev->uiLastUpdate;

	switch (dev->ucType) {
	case device_TYPE_DS18B20:
		if (dev->pDevStruct) {
			nValue = (int32_t) ((ds18b20_device*) dev->pDevStruct)->iDevValue;
			nLastUpdate = ((ds18b20_device*) dev->pDevStruct)->uiLastUpdate;
		}
		break;
	case device_TYPE_DHT22:
		if (dev->pDevStruct) {
// if nValCode==0 return temperature, else - humidity:
			nValue =
				(nValCode == 0) ?
						(int32_t) (((DHT_data_t*) dev->pDevStruct)->temperature) :
						((int32_t) ((DHT_data_t*) dev->pDevStruct)->humidity);
			nLastUpdate = ((DHT_data_t*) dev->pDevStruct)->uiLastUpdate;
		}
		break;
	case device_TYPE_BMP180:
#ifdef  M_BMP180
		if (dev->pDevStruct) {
// if nValCode==0 return temperature, else - pressure:
			nValue =
				(nValCode == 0) ?
						(int32_t) (((BMP180_data_t*) dev->pDevStruct)->iTemperature) :
						((int32_t) ((BMP180_data_t*) dev->pDevStruct)->uiPressure);
			nLastUpdate = ((BMP180_data_t*) dev->pDevStruct)->uiLastUpdate;
		}
#endif
		break;

	case device_TYPE_MHZ19:
#ifdef  M_MHZ19
			nValue = (int32_t) dev->pDevStruct;
			nLastUpdate = dev->uiLastUpdate;
#endif
		break;

	case device_TYPE_BME280:
#ifdef  M_BME280
		if (dev->pDevStruct) {
// if nValCode==0 return temperature, 1 - pressure, 2 - humidity:

				if (nValCode == 0) {
					nValue = ((bme280_t*) dev->pDevStruct)->iTemperature;
					//nPrevVal = ((bme280_t*) dev->pDevStruct)->iPrevTemperature;
				} else if (nValCode == 1) {
					nValue = ((bme280_t*) dev->pDevStruct)->uiPressure;
					//nPrevVal = ((bme280_t*) dev->pDevStruct)->uiPrevPressure;
				} else {
					nValue = ((bme280_t*) dev->pDevStruct)->uiHumidity;
					//nPrevVal = ((bme280_t*) dev->pDevStruct)->uiPrevHumidity
				}
				nLastUpdate = ((bme280_t*) dev->pDevStruct)->uiLastUpdate;
		}
#endif
		break;

	case device_TYPE_STEPMOTOR:
#ifdef  M_STEPMOTOR
		if (dev->pDevStruct) {
// if nValCode == 0 return absolute position,
// 1: return relative position (%)
// 2: return target position,
// 3: return current state:
			switch (nValCode) {
			case 0:
			default:
				nValue = StepMotorGetPosition(dev);
				break;
			case 1:
				nValue = StepMotorGetRelativePosition(dev);
				break;
			case 2:
				nValue = StepMotorGetTargetPosition(dev);
				break;
			case 3:
				nValue = StepMotorGetCurrentState(dev);
				break;
			}
		}
#endif
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

		getMemoryDevValue(dev, nValCode, &nValue);

		break;
	case device_TYPE_BB1BIT_IO_AI:
		if (dev->pDevStruct) {
			nValue = ((sADC_data_t*) dev->pDevStruct)->nADCValueArray[nValCode];
		//nLastUpdate = ((sADC_data_t*)dev->pDevStruct)->uiLastUpdate;
		//nLastUpdate = dev->uiLastUpdate;
		}
		break;
	}

	*nDevValue = nValue;
	*nDevLastUpdate = nLastUpdate;

	finishGetDevData: return res;
}

/* return number available value codes for device type */
uint8_t getNumDevValCodes(uint8_t ucType)
{
	uint8_t n;
	switch (ucType) {
	case device_TYPE_DHT22:
	case device_TYPE_BMP180:
		n = 2;
		break;
	case device_TYPE_DS18B20:
	case device_TYPE_BB1BIT_IO_PP:
	case device_TYPE_BB1BIT_IO_OD:
	case device_TYPE_BB1BIT_IO_INPUT:
	case device_TYPE_MHZ19:
		n = 1;
		break;
	case device_TYPE_BME280:
		n = 3;
		break;
	case device_TYPE_STEPMOTOR:
		n = 4;
		break;
	default:
		n = 0;
	}
	return n;
}

/* return number available value codes by device*/
uint8_t getNumberDevValues(sDevice* pDev) {
	uint8_t nNumValTypes = 0;
	if (pDev) {
		if (pDev->ucType == device_TYPE_MEMORY) {
			nNumValTypes = getMemoryDevQty(pDev);
		} else
			if (pDev->ucType == device_TYPE_BB1BIT_IO_AI) {
			nNumValTypes = getAdcDevQty(pDev);
		}
		else {
			nNumValTypes = getNumDevValCodes(pDev->ucType);
		}
	}
	return nNumValTypes;
}

void setDevValueByID(int32_t nValue, uint8_t nDevCmd, uint16_t nDevID, uint8_t nDataType)
{
	sDevice * pDev;

// if DevID == 0 then search action with ActID == nValue, else search device:

	if (nDevID == 0) {
		vMainStartTimer(getActionByID(nValue));

//		for (i=0; i<act_counter;i++) {
//			if (actArray[i]->nActId == nValue) {
//				vMainStartTimer(actArray[i]);
//			}
//		}
	} else {
		pDev = getDeviceByID(nDevID);
//		for (i = 1; i < all_devs_counter; i++) {
//			pDev = getDevByNo(i);
//			if (pDev->nId == nDevID) {
//				if (nDataType == eElmDevValue) {
//					nValue = getDevValueByID(nDevCmd, (uint16_t) nValue); // for eElmDevValue datatype nValue = devID
//				} else {
//					setDevValue(nValue, nDevCmd, pDev, nDataType);
//				}
//				break;
//			}
//		}
		// if local device array not contain this device, send to input queue for routing
		if (pDev) {
			setDevValue(nValue, nDevCmd, pDev, nDataType);
		} else
		{
//			char msg[mainMAX_MSG_LEN];
			char *msg = pvPortMalloc(mainMAX_MSG_LEN);
			if (nDataType == eElmString) {
				xprintfMsgStr(msg, "{\"ssn\":{\"v\":1,\"obj\":0,\"cmd\":\"sdv\", \"data\": {\"adev\":%d,\"acmd\":%d,\"aval\":\"%s\"}}}", nDevID, nDevCmd, (char*)nValue);
				vPortFree((void*)nValue);
			} else {
				xprintfMsgStr(msg, "{\"ssn\":{\"v\":1,\"obj\":0,\"cmd\":\"sdv\", \"data\": {\"adev\":%d,\"acmd\":%d,\"aval\":\"%d\"}}}", nDevID, nDevCmd, nValue);
			}
			char* buf = pvPortMalloc(strlen(msg));
			if (buf)
			{
				memcpy(buf, &msg, strlen(msg) + 1);
				vSendInputMessage(1, 0, mainJSON_MESSAGE, 0, 0, nDevID, (void*) buf, strlen(buf), 0);
			}
			vPortFree(msg);
		}
	}
}

void setDevValue(int32_t nValue, uint8_t nDevCmd, sDevice* dev, uint8_t nDataType)
{
//	int32_t *e;
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
		setMemoryDevValue(dev, nDevCmd, nValue);
		uiLastSaveMemoryTick = rtc_get_counter_val();
		break;
	case device_TYPE_PWM:
		if (dev->pDevStruct) {
			timer_set_oc_value(dev->pGroup->GrpDev.pTimer, getTIM_OC(nDevCmd), nValue); // channel = nDevCmd, % = nValue
		}
		break;
	case device_TYPE_GSM:
#ifdef  M_GSM
		setGSMDevValue(nValue, nDevCmd, dev, nDataType);
#endif
		break;
// *********************  Step Motor:
// cmd = 0 - set position (absolutely value in steps)
// cmd = 1 - set position in percents
// cmd = 2 - perform calibrate
// cmd = 3 - make step (if value > 0 than forward, else backward)
	case device_TYPE_STEPMOTOR:
		// check enable device flag
		if (!(dev->nFlag & 0x80)) {
			switch (nDevCmd) {
				case 0:
					StepMotorSetTargetPosition(dev, nValue);
					break;
				case 1:
					StepMotorSetTargetPositionPercents(dev, nValue);
					break;
				case 2:
					StepMotorCalibrate(dev);
					break;
				case 3:
					StepMotorOneStep(dev, nValue);
					break;
			}

		}
		break;
	}
	// log device value changing for local device:
//	logAction(0, dev->nId, nDevCmd, nValue);

}

// Scan thru all devices and store memory devices to persistent store:
int32_t storeAllMemDevs()
{
	sDevice* pDev;
	void *pBufDev = NULL;
	void *pBufAll = NULL;
	void *pBufTmp;
	uint16_t nBufDevSize;
	uint16_t nBufAllSize = 0;
	uint16_t nNumMemDevs = 0;
	int32_t nRes = pdPASS;

	for (uint16_t j = 1; j <= all_devs_counter; j++) {
		pDev = getDevByNo(j);
		if (pDev) {
			if (pDev->ucType == device_TYPE_MEMORY) {
				nNumMemDevs++;
				nRes = serializeMemDevs(pDev, &pBufDev, &nBufDevSize);
				if (nRes) {
					pBufTmp = pvPortMalloc(nBufAllSize + nBufDevSize);
					if (pBufTmp) {
						if (pBufAll) {
							memcpy(pBufTmp,pBufAll,nBufAllSize);
							vPortFree(pBufAll);
						}
						memcpy(pBufTmp+nBufAllSize,pBufDev,nBufDevSize);
						vPortFree(pBufDev);
						pBufAll = pBufTmp;
						nBufAllSize+=nBufDevSize;
					} else {
						nRes = pdFAIL;
					}
				} else {
					nRes = pdFAIL;
				}
			}
		}
	}

	if (nRes && nNumMemDevs) {
		// allocate memory for all device data and metadata:
		/* Full memDev data structure:

		   NUMDEVS  DEV1  e1  Val_1    Val_2    Val_n    DEV2  e2   Val_1     Val_2     CRC    Addr
		   -------|------|--|--------|--------|--------|------|--|---------|----------|------|-------|
		      2b    2b    2b   4b        4b       4b      2b    2b    4b       4b       2b     2b
		 */
		pBufTmp = pvPortMalloc(nBufAllSize+6);
		if (pBufTmp) {
			memcpy(pBufTmp+2,pBufAll,nBufAllSize);
			*(uint16_t*)(pBufTmp)=nNumMemDevs;
			vPortFree(pBufAll);
			// CRC and EEPROM Addr calculate in storeMemDevs:
			if (storeMemDevs(pBufTmp, nBufAllSize+6)) {
				uiLastSaveMemoryTick = rtc_get_counter_val();
				vPortFree(pBufTmp);
			} else {
				nRes = pdFAIL;
			}
		}
	}
	return nRes;
}

// Scan thru all devices and restore memory devices from persistent store:
int32_t restoreAllMemDevs()
{
	sDevice* pDev;
	for (uint16_t j = 1; j <= all_devs_counter; j++) {
		pDev = getDevByNo(j);
		if (pDev) {
			if (pDev->ucType == device_TYPE_MEMORY)
				return restoreMemDevs(pDev);
		}
	}
	return pdFAIL;
}

void	clearActionsDeviceCash (sDevice* pDev)
{
	if (pDev) {
		if (pDev->nActionsCashSize > 0)
		{
			vPortFree(pDev->pActionsCash);
			pDev->pActionsCash = NULL;
		}
	}
}


/* clear old cash info, reallocate memory and fill new arrays */
int32_t refreshActions2DeviceCash() {
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

	for (nDevIndex = 0; nDevIndex <= all_devs_counter; nDevIndex++) {
		pDev = getDevByNo(nDevIndex);
		if (pDev) {
			clearActionsDeviceCash(pDev);
			nTmpArrayIndex = 0;
			// scan all actions for this device
			for (nActIndex = 1; nActIndex <= act_counter; nActIndex++) {
//				pAct = actArray[nActIndex];
				pAct = getActionByNo(nActIndex);
				if (pAct) {
					pEvtElm = pAct->pStartElmEvents;
					do {
						if (pEvtElm) {
							if (pEvtElm->nElmType == eElmInterval) {
								// skip action if interval type
								break;
							}
							if ((pEvtElm->nElmType == eElmDevValue)
									&& (pEvtElm->nElmData1 == pDev->nId)) {
								// minimum once use this device in action -> add to array and break scan thru other elements
								pTmpActArray[nTmpArrayIndex++] = pAct;
								break;
							}
							// search time events for abstract device[0]
							if (pDev->nId == 0) {
								switch (pEvtElm->nElmType) {
								case eElmTimeValue:
								case eElmDateYear:
								case eElmDateMonth:
								case eElmDateDay:
								case eElmDateDayWeek: {
									pTmpActArray[nTmpArrayIndex++] = pAct;
									break;
								}
								}
							}
							// scan left to right:
							//      check cyclic loop:
							if (pEvtElm == pEvtElm->pPrevElm) {
								pEvtElm = NULL;
							} else {
								pEvtElm = pEvtElm->pPrevElm;
							}
						}
					} while (pEvtElm);
				}
			}
			// if find any action for this device, than add it to cash
			if (nTmpArrayIndex > 0) {
				pDev->pActionsCash = pvPortMalloc(
						nTmpArrayIndex * sizeof(sAction*));
				pDev->nActionsCashSize = nTmpArrayIndex;
				for (i = 0; i < nTmpArrayIndex; i++) {
					((sAction**) pDev->pActionsCash)[i] = pTmpActArray[i];
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

