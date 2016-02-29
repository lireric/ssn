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
#include "stack.h"
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
				vSendInputMessage(1, 0, mainJSON_MESSAGE, 0, nDevID, (void*) buf, strlen(buf), 0);
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
}

sAction* getActionByID (uint16_t nActId)
{
	sAction* pAct = 0;
	uint16_t i;
	for (i=0; i<act_counter; i++) {
		if (actArray[i]->nActId == nActId) {
			pAct = actArray[i];
			break;
		}
	}
	return pAct;
}

// ------------------------------------------------------------------
// convert string formula action description into Reverse Polish Notation (RPN)

sEvtElm* 	elmPush (sEvtElm* headEvtElm, sEvtElm* newEvtElm);
sEvtElm* 	elmPop (sEvtElm* headEvtElm);
uint8_t 	getPriority(uint8_t nOper);

/* function push new element record on stack. Return pointer into new stack head */
sEvtElm* 	elmPush(sEvtElm* headEvtElm, sEvtElm* newEvtElm)
{
	sEvtElm* pEvtElm;

	if((pEvtElm=pvPortMalloc(sizeof(sEvtElm))) == pdFALSE)
	{
		return NULL;
	}
	pEvtElm->nElmData1 	= newEvtElm->nElmData1;
	pEvtElm->nElmData2 	= newEvtElm->nElmData2;
	pEvtElm->nElmType	= newEvtElm->nElmType;
	pEvtElm->pNextElm	= headEvtElm;
	pEvtElm->pPrevElm	= NULL;
	if (headEvtElm) {
		headEvtElm->pPrevElm = pEvtElm;
	}

/* pEvtElm - new stack head */
  return pEvtElm;
}


sEvtElm* elmPopBase(sEvtElm* headEvtElm, uint8_t direction)
{
  sEvtElm* xNextEvtElm;
  sEvtElm* xPrevEvtElm;
  sActElm* pActElm;

  if(headEvtElm == NULL) return NULL;

  xNextEvtElm = headEvtElm->pNextElm;
  xPrevEvtElm = headEvtElm->pPrevElm;
  if (xPrevEvtElm) {
	  if (xNextEvtElm) {
		  xPrevEvtElm->pNextElm = xNextEvtElm->pPrevElm;
	  } else {
		  xPrevEvtElm->pNextElm = NULL;
	  }
  }
  if (xNextEvtElm) {
	  if (xPrevEvtElm) {
		  xNextEvtElm->pPrevElm = xPrevEvtElm->pNextElm;
	  } else {
		  xNextEvtElm->pPrevElm = NULL;
	  }
  }

  if (headEvtElm->nElmType == eElmAction) {
	  pActElm = (sActElm*)headEvtElm->nElmData1;
	  if (pActElm) {
		  if (pActElm->nElmDataType == eElmString) {
			  vPortFree((void*)pActElm->nElmDataIn);
			  vPortFree((void*)pActElm->nElmDataOut);
		  }
		  vPortFree(pActElm);
	  }
  }
  vPortFree(headEvtElm);

  if (direction == 0)
  {
	  return xNextEvtElm;
  } else {
	  return xPrevEvtElm;
  }
}

/*
 * elmPop get element from stack (placed on stack, pointed headEvtElm) end remove it from stack.
 * Return element, closed to End of stack
*/
sEvtElm* elmPop(sEvtElm* headEvtElm)
{
	return elmPopBase(headEvtElm, 0);
}
/*
 * elmPop get element from stack (placed on stack, pointed headEvtElm) end remove it from stack.
 * Return element, closed to Begin of stack
*/
sEvtElm* elmPopRight(sEvtElm* headEvtElm)
{
	return elmPopBase(headEvtElm, 1);
}

/* getPriority return operation priority */
uint8_t getPriority(uint8_t nOper)
{
  switch(nOper)
  {
    case eOpMul:
    case eOpDiv:
         return 4;

    case eOpPlus:
    case eOpMinus:
         return 3;

    case eOpEq:
    case eOpL:
    case eOpLE:
    case eOpG:
    case eOpGE:
    case eOpAND:
    case eOpOR:
    case eOpNOT:
         return 2;

    case eOpSO:
         return 1;

    case eOpSC:
         return 0;
  }

  return 0;

}


typedef enum
{
	eCurStateInit = 0,			/* init state */
	eCurStateParseError,		/* fatal parsing error */
	eCurStateArgumentError,
	eCurState_Resume,			/* operands after '=' (can be only 'a' - actions) */
	eCurState_Oper_begin,		/* begin process operand <, <=, == etc */
	eCurState_A_begin,			/* begin process 'a' operand */
	eCurState_A_Arg1_Process,	/* process argument 1 of 'a' operand */
	eCurState_A_Arg2_Process,	/* process argument 2 of 'a' operand */
	eCurState_A_Arg3_Process,	/* process argument 3 of 'a' operand */
	eCurState_A_Arg3S_Process,	/* process argument 3 as string of 'a' operand "ex: ..=a(action_dev_ID,action_dev_channel,_d(x,y)_)" */
	eCurState_A_Arg3D_Process,	/* process argument 3 as get device value of 'a' operand */
	eCurState_A_Arg4x_Process,	/* preprocess argument 4 of 'a' operand: wait ',' or ')' if only 3 arguments */
	eCurState_A_Arg4_Process,	/* process argument 4 of 'a' operand */
	eCurState_A_Arg5_Process,	/* process argument 5 of 'a' operand */
	eCurState_A_Arg5S_Process,	/* process argument 5 as string of 'a' operand */
	eCurState_A_Arg6x_Process,	/* preprocess finish 'a' operand: wait ')' */
	eCurState_A_Push,			/* push 'a' operand to elm stack */
	eCurState_T_begin,			/* begin process 't' operand */
	eCurState_D_begin,			/* begin process 'd' operand */
	eCurState_D_Arg1_Process,	/* process argument 1 of 'd' operand */
	eCurState_D_Arg2_Process,	/* process argument 2 of 'd' operand */
	eCurState_I_begin,			/* begin process 'i' operand */
	eCurState_I_Arg1_Process,	/* process argument 1 of 'i' operand */
	eCurState_I_Arg2_Process,	/* process argument 1 of 'i' operand */
	eCurState_Const_Str_Process_h, /*  process string constant - convert it to seconds from 00:00:00 */
	eCurState_Const_Str_Process_m, /*  process string constant - convert it to seconds from 00:00:00 */
	eCurState_Const_Str_Process_s, /*  process string constant - convert it to seconds from 00:00:00 */
	eCurState_Const_Str_Process_Finish, /* store converted string constant */
	eCurState_Const_Process,		/* process constant */
	eCurState_A3D_Arg1_Process,	/* process argument 1 device in action operand */
	eCurState_A3D_Arg2_Process	/* process argument 2 device in action operand */
} eElmParseStatuses;

sEvtElm* parseActionString(char* pAStr)
{
//	int32_t res = pdFALSE;
	int32_t 	nCurrentState = eCurStateInit;
	sEvtElm 	xEvtElm;
	sActElm 	xActElm;
	sActElm* 	pActElm;
	sEvtElm* 	pEvtElmHead = 0;
	char 		c;
	char 		c1 = 0;
	char 		strBuf[devMAX_MSG_LEN];
	uint16_t	k;
	uint16_t 	nStrBuf = 0;
	uint16_t 	nStrLen = 0;
	uint32_t 	d = 0;
	uint32_t 	arg1 = 0;
	uint32_t 	arg2 = 0;

	xActElm.nDevId = 0;
	xActElm.nActCmdOut = 0;
	xActElm.nElmDataOut = 0;
	xActElm.nElmDataIn = 0;
	xActElm.nElmDataType = 0;
	xActElm.nActCmdIn = 0;
	xActElm.nActFlag = 0;
	xActElm.ngetDevCmdIn = 0;

	if (!pAStr) return NULL;

	nStrLen = strlen(pAStr);

	/* scan full input string
	 * after executing sEvtElm stack will be build for next processing
	 * */
	    for(k = 0; k < nStrLen; k++)
	    {
	    	c = pAStr[k];
	    	if (c == ' ') continue;

beginSwitch:
	    	switch(nCurrentState)
	    	{
				case eCurState_Resume:
				{
					switch(c)
					{
    		   	    	case 'a':
    		   	    	{
    						nCurrentState = eCurState_A_begin;
    						break;
    		   	    	}
    		   	    	case ',':
    		   	    	{
	    		   	    	break;
    		   	    	}
					}
					// to do: process error
					break;
				}



	    	    case eCurState_A3D_Arg1_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint32_t )conv2d(strBuf);
	    	    			xActElm.nElmDataIn = arg1; // in nElmDataIn store get device_ID
		    	    		d = 0;
	    	    			nCurrentState = eCurState_A3D_Arg2_Process;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_A3D_Arg2_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ')')
	    	    		{
	    	    			if (d > 6) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg2 = (uint32_t )conv2d(strBuf);

	    	    			strBuf[d] = 0;
	    	    			xActElm.ngetDevCmdIn = arg2; // store get dev value command
	    	    			xActElm.nElmDataType = eElmGetDevValue;
		    	    		d = 0;
		    	    		xActElm.nActCmdOut = 0;
	    	    			nCurrentState = eCurState_A_Arg4x_Process; // continue processing
	    	    			xActElm.nActFlag ^= devEXEC_ACTION_FLAG_OUT_ARG;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }


	    	    case eCurState_A_begin:
	    	    {
	    	    	if (c == '(')
	    	    	{
	    	    		d = 0;
	    	    		xActElm.nActFlag ^= devEXEC_ACTION_FLAG_OUT_ARG;
	    	    		xActElm.nActCmdOut = 0;
	    	    		xActElm.nElmDataOut = 0;
	    	    		nCurrentState = eCurState_A_Arg1_Process;
	    	    	}
	    	    	else
	    	    		nCurrentState = eCurStateParseError;
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg1_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 6) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nDevId = (uint16_t )conv2d(strBuf);
		    	    		d = 0;
	    	    			nCurrentState = eCurState_A_Arg2_Process;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg2_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 3) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nActCmdIn = (uint8_t )conv2d(strBuf);
		    	    		d = 0;
	    	    			nCurrentState = eCurState_A_Arg3_Process;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg3_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nElmDataIn = (uint32_t)conv2d(strBuf);
	    	    			xActElm.nElmDataType = eElmInteger;
		    	    		d = 0;
		    	    		xActElm.nActCmdOut = 0;
		    	    		xActElm.nActFlag |= devEXEC_ACTION_FLAG_OUT_ARG;
	    	    			nCurrentState = eCurState_A_Arg4_Process;
	    	    		} else
	   	    			if (c == '\'')
	   	    			{
	   	    				nStrBuf = 0;
		    	    		xActElm.nActCmdOut = 0;
	    	    			nCurrentState = eCurState_A_Arg3S_Process;
	   	    			} else
		   	    			if (c == 'd')
		   	    			{
		   	    				// process get device value in action:
		   	    				nStrBuf = 0;
			    	    		xActElm.nActCmdOut = 0;
		    	    			nCurrentState = eCurState_A_Arg3D_Process;
		   	    		} else
	    	    		if (c == ')') {
	    	    			// action has only 3 arguments - push it to elm stack
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nElmDataIn = (uint32_t)conv2d(strBuf);
	    	    			xActElm.nElmDataType = eElmInteger;
		    	    		d = 0;
	    	    			nCurrentState = eCurState_A_Push;
	    	    			goto beginSwitch;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    			goto beginSwitch;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg3D_Process:
	    	    {
  	    	    	// process get device value in action:
	    	    	if (c == '(')
	    	    	{
	    	    		d = 0;
	    	    		nCurrentState = eCurState_A3D_Arg1_Process;
	    	    	}
	    	    	else
	    	    		nCurrentState = eCurStateParseError;
	    	    	break;
	    	    }


	    	    case eCurState_A_Arg3S_Process:
	    	    {
	    	    	if (c != '\'')
	    	    	{
	    	    		strBuf[nStrBuf++] = c;
	    	    		if (nStrBuf == devMAX_MSG_LEN) {
	    	    			nCurrentState = eCurStateArgumentError;
	    	    			break;
	    	    		}
	    	    	} else {
	    	    		strBuf[nStrBuf++] = 0;
	    	    		xActElm.nElmDataIn = (uint32_t)pvPortMalloc(strlen(strBuf));
	    	    		if (!xActElm.nElmDataIn) {
    	    				nCurrentState = eCurStateParseError;
	    	    		} else {
	    	    			memcpy((void*)xActElm.nElmDataIn, &strBuf, strlen(strBuf));
	    	    			xActElm.nElmDataType = eElmString;
	    	    			nCurrentState = eCurState_A_Arg4x_Process;
	    	    		}
    	    			break;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg4x_Process:
	    	    {
	    	    	if (c == ',') {
	    	    		d = 0;
    	    			nCurrentState = eCurState_A_Arg4_Process;
    	    			xActElm.nActFlag |= devEXEC_ACTION_FLAG_OUT_ARG;
	    	    	} else
		    	    	if (c == ')') {
		    	    		// action has only 3 arguments - push it to elm stack
	    	    			nCurrentState = eCurState_A_Push;
	    	    			goto beginSwitch;
	    	    	}
	    			break;
	    	    }
	    	    case eCurState_A_Arg4_Process:
	    	    {
	    	    	//xActElm.nActFlag |= devEXEC_ACTION_FLAG_OUT_ARG;
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 3) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nActCmdOut = (uint8_t )conv2d(strBuf);
		    	    		d = 0;
	    	    			nCurrentState = eCurState_A_Arg5_Process;
	    	    		}
	    	    			else {
	    	    				nCurrentState = eCurStateParseError;
	    	    			}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg5_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ')')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			xActElm.nElmDataOut = (uint32_t )conv2d(strBuf);
	    	    			xActElm.nElmDataType = eElmInteger;
	    	    			nCurrentState = eCurState_A_Push;
	    	    			goto beginSwitch;
	    	    		} else
	    	    			if (c == '\'')
	    	    			{
	    	    				nStrBuf = 0;
		    	    			nCurrentState = eCurState_A_Arg5S_Process;
	    	    			} else {
	    	    				nCurrentState = eCurStateParseError;
	    	    			}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg5S_Process:
	    	    {
	    	    	if (c != '\'')
	    	    	{
	    	    		strBuf[nStrBuf++] = c;
	    	    		if (nStrBuf == devMAX_MSG_LEN) {
	    	    			nCurrentState = eCurStateArgumentError;
	    	    			break;
	    	    		}
	    	    	} else {
	    	    		strBuf[nStrBuf++] = 0;
	    	    		xActElm.nElmDataOut = (uint32_t)pvPortMalloc(strlen(strBuf));
	    	    		if (!xActElm.nElmDataOut) {
    	    				nCurrentState = eCurStateParseError;
	    	    		} else {
	    	    			memcpy((void*)xActElm.nElmDataOut, &strBuf, strlen(strBuf));
	    	    			xActElm.nElmDataType = eElmString;
	    	    			nCurrentState = eCurState_A_Arg6x_Process;
	    	    		}
    	    			break;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_A_Arg6x_Process:
	    	    {
		    	    	if (c == ')') {
		    	    		// push operand to elm stack
	    	    			nCurrentState = eCurState_A_Push;
		    	    	} else {
    	    				nCurrentState = eCurStateParseError;
		    	    	}
		    	    	goto beginSwitch;;
	    	    }
	    	    case eCurState_A_Push:
	    	    {
	    	    	pActElm = (sActElm*)pvPortMalloc(sizeof(sActElm));
	    	    	if (pActElm) {
	    	    		pActElm->nActCmdIn 	= xActElm.nActCmdIn;
	    	    		pActElm->nActCmdOut = xActElm.nActCmdOut;
	    	    		pActElm->nDevId 	= xActElm.nDevId;
	    	    		pActElm->nElmDataIn = xActElm.nElmDataIn;
	    	    		pActElm->nElmDataOut  = xActElm.nElmDataOut;
	    	    		pActElm->nElmDataType = xActElm.nElmDataType;
	    	    		pActElm->nActFlag	= xActElm.nActFlag;
	    	    		pActElm->ngetDevCmdIn = xActElm.ngetDevCmdIn;

	    	    		xEvtElm.nElmData1 = (uint32_t)pActElm;
	    	    		xEvtElm.nElmType = eElmAction;
	    	    		pEvtElmHead = elmPush(pEvtElmHead, &xEvtElm);
	    	    		nCurrentState = eCurState_Resume;
	    	    	} else {
	    	    		nCurrentState = eCurStateParseError;
	    	    	}
	    			break;
	    	    }

    			case eCurState_T_begin:
    			{
    				  nCurrentState = eCurStateInit;
	    		   	  switch(c)
	    		   	  {
	    		   	    case 't':
	    	    			xEvtElm.nElmType = eElmTimeValue;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case 'y':
	    	    			xEvtElm.nElmType = eElmDateYear;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case 'm':
	    	    			xEvtElm.nElmType = eElmDateMonth;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case 'd':
	    	    			xEvtElm.nElmType = eElmDateDay;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case 'w':
	    	    			xEvtElm.nElmType = eElmDateDayWeek;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    default:
	    		   	    {
	    					nCurrentState = eCurStateParseError;
    		   	    		break;
	    		   	    }
	    		   	  }
	    			break;
    			}
	    		case eCurState_Oper_begin:
	    		{
	    			if (c == '=') {
		    		   	  switch(c1)
		    		   	  {
		    		   	    case '<':
		    	    			xEvtElm.nElmData1 = eOpLE;
		    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
		    		   	    	break;
		    		   	    case '>':
		    	    			xEvtElm.nElmData1 = eOpGE;
		    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
		    		   	    	break;
		    		   	    case '=':
		    	    			xEvtElm.nElmData1 = eOpEq;
		    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
		    		   	    	break;
		    		   	  }
	    				nCurrentState = eCurStateInit;
		    			break;
	    			} else
	    			if (c == '&') {
	    				if (c1 == '&')
	    				{
	    					xEvtElm.nElmData1 = eOpAND;
    	    				pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    					nCurrentState = eCurStateInit;
    		   	    		break;
	    				} else {
	    					nCurrentState = eCurStateParseError;
    		   	    		break;
	    				}
	    			} else
	    			if (c == '|') {
	    				if (c1 == '|')
	    				{
	    					xEvtElm.nElmData1 = eOpOR;
	    					pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    					nCurrentState = eCurStateInit;
	    					break;
	    				} else {
	    					nCurrentState = eCurStateParseError;
    		   	    		break;
	    				}
	    			}

	    		   	switch(c1)
	    		   	{
	    		   	    case '<':
	    	    			xEvtElm.nElmData1 = eOpL;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '>':
	    	    			xEvtElm.nElmData1 = eOpG;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '=':
	    	    			xEvtElm.nElmData1 = eOpResume;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	        				nCurrentState = eCurState_Resume;
	        				goto beginSwitch;
	    		   	}
    				// now let's continue process this char as eCurStateInit state once more
    				nCurrentState = eCurStateInit;
    				goto beginSwitch;
	    		}
	    	    case eCurState_Const_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else {
    	    			if (d > 12) {
    	    				nCurrentState = eCurStateArgumentError;
    	    				break;
    	    			}
    	    			strBuf[d] = 0;
    	    			arg1 = (uint32_t )conv2d(strBuf);

    	    			nCurrentState = eCurStateInit;
    	    			xEvtElm.nElmData1 = arg1;
    	    			xEvtElm.nElmData2 = 0;
    	    			xEvtElm.nElmType = eElmConst;
    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
        				// now let's continue process this char as eCurStateInit state once more
        				goto beginSwitch;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_Const_Str_Process_h:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ':') {
	    	    			if (d > 3) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint8_t )conv2d(strBuf);
	    	    			arg2 = arg1 * 3600; // arg2 accumulate temporary result
		    				d = 0;
	    	    			nCurrentState = eCurState_Const_Str_Process_m;
	    	    	} else {
    					nCurrentState = eCurStateArgumentError;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_Const_Str_Process_m:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ':') {
	    	    			if (d > 3) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint8_t )conv2d(strBuf);
	    	    			arg2 += arg1 * 60; // arg2 accumulate temporary result
		    				d = 0;
	    	    			nCurrentState = eCurState_Const_Str_Process_s;
	    	    	} else {
    					nCurrentState = eCurStateArgumentError;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_Const_Str_Process_s:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == '\'') {
	    	    			if (d > 3) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint8_t )conv2d(strBuf);
	    	    			arg2 += arg1; // arg2 accumulate temporary result
	    	    			arg1 = arg2;
	    	    			nCurrentState = eCurState_Const_Str_Process_Finish;
	    	    	} else {
    					nCurrentState = eCurStateArgumentError;
	    	    	}
	    	    	break;
	    	    }
	    	    case eCurState_Const_Str_Process_Finish:
	    	    {
	    			xEvtElm.nElmData1 = arg1;
	    			xEvtElm.nElmData2 = 0;
	    			xEvtElm.nElmType = eElmConst;
	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    			nCurrentState = eCurStateInit;
	    			break;
	    	    }
	    	    case eCurStateInit:
	    	    {
	    	    	  if (c >= '0' && c <= '9') {
	    	    		strBuf[0] = c;
	    				d = 1;
	    				nCurrentState = eCurState_Const_Process;
	    				break;
	    	    	  }
	    		   	  switch(c)
	    		   	  {
	    		   	    case '\'':
	    		   	    	arg1 = 0;
		    				d = 1;
	    		   	    	nCurrentState = eCurState_Const_Str_Process_h;
	    		   	    	break;
	    		   	    case 't':
//	    		   	    	c1 = c;
	    		   	    	nCurrentState = eCurState_T_begin;
	    		   	    	break;
	    		   	    case 'd':
	    		   	    	nCurrentState = eCurState_D_begin;
	    		   	    	break;
	    		   	    case 'i':
	    		   	    	nCurrentState = eCurState_I_begin;
	    		   	    	break;
	    		   	    case '<':
	    		   	    case '>':
	    		   	    case '=':
	    		   	    case '&':
	    		   	    case '|':
	    		   	    	c1 = c;
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    		   	    	nCurrentState = eCurState_Oper_begin;
	    		   	    	break;
	    		   	    case '(':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpSO;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case ')':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpSC;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '+':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpPlus;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '-':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpMinus;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '*':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpMul;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '/':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpDiv;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;
	    		   	    case '~':
	    		   	    	xEvtElm.nElmType = eElmOperation;
	    	    			xEvtElm.nElmData1 = eOpNOT;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    		   	    	break;

	    		   	  }
	    		   	  break;
	    	    }
	    	    case eCurState_D_begin:
	    	    {
	    	    	if (c == '(')
	    	    	{
	    	    		d = 0;
	    	    		nCurrentState = eCurState_D_Arg1_Process;
	    	    	}
	    	    	else
	    	    		nCurrentState = eCurStateParseError;
	    	    	break;
	    	    }
	    	    case eCurState_I_begin:
	    	    {
	    	    	if (c == '(')
	    	    	{
	    	    		d = 0;
	    	    		nCurrentState = eCurState_I_Arg1_Process;
	    	    	}
	    	    	else
	    	    		nCurrentState = eCurStateParseError;
	    	    	break;
	    	    }
	    	    case eCurState_I_Arg1_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint32_t )conv2d(strBuf);
		    	    		d = 0;
	    	    			nCurrentState = eCurState_I_Arg2_Process;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_I_Arg2_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ')')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg2 = (uint32_t )conv2d(strBuf);

	    	    			nCurrentState = eCurStateInit;
	    	    			xEvtElm.nElmData1 = arg1;
	    	    			xEvtElm.nElmData2 = (uint16_t)arg2;
	    	    			xEvtElm.nElmType = eElmInterval;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_D_Arg1_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ',')
	    	    		{
	    	    			if (d > 12) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg1 = (uint32_t )conv2d(strBuf);
		    	    		d = 0;
	    	    			nCurrentState = eCurState_D_Arg2_Process;
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	    case eCurState_D_Arg2_Process:
	    	    {
	    	    	if (c >= '0' && c <= '9') {
	    	    		strBuf[d++] = c;
	    	    	} else
	    	    		if (c == ')')
	    	    		{
	    	    			if (d > 6) {
	    	    				nCurrentState = eCurStateArgumentError;
	    	    				break;
	    	    			}
	    	    			strBuf[d] = 0;
	    	    			arg2 = (uint32_t )conv2d(strBuf);

	    	    			nCurrentState = eCurStateInit;
	    	    			xEvtElm.nElmData1 = arg1;
	    	    			xEvtElm.nElmData2 = (uint16_t)arg2;
	    	    			xEvtElm.nElmType = eElmDevValue;
	    	    			pEvtElmHead = elmPush (pEvtElmHead, &xEvtElm);
	    	    		}
	    	    		else {
	    	    			nCurrentState = eCurStateParseError;
	    	    		}
	    	    	break;
	    	    }
	    	}

	    	if ((nCurrentState == eCurStateParseError) || (nCurrentState == eCurStateArgumentError)) {
	    		// free resources and return with error
	    		while (pEvtElmHead) {
	    			if (pEvtElmHead->nElmType == eElmAction) {
	    				vPortFree((void*)pEvtElmHead->nElmData1);
	    			}
	    			pEvtElmHead = elmPop(pEvtElmHead);
	    		}
	    		goto endParseAction;
	    	}
	    }

endParseAction:
	return pEvtElmHead;
}

/*
 * set new or update exist Action, based on string formula description
 * return pdTRUE if success
 */
int32_t	setAction (uint16_t nActId, char* pAStr, uint32_t arep, uint16_t nFlags)
{
	int32_t res = pdFALSE;
	sEvtElm*	pEvtElm;
	sEvtElm*	pFirstActionEvtElm;	// first action element (after '=')
	sEvtElm*	pFirstEvtElm;		// begin of actions formula
	sEvtElm*	pOperationsEvtElm;	// operations stack
	sActElm* 	pActElm;
	int8_t		nState; // process state: 0 - before '=', 1 - after '='
	uint8_t		nSizeElm;
	sCommonStack xOutStack;
	sCommonStack xOpersStack;

	if (!pAStr) return pdFALSE;
	if (strlen(pAStr) == 0) return pdFALSE;

	sAction* pAct = getActionByID(nActId);

	// if perform exist action updating then prior free resources
	if (pAct) {
		if (pAct->xActTimer) {
			xTimerDelete(pAct->xActTimer, 0);
		}
		if (pAct->xActTimerOneShot) {
			xTimerDelete(pAct->xActTimerOneShot, 0);
		}
		// free action elements resources
		pEvtElm = pAct->pStartElmActions;
			while (pEvtElm) {
				if (pEvtElm->nElmType == eElmAction) {
					vPortFree((void*)pEvtElm->nElmData1);
				}
				pEvtElm = elmPop(pEvtElm);
			}
			pEvtElm = pAct->pStartElmEvents;
				while (pEvtElm) {
					pEvtElm = elmPop(pEvtElm);
				}


	} else {
		if (act_counter < mainMAX_ACTIONS)
			{
				pAct = pvPortMalloc(sizeof(sAction));
				if (pAct) {
					actArray[act_counter++] = pAct;
					pAct->nActId = nActId;
				} else
					return pdFALSE;
			} else {
				return pdFALSE;
			}
	}

		pAct->nActRepeat = arep;
		pAct->nFlags = nFlags;

		pOperationsEvtElm = NULL;
		pFirstActionEvtElm = NULL;
		nState = 0;

// fill stack of parsed elements and get stack's head:
		pEvtElm = parseActionString(pAStr);

		if (!pEvtElm) return pdFALSE;

// search first element and set pointer to parent Action for eElmAction types elements:
		pFirstEvtElm = pEvtElm;
		nSizeElm = 0;
		while (pFirstEvtElm->pNextElm)
		{
			if (pFirstEvtElm->nElmType == eElmAction)
			{
				pActElm = (sActElm*)pFirstEvtElm->nElmData1;
				pActElm->pAction = pAct;
				pFirstActionEvtElm = pFirstEvtElm;
			}
			pFirstEvtElm = pFirstEvtElm->pNextElm;
			nSizeElm++;
		}

// process original stack left to right ****************************************************
// init temporary stacks for Dijkstra algorithm:
		res = initStack(&xOutStack, nSizeElm);
		if (!res) return pdFALSE;
		res = initStack(&xOpersStack, nSizeElm);
		if (!res) return pdFALSE;

// Dijkstra algorithm implementation. pEvtElm - input elements for converting into RPN:
		pEvtElm = pFirstEvtElm;
		while (pEvtElm->pPrevElm)
		{
		   	  switch(pEvtElm->nElmType)
		   	  {
					case  eElmOperation:
					{
					   	// if element == ')' then push to out stack all operations until '('
						if (pEvtElm->nElmData1 == eOpSC)
						{
							do {
								pOperationsEvtElm = (sEvtElm*)popStack(&xOpersStack);
								if (!pOperationsEvtElm) break;

								if (pOperationsEvtElm->nElmData1 != eOpSO) {
									pushStack (&xOutStack, (void*)pOperationsEvtElm);
								}

							} while (pOperationsEvtElm->nElmData1 != eOpSO);
						} else
					   	// if element == '(' then push it to operations stack
						if (pEvtElm->nElmData1 == eOpSO)
						{
							pushStack (&xOpersStack, (void*)pEvtElm);
						} else

					   	// if element == '=' then finish process parsing RPN: ***********************************************
						if (pEvtElm->nElmData1 == eOpResume)
						{
							nState = 1;
// copy all operations from operations stack to out stack:
							do {
								pOperationsEvtElm = (sEvtElm*)popStack(&xOpersStack);
								if (pOperationsEvtElm)
									pushStack (&xOutStack, (void*)pOperationsEvtElm);
							} while (pOperationsEvtElm);
							// align elements (correct next and prev pointers) according new order (in xOutStack):
							// pFirstEvtElm = old, pEvtElm = new
									pFirstEvtElm = (sEvtElm*)popStack(&xOutStack);
									pEvtElm = pFirstEvtElm;
									if (pFirstEvtElm) {
										pEvtElm->pPrevElm = NULL; // new head of elements chain
										while (pFirstEvtElm)
										{
											pEvtElm = (sEvtElm*)popStack(&xOutStack);
											pFirstEvtElm->pNextElm = pEvtElm;
											if (pEvtElm) {
												pEvtElm->pPrevElm = pFirstEvtElm;
											} else {
												pAct->pStartElmEvents  = pFirstEvtElm; // store last not NULL (first) element
											}
											pFirstEvtElm = pEvtElm;
										}
									}
									pAct->pStartElmActions = pFirstActionEvtElm;
									// to do: clear resources for dropped elements: '(', ')'
									res = pdTRUE;
						// **************************************************************************************************
						} else
						{
							// for other operation types
// if operations stack empty then operation from input push to operations stack
							if (!pOperationsEvtElm) {
								pushStack (&xOpersStack, (void*)pEvtElm);
								pOperationsEvtElm = pEvtElm;
							} else {
// if new operation priority greater operation from stack's head then push new operation to the operations stack
								if (getPriority((uint8_t) pEvtElm->nElmData1) > getPriority((uint8_t) pOperationsEvtElm->nElmData1))
								{
//									pOperationsEvtElm = elmPush (pOperationsEvtElm, pEvtElm);
									pushStack (&xOpersStack, (void*)pEvtElm);
									pOperationsEvtElm = pEvtElm;
								} else {
// else push to out stack all operations with greater priority. Push to operations stack new operation
									while (getPriority((uint8_t) pEvtElm->nElmData1) <= getPriority((uint8_t) pOperationsEvtElm->nElmData1))
									{
										pOperationsEvtElm = (sEvtElm*)popStack(&xOpersStack);
										pushStack (&xOutStack, (void*)pOperationsEvtElm);
										if (!pOperationsEvtElm) break;
									}
									pushStack (&xOpersStack, (void*)pEvtElm);
								}
							}
						}
						break;
					}
					case  eElmAction:
					{

						break;
					}
					case  eElmInterval:
					{
						// if interval timer, then add it to pAct data and start it:
						xTimerHandle xTimer;
						xTimerHandle xTimerOneShot;
						pAct->pStartElmEvents = NULL;

						if (nFlags & devACTION_FLAG_SINGLE_TIMER) {
							xTimer = mainTimerCreate("SingleTimer_1", pEvtElm->nElmData1, pdFALSE, pEvtElm);
						} else {
							xTimer = mainTimerCreate("AutoReloadTimer", pEvtElm->nElmData1, pdTRUE, pEvtElm);
						}
						pAct->xActTimer = xTimer;

						if (pEvtElm->nElmData2 > 0) {
							xTimerOneShot = mainTimerCreateOneShot("SingleTimer_2", pEvtElm->nElmData2, pEvtElm);
							pAct->xActTimerOneShot = xTimerOneShot;
						}
						xTimerStart(xTimer, 0);
						break;
					}
					default:
					{
						pushStack (&xOutStack, (void*)pEvtElm);
						break;
					}
		   	  }
		   	  // to do: process elements after '='
		   	  if (nState == 1) break;

		   	  // move to head of elements chain (left to right)
		   	  pEvtElm = pEvtElm->pPrevElm;
		}

		freeStack(&xOpersStack);
		freeStack(&xOutStack);

		return res;
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


void	clearActionsDeviceCash (sDevice* pDev)
{
	if (pDev) {
		vPortFree(pDev->pActionsCash);
		pDev->pActionsCash = NULL;
	}
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

/*
 * calculate action event formula and call action device(devices) if needed
 * return pdTRUE if success
 */
int32_t	calcAndDoAction (sAction* pAct)
{
	int32_t res = pdTRUE;
	sEvtElm* pEvtElm;
	sEvtElm* pActElm;
	sActElm* pActionInfo;
	sRPNStack xRPNStack;
	int32_t nCalcResult;
	RTC_t rtc;
//	char* buff;
	char msg[50];

	if (pAct)
	{
		resetRS(&xRPNStack);	// reset stack
		pEvtElm = pAct->pStartElmEvents;
		rtc_gettime(&rtc);

		do {
			if (pEvtElm)
			{
			   	  switch(pEvtElm->nElmType)
			   	  {
						case eElmOperation:
						{
//							xsprintf(msg, "a_oper = %d ", pEvtElm->nElmData1);
//							sendBaseOut(msg);

			    		   	  switch(pEvtElm->nElmData1)
			    		   	  {
			    		   	    case eOpResume:
			    		   	    	goto finishCalc;
			    		   	    case eOpLE:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) >= popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpGE:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) <= popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpEq:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) == popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpL:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) > popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpG:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) < popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpAND:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) && popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpOR:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) || popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpNOT:
			    	    			pushRS(&xRPNStack, (~popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpPlus:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) + popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpMinus:
			    	    			pushRS(&xRPNStack, (- popRS(&xRPNStack) + popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpMul:
			    	    			pushRS(&xRPNStack, (popRS(&xRPNStack) * popRS(&xRPNStack)));
			    		   	    	break;
			    		   	    case eOpDiv:
			    	    			pushRS(&xRPNStack, (1 / popRS(&xRPNStack) * popRS(&xRPNStack)));
			    		   	    	break;
			    		   	  }
							break;
						}
						case eElmAction:
						{
							// to do: process error. Action shouldn't be here
							res = pdFALSE;
							break;
						}
						case eElmConst:
						{
							if (!isRSFull(&xRPNStack)) {
								pushRS(&xRPNStack, pEvtElm->nElmData1);
							} else {
								// to do: process errors
								// xprintf("\r\nExpression is too complex");
								res = pdFALSE;
							}
							break;
						}
						case eElmDevValue:
						{
							pushRS(&xRPNStack, getDevValueByID((uint8_t) pEvtElm->nElmData2, (uint16_t) pEvtElm->nElmData1));
							break;
						}
						case  eElmTimeValue:
						{
							pushRS(&xRPNStack, (int32_t) (rtc.hour*3600 + rtc.min*60 + rtc.sec));
							break;
						}
						case  eElmDateYear:
						{
							pushRS(&xRPNStack, (int32_t) rtc.year);
							break;
						}
						case  eElmDateMonth:
						{
							pushRS(&xRPNStack, (int32_t) rtc.month);
							break;
						}
						case  eElmDateDay:
						{
							pushRS(&xRPNStack, (int32_t) rtc.mday);
							break;
						}
						case  eElmDateDayWeek:
						{
							pushRS(&xRPNStack, (int32_t) rtc.wday);
							break;
						}
						case eElmInterval:
						{
							break;
						}
			   	  }

			// scan left to right
			pEvtElm = pEvtElm->pPrevElm;
			}
		} while (pEvtElm);

finishCalc:
		nCalcResult = popRS(&xRPNStack);

		// if changed result and timeout over -> do all actions
		if ((pAct->nLastResult != nCalcResult) && ((rtc_get_counter_val()-pAct->nLastChange) >= pAct->nActRepeat))
		{
			// store new result and time of change
			pAct->nLastResult = nCalcResult;
			pAct->nLastChange = rtc_get_counter_val();

			pActElm = pAct->pStartElmActions;

				while (pActElm) {
					pActionInfo = (sActElm*) pActElm->nElmData1;
					if (pActionInfo) {
						if (nCalcResult) {
							uint8_t nSetDevCmd = pActionInfo->nActCmdIn;
							uint32_t nSetDevValue =  pActionInfo->nElmDataIn;
							uint8_t nSetDataType = pActionInfo->nElmDataType;

							// if data type get dev value, than call function for it:
							if (pActionInfo->nElmDataType == eElmGetDevValue) {
								nSetDevValue = getDevValueByID(pActionInfo->ngetDevCmdIn, (uint16_t) nSetDevValue);
								nSetDataType = eElmInteger; // get dev value in action works only for integer data type
							}
							setDevValueByID(nSetDevValue, nSetDevCmd, pActionInfo->nDevId, nSetDataType);
							if (nSetDataType == eElmString) {
								xsprintf(msg, " act_in[%d] -> set dev[%d,%d] = %s ", pAct->nActId, pActionInfo->nDevId, pActionInfo->nActCmdOut, (char*)pActionInfo->nElmDataIn);
							} else {
								xsprintf(msg, " act_in[%d] -> set dev[%d,%d] = %d ", pAct->nActId, pActionInfo->nDevId, nSetDevCmd, nSetDevValue);
								if (!(pAct->nFlags & devACTION_FLAG_NOLOG))
								{
								//	logAction(pAct->nActId, pActionInfo->nDevId, nSetDevValue);
								}
							}

						} else if (pActionInfo->nActFlag & devEXEC_ACTION_FLAG_OUT_ARG)	{
							setDevValueByID(pActionInfo->nElmDataOut, pActionInfo->nActCmdOut, pActionInfo->nDevId, pActionInfo->nElmDataType);
							if (pActionInfo->nElmDataType == eElmString) {
								xsprintf(msg, " act_out[%d] -> set dev[%d,%d] = %s ", pAct->nActId, pActionInfo->nDevId, pActionInfo->nActCmdOut, (char*)pActionInfo->nElmDataOut);
							} else {
								xsprintf(msg, " act_out[%d] -> set dev[%d,%d] = %d ", pAct->nActId, pActionInfo->nDevId, pActionInfo->nActCmdOut, pActionInfo->nElmDataOut);
								if (!(pAct->nFlags & devACTION_FLAG_NOLOG))
								{
								//	logAction(pAct->nActId, pActionInfo->nDevId, pActionInfo->nElmDataOut);
								}
							}
						}
					}

					if (!(pAct->nFlags & devACTION_FLAG_NOLOG))
					{
						sendBaseOut(msg);
						// to do: make notification!!!
	//					buff = pvPortMalloc(50);
	//					if (buff) {
	//						xsprintf(buff, "set dev[%d] = %d ",actArray[i]->nActDevID, actArray[i]->nActDevValue);
	//							vSendInputMessage (1, mainINFO_MESSAGE, main_IF_PROGRAM, main_IF_PROGRAM, (void*) buff, strlen(buff),0);
					}

					pActElm = pActElm->pPrevElm;
					}
		}
	}

	return res;
}

