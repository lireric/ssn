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
/* stepmotor.c
 *
 *  Created on: 12 окт. 2016 г.
 *      Author: eric
 */

#include "device.h"
#include "stepmotor.h"
#include <libopencm3/stm32/gpio.h>
#include "FreeRTOS.h"
#include "../inc/ssn.h"
#include "commands.h"

//extern static void 	StepMotorTimerFunction(void* pParam);

//StaticTimer_t xTimerBuffer;

static void StepMotorTimerFunction(void* pParam)
{
	sDevice* dev = (sDevice*) pvTimerGetTimerID(pParam);

	StepMotorNextStep(dev);
}


/*
 * Hardware init and memory allocation for device structure
 */
stepmotor_data_t* StepMotorInitStruct(sDevice* dev)
{

	stepmotor_data_t* pSMDev = NULL;

	if (dev) {
		stepmotor_data_t* pSMDev = (stepmotor_data_t*) pvPortMalloc(sizeof(stepmotor_data_t));

		if (pSMDev) {
			memset(pSMDev, 0, sizeof(stepmotor_data_t));
			dev->pDevStruct = (void*) pSMDev;
			// set 0xFF values for next checking port correctness
			dev->pGroup->GrpDev.ucPin = 0xFF;
			dev->pGroup->GrpDev.ucPin2 = 0xFF;
			pSMDev->ucPinPhase[0] = 0xFF;
			pSMDev->ucPinPhase[1] = 0xFF;
			pSMDev->ucPinPhase[2] = 0xFF;
			pSMDev->ucPinPhase[3] = 0xFF;
		}
	}
	return pSMDev;
}

void StepMotorInitHW(sDevice* dev)
{
//	char msg[mainMAX_MSG_LEN];
	stepmotor_data_t* pSMDev = (stepmotor_data_t*) dev->pDevStruct;

	if (pSMDev) {

		rcc_periph_clock_enable(dev->pGroup->GrpDev.pPort);

		if (dev->pGroup->GrpDev.ucPin != 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << dev->pGroup->GrpDev.ucPin);
		if (dev->pGroup->GrpDev.ucPin2 != 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << dev->pGroup->GrpDev.ucPin2);
		if (pSMDev->ucPinPhase[0]!= 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << pSMDev->ucPinPhase[0]);
		if (pSMDev->ucPinPhase[1] != 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << pSMDev->ucPinPhase[1]);
		if (pSMDev->ucPinPhase[2] != 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << pSMDev->ucPinPhase[2]);
		if (pSMDev->ucPinPhase[3] != 0xFF)
			gpio_set_mode(dev->pGroup->GrpDev.pPort, GPIO_MODE_OUTPUT_2_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, 1 << pSMDev->ucPinPhase[3]);

		// set initial phases values:

		switch (pSMDev->sm_type) {
			case eSMAlg_WD:
			default:
				pSMDev->phases[0] = 0b10001000;
				pSMDev->phases[1] = 0b01000100;
				pSMDev->phases[2] = 0b00100010;
				pSMDev->phases[3] = 0b00010001;
				break;
			case eSMAlg_FS:
				pSMDev->phases[0] = 0b10011001;
				pSMDev->phases[1] = 0b11001100;
				pSMDev->phases[2] = 0b01100110;
				pSMDev->phases[3] = 0b00110011;
				break;
			case eSMAlg_HS:
				pSMDev->phases[0] = 0b11000001;
				pSMDev->phases[1] = 0b01110000;
				pSMDev->phases[2] = 0b00011100;
				pSMDev->phases[3] = 0b00000111;
				break;
			}

		pSMDev->uiState = eSMInit; // set init state

		StepMotorSetAllPhases(dev);

		// perform calibration:
		pSMDev->iPositionMax = pSMDev->iPositionPrefMax; // copy manual setted max position
		if (pSMDev->uiFlag & SM_FLAG_NEED_CALIBRATION)
			StepMotorCalibrate(dev);

		// create timer for this motor:
		pSMDev->pStepMotorTimerHandle = xTimerCreate("StepMotorTimer", pSMDev->uiStepTime / portTICK_RATE_MS, pdTRUE, (void*)dev, StepMotorTimerFunction);
//		pSMDev->pStepMotorTimerHandle = xTimerCreateStatic("StepMotorTimer", pSMDev->uiStepTime / portTICK_RATE_MS, pdTRUE, (void*)dev, StepMotorTimerFunction, &(xTimerBuffer));
		if (pSMDev->pStepMotorTimerHandle)
			xTimerStart(pSMDev->pStepMotorTimerHandle, 0);

		if (!(pSMDev->uiFlag & SM_FLAG_CURR_ON))
			StepMotorOffAllPhases(dev);

		pSMDev->uiState = eSMReady; // device ready to next steps
		xprintfMsg("\r\nStep motor device initialized: %d ", dev->nId);
	}

}

/* Set/Reset phases:
 * dev - step motor device
 * uiPhaseNum - phase numer (0..4)
 * ubValue - setting value (0 - reset, any other - set)
*/
void StepMotorSetPhase(sDevice* dev, uint8_t uiPhaseNum, uint8_t ubValue)
{
	if (dev) {
		if (ubValue)
			gpio_set(dev->pGroup->GrpDev.pPort, 1<<((stepmotor_data_t*) dev->pDevStruct)->ucPinPhase[uiPhaseNum]);
		else
			gpio_clear(dev->pGroup->GrpDev.pPort, 1<<((stepmotor_data_t*) dev->pDevStruct)->ucPinPhase[uiPhaseNum]);
	}
}

/* Check value of the 0 position end switch:
 * dev - step motor device
 * return 1 if switch on
*/
uint32_t StepMotorCheckPosition0(sDevice* dev)
{
	volatile uint8_t result = 0;
	if (dev) {
		if (dev->pGroup->GrpDev.ucPin != 0xFF) {

		result = GPIO_IDR(dev->pGroup->GrpDev.pPort) & (1 << dev->pGroup->GrpDev.ucPin);

		if (result != 0)
			result = 1;
		else
			 result = 0;
		}
	}
	return result;
}

/* Check value of the MAX position end switch:
 * dev - step motor device
 * return 1 if switch on
*/
uint32_t StepMotorCheckPositionMax(sDevice* dev)
{
	volatile uint8_t result = 0;
	if (dev) {
		if (dev->pGroup->GrpDev.ucPin2 != 0xFF) {
		result = GPIO_IDR(dev->pGroup->GrpDev.pPort) & (1 << dev->pGroup->GrpDev.ucPin2);

		if (result != 0)
			result = 1;
		else
			 result = 0;
		}
	}
	return result;
}

/* Apply current state to all phases
 *
 */
void StepMotorSetAllPhases(sDevice* dev)
{
	for (uint8_t i = 0; i < 4; i++) {
		StepMotorSetPhase(dev, i, ((stepmotor_data_t*) (dev->pDevStruct))->phases[i] & 0x01);
	}
	dev->uiLastUpdate = rtc_get_counter_val();
//	((stepmotor_data_t*) (dev->pDevStruct))->uiLastTick = getMainTimerTick();
}

/* Switch off all phases
 *
 */
void StepMotorOffAllPhases(sDevice* dev)
{
	for (uint8_t i = 0; i < 4; i++) {
		StepMotorSetPhase(dev, i, 0);
	}
	dev->uiLastUpdate = rtc_get_counter_val();
}

/* Calculate new state to all phases
 *const unsigned int mask = (CHAR_BIT*sizeof(n)-1);

  assert ( (c<=mask) &&"rotate by type width or more");
  c &= mask;  // avoid undef behaviour with NDEBUG.  0 overhead for most types / compilers
  return (n<<c) | (n>>( (-c)&mask ));
 */
void StepMotorMoveAllPhases(sDevice* dev)
{
	for (uint8_t i = 0; i < 4; i++) {
		if (((stepmotor_data_t*)(dev->pDevStruct))->iDirection > 0) {
			// ROTL
			((stepmotor_data_t*) (dev->pDevStruct))->phases[i] =
					(((stepmotor_data_t*) (dev->pDevStruct))->phases[i] << 1) | (((stepmotor_data_t*) (dev->pDevStruct))->phases[i] >> 7);
		} else {
			// ROTR
			((stepmotor_data_t*) (dev->pDevStruct))->phases[i] =
					(((stepmotor_data_t*) (dev->pDevStruct))->phases[i] >> 1) | (((stepmotor_data_t*) (dev->pDevStruct))->phases[i] << 7);

		}
	}
}

/* Perform calibration
 *
 */
void StepMotorCalibrate(sDevice* dev)
{
	// check for correct setting pins of switches (as minimum 0 position switch)
	if (dev) {
		if (dev->pGroup->GrpDev.ucPin != 0xFF) {
			((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMCalibrating0; // set calibrating state
			((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition = 0;	// move until 0 position switch on
			((stepmotor_data_t*) (dev->pDevStruct))->iDirection = -1; 		// move to begin
		} else {
			((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMReady; // cannot perform calibrating, reset calibrating state
		}
	}
}

/* Perform step forward or backward
 * step can be performed only in eSMReady state!
 *
 */
void StepMotorOneStep(sDevice* dev, int8_t iDirection)
{
	if (((stepmotor_data_t*)(dev->pDevStruct))->uiState == eSMReady) {
		if (iDirection < 0 )
			iDirection = -1;
		else
			iDirection = 1;
		((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition += iDirection;
		((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMMovingToPoint;
		StepMotorNextStep(dev);
	}
}

/* Perform next step
 *
 */
void StepMotorNextStep(sDevice* dev)
{
//	char msg[mainMAX_MSG_LEN];

	if (dev) {
		switch (((stepmotor_data_t*)(dev->pDevStruct))->uiState) {
			case eSMCalibrating0:
				// check setting pin0
				if (StepMotorCheckPosition0(dev)) {
					((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep = 0;
					// if setted pin2 than calibrate it, else stop calibration
					if (dev->pGroup->GrpDev.ucPin2 != 0xFF) {
						((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMCalibratingMax;
						((stepmotor_data_t*) (dev->pDevStruct))->iDirection = 1;
					} else {
						if (!(((stepmotor_data_t*)(dev->pDevStruct))->uiFlag & SM_FLAG_CURR_ON))
							StepMotorOffAllPhases(dev);
						((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMReady;
					}
				} else {
					StepMotorMoveAllPhases(dev);
					StepMotorSetAllPhases(dev);
				}
				break;

			case eSMCalibratingMax:
				if (StepMotorCheckPositionMax(dev)) {
					((stepmotor_data_t*) (dev->pDevStruct))->iPositionMax = ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep;
					if (!(((stepmotor_data_t*)(dev->pDevStruct))->uiFlag & SM_FLAG_CURR_ON))
						StepMotorOffAllPhases(dev);
					((stepmotor_data_t*) (dev->pDevStruct))->uiState = eSMReady;
				} else {
					((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep += ((stepmotor_data_t*) (dev->pDevStruct))->iDirection;
					StepMotorMoveAllPhases(dev);
					StepMotorSetAllPhases(dev);
				}
				break;

			case eSMMovingToPoint:
//				xsprintf(( portCHAR *) msg, "\r\nMotor[%d], step: %d of %d", dev->nId, ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep, ((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition);
//				debugMsg(msg);

				if (((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition > ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep)
					((stepmotor_data_t*) (dev->pDevStruct))->iDirection = 1;

				if (((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition < ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep)
					((stepmotor_data_t*) (dev->pDevStruct))->iDirection = -1;

				if (((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition != ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep) {
					// check for switches states. If not ON, than make step
					if (!StepMotorCheckPosition0(dev) && !StepMotorCheckPositionMax(dev)) {
						((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep += ((stepmotor_data_t*) (dev->pDevStruct))->iDirection;
						StepMotorMoveAllPhases(dev);
						StepMotorSetAllPhases(dev);
					}

				} else {
					if (!(((stepmotor_data_t*)(dev->pDevStruct))->uiFlag & SM_FLAG_CURR_ON))
						StepMotorOffAllPhases(dev);
					((stepmotor_data_t*)(dev->pDevStruct))->uiState = eSMReady;
				}
				break;

			case eSMReady:
			case eSMInit:
				break;
			}
	}

}

/* Set new target position
 *
 */
void StepMotorSetTargetPosition(sDevice* dev, int32_t 	iTargetPosition)
{
	if (dev) {
		((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition = iTargetPosition;
		((stepmotor_data_t*)(dev->pDevStruct))->uiState = eSMMovingToPoint;
	}
}

/* Get absolute position
 *
 */
int32_t StepMotorGetPosition(sDevice* dev)
{
		return ((stepmotor_data_t*) (dev->pDevStruct))->iCurrentStep;
}

/* Get target position
 *
 */
int32_t StepMotorGetTargetPosition(sDevice* dev)
{
		return ((stepmotor_data_t*) (dev->pDevStruct))->iTargetPosition;
}

/* Get relative position
 *
 */
int8_t StepMotorGetRelativePosition(sDevice* dev)
{
	int8_t 	iRelPosition = 0;
	if (((stepmotor_data_t*)(dev->pDevStruct))->iPositionMax) {
		iRelPosition = (100*((stepmotor_data_t*)(dev->pDevStruct))->iCurrentStep)/((stepmotor_data_t*)(dev->pDevStruct))->iPositionMax;
	}
	return iRelPosition;
}

/* Get current motor state
 *
 */
uint8_t StepMotorGetCurrentState(sDevice* dev)
{
		return ((stepmotor_data_t*)(dev->pDevStruct))->uiState;
}

/* Set new target position in percents (0..100) relative max position
 *
 */
void StepMotorSetTargetPositionPercents(sDevice* dev, uint8_t 	uiPosition)
{
	int32_t 	iTargetPosition = 0;

	if (dev) {
		if (iTargetPosition > 100)
			iTargetPosition = 100;

		iTargetPosition = (uiPosition * ((stepmotor_data_t*)(dev->pDevStruct))->iPositionMax) / 100;

		StepMotorSetTargetPosition(dev, iTargetPosition);
	}
}


