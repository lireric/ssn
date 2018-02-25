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
/* stepmotor.h
 *
 *  Created on: 12 окт. 2016 г.
 *      Author: eric
 */

#ifndef INC_STEPMOTOR_H_
#define INC_STEPMOTOR_H_

#include "device.h"
#include "stdint.h"

#define SM_FLAG_CURR_ON				(0x01) // switch off motor after the setting position
#define SM_FLAG_NEED_CALIBRATION	(0x02)

/* Step motor states */
typedef enum
{
	eSMInit = 0,			// initial state
	eSMCalibrating0,		// in calibrating process - search 0 position
	eSMCalibratingMax,		// in calibrating process - search maximum position
	eSMError,				// device error
	eSMMovingToPoint,		// in movement to target  position
	eSMReady				// calibration passed, moving to target finished, etc - ready to work
} eStepMotorStates;

/* Step motor control phases algorithms */
typedef enum
{
	eSMAlg_WD = 0,			// wave drive mode
	eSMAlg_FS,				// full step mode
	eSMAlg_HS				// half step mode
} eStepMotorAlg;

typedef struct{
	int32_t		iCurrentStep;
    int32_t 	iTargetPosition;// target position for next steps
	int32_t		iPositionMax;	// calibrated maximum position (in steps)
	xTimerHandle pStepMotorTimerHandle; // step motor timer handle
    uint8_t		phases[4];		// current phases values ( 0 - A, 1 - B, 2 - C, 3 - D)
    uint8_t 	uiState;		// current state (according eStepMotorStates enum)
    int8_t 		iDirection;		// current movement direction (+1 / -1)

	/* Settings */
    uint8_t	uiFlag;				// 1-st bit: current control (0 - switch off current, after the step, 1 - current stay on)
    							// 2-nd bit: need or not calibration (0 - not need, 1 - need)
    uint8_t sm_type;			// step motor algorithm (eStepMotorAlg enum): 0 = wave drive mode, 1 = full step mode, 2 = half step mode
	int32_t uiStepTime;			// step time (ms)
	int32_t	iPositionPrefMax;	// manual max position. If set, this value coping to iPositionMax (instead automatic calibration)
    // step motor control pins
    // At base sGrpDev structure: Pin1 = 0 position end switch (not mandatory); Pin2 = max position end switch (not mandatory)
    uint8_t ucPinPhase[4];		// 0 - pin phase A, 1 - pin phase B, 2 - pin phase C, 3 - pin phase D
} stepmotor_data_t;

void 				deviceProcAttributes_StepMotor(sDevice* pDev, char* sName, char* sValue);
stepmotor_data_t* 	StepMotorInitStruct(sDevice* dev);
int32_t				StepMotorInitHW(sDevice* dev);
void 				StepMotorSetPhase(sDevice* dev, uint8_t uiPhaseNum, uint8_t ubValue);
void 				StepMotorSetAllPhases(sDevice* dev);
void 				StepMotorCalibrate(sDevice* dev);
void 				StepMotorMoveAllPhases(sDevice* dev);
//static void 		StepMotorTimerFunction(void* pParam);
void 				StepMotorNextStep(sDevice* dev);
void 				StepMotorOneStep(sDevice* dev, int8_t iDirection);
void 				StepMotorSetTargetPosition(sDevice* dev, int32_t 	iTargetPosition);
void 				StepMotorSetTargetPositionPercents(sDevice* dev, uint8_t 	uiPosition);
void 				StepMotorOffAllPhases(sDevice* dev);
int32_t 			StepMotorGetPosition(sDevice* dev);
int8_t	 			StepMotorGetRelativePosition(sDevice* dev);
int32_t 			StepMotorGetTargetPosition(sDevice* dev);
uint8_t 			StepMotorGetCurrentState(sDevice* dev);

#endif /* INC_STEPMOTOR_H_ */
