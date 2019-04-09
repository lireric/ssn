/*
 * ssnpdu.c
 *
 *  Created on: 7 апр. 2019 г.
 *      Author: eric
 */

/* SSN PDU process procedures.
 *
 */

#include "stdint.h"
#include "ssnpdu.h"
#include "ssn.h"
#include "projdefs.h"
#include "commands.h"
#include "utils.h"

/* Parse buffer with SSN PDE string
 * input:  pointer to PDU string
 * output: pointer to SSNPDU structure or NULL if error
 * */
sSSNPDU* parseSSNPDU (char * buffer)
{
	uint16_t calcCRC;
	char cTmpBuf[6];
	if (!buffer) { return pdFAIL; }
	if (strlen(buffer) < (15+strlen(cSSNSTART))) { return pdFAIL; }
	if (!strstr(buffer, cSSNSTART)) { return pdFAIL; }
	sSSNPDU* xSSNPDU = (sSSNPDU*)pvPortMalloc(sizeof(sSSNPDU));
	if (!xSSNPDU) { return pdFAIL; }

	xSSNPDU->buffer = 0;
	xSSNPDU->state = SSN_STATE_INIT;
	memcpy(xSSNPDU->cSSNBuffer,(buffer+strlen(cSSNSTART)),15);

	// get object destination
	cTmpBuf[3] = xSSNPDU->cSSNBuffer[3];
	cTmpBuf[2] = xSSNPDU->cSSNBuffer[2];
	cTmpBuf[1] = xSSNPDU->cSSNBuffer[1];
	cTmpBuf[0] = xSSNPDU->cSSNBuffer[0];
	cTmpBuf[4] = 0;
	xSSNPDU->obj_dest = convHex2d(cTmpBuf);

	if ((xSSNPDU->obj_dest != getMC_Object()) || (xSSNPDU->obj_dest == 0)) {
		// if other destination object cancel and return
		vPortFree(xSSNPDU);
		return pdFAIL;
	}
					// get source object
	cTmpBuf[3] = xSSNPDU->cSSNBuffer[7];
	cTmpBuf[2] = xSSNPDU->cSSNBuffer[6];
	cTmpBuf[1] = xSSNPDU->cSSNBuffer[5];
	cTmpBuf[0] = xSSNPDU->cSSNBuffer[4];
	cTmpBuf[4] = 0;
	xSSNPDU->obj_src = convHex2d(cTmpBuf);

	// get message type
	cTmpBuf[1] = xSSNPDU->cSSNBuffer[9];
	cTmpBuf[0] = xSSNPDU->cSSNBuffer[8];
	cTmpBuf[2] = 0;
	xSSNPDU->message_type = convHex2d(cTmpBuf);

	// get message length
	cTmpBuf[3] = xSSNPDU->cSSNBuffer[13];
	cTmpBuf[2] = xSSNPDU->cSSNBuffer[12];
	cTmpBuf[1] = xSSNPDU->cSSNBuffer[11];
	cTmpBuf[0] = xSSNPDU->cSSNBuffer[10];
	cTmpBuf[4] = 0;
	xSSNPDU->nDataSize = convHex2d(cTmpBuf);

	if (xSSNPDU->nDataSize > (strlen(buffer) - (15+strlen(cSSNSTART)))) {
		vPortFree(xSSNPDU);
		xprintfMsg("\r\nError! SSN buffer less then PDU size");
		return pdFAIL;
	}

	if (xSSNPDU->nDataSize < mainMINMEMORYALLOCATE) {
		xSSNPDU->buffer = pvPortMalloc(mainMINMEMORYALLOCATE);
	} else {
		xSSNPDU->buffer = pvPortMalloc(xSSNPDU->nDataSize+4);
	}

	if (!xSSNPDU->buffer) {
		xSSNPDU->state = SSN_STATE_ERROR;
		xprintfMsg("\r\nSSN buffer allocation error!");
		vPortFree(xSSNPDU);
		return pdFAIL;
	} else {
		xSSNPDU->state = SSN_STATE_DATA;
		xSSNPDU->counter = 0;
	}

	memcpy(xSSNPDU->buffer, buffer+(15+strlen(cSSNSTART)-1), xSSNPDU->nDataSize+4);

	// get CRC
	cTmpBuf[0] = xSSNPDU->buffer[xSSNPDU->nDataSize];
	cTmpBuf[1] = xSSNPDU->buffer[xSSNPDU->nDataSize+1];
	cTmpBuf[2] = xSSNPDU->buffer[xSSNPDU->nDataSize+2];
	cTmpBuf[3] = xSSNPDU->buffer[xSSNPDU->nDataSize+3];
	cTmpBuf[4] = 0;

	xSSNPDU->crc16 = convHex2d(cTmpBuf);
	calcCRC = crc16((uint8_t*) xSSNPDU->buffer, xSSNPDU->nDataSize);
	if (xSSNPDU->crc16 == calcCRC) {
		xSSNPDU->state = SSN_STATE_READY;
		return xSSNPDU;
	} else {
		xSSNPDU->state = SSN_STATE_ERROR;
		xprintfMsg("\r\nSSN data CRC error! (calc=%04x, msg=%04x)", calcCRC,
				xSSNPDU->crc16);
		vPortFree(xSSNPDU->buffer);
		vPortFree(xSSNPDU);
	}
	return pdFAIL;
}

