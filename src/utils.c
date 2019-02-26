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


#include <assert.h>
#include <string.h>
#include "projdefs.h"
#include "utils.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

volatile int critical_section_counter;

// --------------------------------------------------------------------------------------------------------------
// parse comma delimited string. return number of parsed values
// input:  char* - string buffer, uint16_t* - pointer to filled array of integer values, uint8_t - max array size
//
uint8_t parseCommaString(char* psChannels, uint8_t* pChannelArray, uint8_t nArrayMaxSize)
{
	uint8_t j=0, i=0, k=0;
	uint8_t ch;
	char cBuf[3] = {0};

	for (j = 0; ((j <= strlen(psChannels)) && (i < nArrayMaxSize)); j++) {
		if ((psChannels[j] == ',') || (j == strlen(psChannels)))
		{
			cBuf[k] = 0;
			if (strlen(cBuf) > 0) {
				ch = conv2d(cBuf);
				pChannelArray[i++] = ch;
				k = 0;
			}
		} else if ((psChannels[j] >= '0') && (psChannels[j] <= '9')) {
			cBuf[k++] = psChannels[j];
		}
	}
	return i;
}

/* Version of strncpy that ensures dest (size bytes) is null-terminated. */
char* strncpy0(char* dest, const char* src, size_t size)
{
    strncpy(dest, src, size);
    dest[size - 1] = '\0';
    return dest;
}

char *strnext(char **sp, const char *delim)
{
	char *tk = *sp;
	char *p;

	if(*sp) {
		p = strpbrk(tk, delim);
		if(p) {
			/* found delimiter */
			if(*p) {
				*sp = p+1;
			} else {
				*sp = 0; /* nothing ahead, so terminate */
			}
			*p = '\0';	/* terminate on the delimiter */
		} else {
			tk = 0;		/* not found */
		}
	}
	return tk;
}

uint32_t get_port_by_name(char* name)
{
	if (!strcmp(name,"GPIOA")) {return GPIOA;}
	else if (!strcmp(name,"GPIOB")) {return GPIOB;}
	else if (!strcmp(name,"GPIOC")) {return GPIOC;}
	else if (!strcmp(name,"GPIOD")) {return GPIOD;}
	else if (!strcmp(name,"GPIOE")) {return GPIOE;}
	else if (!strcmp(name,"GPIOF")) {return GPIOF;}
	else if (!strcmp(name,"GPIOG")) {return GPIOG;}
	else if (!strcmp(name,"TIM1")) {return TIM1;}
	else if (!strcmp(name,"TIM2")) {return TIM2;}
	else if (!strcmp(name,"TIM3")) {return TIM3;}
	else if (!strcmp(name,"TIM4")) {return TIM4;}
	else if (!strcmp(name,"TIM5")) {return TIM5;}
	else if (!strcmp(name,"TIM6")) {return TIM6;}
	else if (!strcmp(name,"TIM7")) {return TIM7;}
	else if (!strcmp(name,"TIM8")) {return TIM8;}
	return 0;
}

uint32_t get_rcc_by_port(uint32_t nPort)
{
	uint32_t nRet = 0;
	switch (nPort) {
		case TIM1:	nRet = RCC_TIM1;	break;
		case TIM2:	nRet = RCC_TIM2;	break;
		case TIM3:	nRet = RCC_TIM3;	break;
		case TIM4:	nRet = RCC_TIM4;	break;
		case TIM5:	nRet = RCC_TIM5;	break;
		case TIM6:	nRet = RCC_TIM6;	break;
		case TIM7:	nRet = RCC_TIM7;	break;
		}
	return nRet;
}

uint32_t conv2d(const char* p) {
	uint32_t v = 0;
	uint8_t i;
	uint8_t nc = 0;
	uint32_t d = 1;
	if (p) {
		for (i = strlen(p) - 1; i >= 0; i--) {
			if (i > 12)
				break;
			if ('0' <= p[i] && p[i] <= '9')
				nc = p[i] - '0';
			v += nc * d;
			d *= 10;
		}
	}
	return v;
}

uint32_t convHex2d(const char* p) {
	uint32_t v = 0;
	uint8_t i;
	uint8_t nc = 0;
	uint32_t d = 1;
	if (p) {
		for (i = strlen(p) - 1; i >= 0; i--) {
			if (i > 8)	// 32bit restriction
				break;
			if ('0' <= p[i] && p[i] <= '9')	{ nc = p[i] - '0'; } else
			if ('a' <= p[i] && p[i] <= 'f')	{ nc = p[i] - 'a' + 10; } else
			if ('A' <= p[i] && p[i] <= 'F')	{ nc = p[i] - 'A' + 10; } else
			{ break; }
			v += nc * d;
			d *= 0x10;
		}
	}
	return v;
}



int32_t	GetNumbersValue(char* pcSrcString)
{
	uint8_t nNumLen;
	uint8_t nNumBegin;
	char pcNumberString[12];
	int32_t xReturn = 0;	// return 0, if string not contain any cifer
	// search digits until \n or end of string
	for (nNumBegin=0; nNumBegin<strlen(pcSrcString); nNumBegin++)
	{
		nNumLen = strspn (pcSrcString+nNumBegin, "1234567890");
		if ((nNumLen > 0) || (pcSrcString[nNumBegin]=='\n')) {
			break;
		}
	}
	if (nNumLen > 0) {
		memcpy (pcNumberString, &pcSrcString[nNumBegin], nNumLen);
		pcNumberString[nNumLen]=0;
		xReturn = conv2d (pcNumberString);
	}
	return xReturn;
}

int32_t	fillCommandStruct(char* pcBuf, sSSNCommand* xSSNCommand)
{
	int32_t xReturn = pdFALSE;
	char* cp = strstr (pcBuf, "==begin==");
	if (cp) {
		cp += 10;
		xSSNCommand->nCmdID = GetNumbersValue(cp);
		cp = strchr (cp, '\n')+1;
		xSSNCommand->nCmd = GetNumbersValue(cp);
		cp = strchr (cp, '\n')+1;
		xSSNCommand->nCmdsLeft = GetNumbersValue(cp);
		cp = strchr (cp, '\n')+1;
		if (cp == strstr (pcBuf, "\n==end==")) {
			xSSNCommand->pcData = 0;
		} else {
			xSSNCommand->pcData = cp;
		}
		xReturn = pdTRUE;
	}
	return xReturn;
}
