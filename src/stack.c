/*
 * stack.c
 *
 *  Created on: 24 февр. 2015 г.
 *      Author: eric
 */

#include "stack.h"

// ************************************************************************
// Common stack functions for work with pointers


int32_t initStack(sCommonStack* psStack, uint8_t nMaxSize) {
	int32_t res = pdFALSE;
	if (psStack) {
		psStack->pStackArray = pvPortMalloc(nMaxSize * sizeof(void*));
		if (psStack->pStackArray) {
			psStack->top = 0;
			psStack->nMaxSize = nMaxSize;
			res = pdTRUE;
		}
	}
	return res;
}

void freeStack(sCommonStack* psStack)
{
	if (psStack)
		vPortFree(psStack->pStackArray);
}

int32_t isStackEmpty(sCommonStack* psStack)
{
		return psStack->top == 0;
}

int32_t isStackFull(sCommonStack* psStack)
{
	return psStack->top == psStack->nMaxSize;
}

void pushStack(sCommonStack* psStack, void* pPointer)
{
	((void**)psStack->pStackArray)[psStack->top++] = pPointer;
}

void* popStack(sCommonStack* psStack)
{
	if (isStackEmpty(psStack))
	{
		return NULL;
	}
	return ((void**)psStack->pStackArray)[--(psStack->top)];
}

uint8_t getCurrentStackSize(sCommonStack* psStack)
{
	return psStack->top;
}

// ************************************************************************
// RPN stack functions for work with values

int32_t resetRS(sRPNStack* pRPNStack, uint8_t nMaxSize) {
	int32_t res = pdFALSE;
	if (pRPNStack) {
		pRPNStack->stack = (int32_t*) pvPortMalloc(nMaxSize * sizeof(int32_t));
		if (pRPNStack->stack) {
			pRPNStack->top = 0;
			pRPNStack->nMaxSize = nMaxSize;
			res = pdTRUE;
		}
	}
	return res;
//	pRPNStack->top = 0;
}

void freeRS(sRPNStack* pRPNStack)
{
	if (pRPNStack)
		vPortFree(pRPNStack->stack);
}

int32_t isRSEmpty(sRPNStack* pRPNStack)
{
	return pRPNStack->top == 0;
}

int32_t isRSFull(sRPNStack* pRPNStack)
{
	return pRPNStack->top == MAX_ACTION_ARRAY_SIZE;
}

void pushRS(sRPNStack* pRPNStack, int32_t value)
{
	pRPNStack->stack[pRPNStack->top++] = value;
}

int32_t popRS(sRPNStack* pRPNStack)
{
	if (isRSEmpty(pRPNStack))
	{
		// to do: process error
		// xprintf("\r\nNot enough operands in expression");
	}
	return pRPNStack->stack[--(pRPNStack->top)];
}

int getCurrentRSSize(sRPNStack* pRPNStack)
{
	return pRPNStack->top;
}

