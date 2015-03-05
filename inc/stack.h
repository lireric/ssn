/*
 * stack.h
 *
 *  Created on: 24 февр. 2015 г.
 *      Author: eric
 */

#ifndef INC_STACK_H_
#define INC_STACK_H_

#include "FreeRTOS.h"

// ************************************************************************
// Common stack functions for work with pointers

typedef struct
{
	void* pStackArray;	// pointer to the stack array
	uint8_t top;
	uint8_t nMaxSize;
} sCommonStack;

int32_t initStack(sCommonStack* psStack, uint8_t nMaxSize);
void 	freeStack(sCommonStack* psStack);
int32_t isStackEmpty(sCommonStack* psStack);
int32_t isStackFull(sCommonStack* psStack);
void 	pushStack(sCommonStack* psStack, void* pPointer);
void* 	popStack(sCommonStack* psStack);
uint8_t getCurrentStackSize(sCommonStack* psStack);


// ************************************************************************
// RPN stack functions for work with values
#ifndef MAX_ACTION_ARRAY_SIZE
#define MAX_ACTION_ARRAY_SIZE (30)
#endif


typedef struct
{
	int32_t stack[MAX_ACTION_ARRAY_SIZE]; // values array
	uint8_t top;
} sRPNStack;

void 	resetRS(sRPNStack* pRPNStack);
int32_t isRSEmpty(sRPNStack* pRPNStack);
int32_t isRSFull(sRPNStack* pRPNStack);
void 	pushRS(sRPNStack* pRPNStack, int32_t value);
int32_t popRS(sRPNStack* pRPNStack);
int 	getCurrentRSSize(sRPNStack* pRPNStack);

#endif /* INC_STACK_H_ */
