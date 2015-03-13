/*
    FreeRTOS V7.5.0 - Copyright (C) 2013 Real Time Engineers Ltd.

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
	INTERRUPT DRIVEN SERIAL PORT DRIVER.
*/


/******************************************************************************
*** NOTE:  COM0 == USART1, COM1 == USART2
******************************************************************************/

/*
 * Modifications for SSN project - using libopencm3 library
 *
 */

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/* Driver includes. */
#include "STM32_USART.h"
/*-----------------------------------------------------------*/

/* The number of COM ports that can be controlled at the same time. */
#define serNUM_COM_PORTS				( 2 )

/* Queues are used to hold characters that are waiting to be transmitted.  This
constant sets the maximum number of characters that can be contained in such a
queue at any one time. */
#define serTX_QUEUE_LEN					( 30 )

/* Queues are used to hold characters that have been received but not yet 
processed.  This constant sets the maximum number of characters that can be 
contained in such a queue. */
#define serRX_QUEUE_LEN					( 30 )

/* The maximum amount of time that calls to lSerialPutString() should wait for
there to be space to post each character to the queue of characters waiting
transmission.  NOTE!  This is the time to wait per character - not the time to
wait for the entire string. */
#define serPUT_STRING_CHAR_DELAY		( 6 / portTICK_RATE_MS )

/*-----------------------------------------------------------*/

/* References to the USART peripheral addresses themselves. */
//static USART_TypeDef * const xUARTS[ serNUM_COM_PORTS ] = { ( ( USART_TypeDef * ) USART1_BASE ), ( ( USART_TypeDef * ) USART2_BASE ) };

/* Queues used to hold characters waiting to be transmitted - one queue per port. */
static xQueueHandle xCharsForTx[ serNUM_COM_PORTS ] = { 0 };

/* Queues holding received characters - one queue per port. */
static xQueueHandle xRxedChars[ serNUM_COM_PORTS ] = { 0 };

static uint32_t xUSART[2] = {USART1,USART2};

/*-----------------------------------------------------------*/

/* UART interrupt handlers, as named in the vector table. */

/*-----------------------------------------------------------*/

/*
 * See header file for parameter descriptions.
 */
long lCOMPortInit( unsigned long ulPort, unsigned long ulWantedBaud, uint8_t priority )
{
long lReturn = pdFAIL;

	if( ulPort < serNUM_COM_PORTS )
	{

		/* Init the buffer structures with the buffer for the COM port being
		initialised, and perform any non-common initialisation necessary.  This
		does not check to see if the COM port has already been initialised. */
		if( ulPort == 0 )
		{
			if (!xCharsForTx[0]) {
			/* Create the queue of chars that are waiting to be sent to COM0. */
			xCharsForTx[ 0 ] = xQueueCreate( serTX_QUEUE_LEN, sizeof( char ) );
			}
			if (!xRxedChars[0]) {
			/* Create the queue used to hold characters received from COM0. */
			xRxedChars[ 0 ] = xQueueCreate( serRX_QUEUE_LEN, sizeof( char ) );
			}

			/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
			rcc_periph_clock_enable(RCC_GPIOA);
			rcc_periph_clock_enable(RCC_AFIO);
			rcc_periph_clock_enable(RCC_USART1);

			/* Enable the USART1 interrupt. */
			nvic_enable_irq(NVIC_USART1_IRQ);
			nvic_set_priority(NVIC_USART1_IRQ, priority);

			/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
			gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

			/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
			gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
				      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

			/* Setup UART parameters. */
			usart_set_baudrate(USART1, ulWantedBaud);
			usart_set_databits(USART1, 8);
			usart_set_stopbits(USART1, USART_STOPBITS_1);
			usart_set_parity(USART1, USART_PARITY_NONE);
			usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
			usart_set_mode(USART1, USART_MODE_TX_RX);

			/* Enable USART1 Receive interrupt. */
			USART_CR1(USART1) |= USART_CR1_RXNEIE;
//			usart_enable_rx_dma(USART1);
//			usart_enable_tx_dma(USART1);
			/* Finally enable the USART. */
			usart_enable(USART1);

			/* Everything is ok. */
			lReturn = pdPASS;
		}
		else if( ulPort == 1 )
		{
			/* Create the queue of chars that are waiting to be sent to COM1. */
			if (!xCharsForTx[1]) {
			xCharsForTx[ 1 ] = xQueueCreate( serTX_QUEUE_LEN, sizeof( char ) );
			}

			if (!xRxedChars[1]) {
			/* Create the queue used to hold characters received from COM0. */
			xRxedChars[ 1 ] = xQueueCreate( serRX_QUEUE_LEN, sizeof( char ) );
			}
			/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
			rcc_periph_clock_enable(RCC_GPIOA);
			rcc_periph_clock_enable(RCC_AFIO);
			rcc_periph_clock_enable(RCC_USART2);

			/* Enable the USART2 interrupt. */
			nvic_enable_irq(NVIC_USART2_IRQ);
			nvic_set_priority(NVIC_USART2_IRQ, priority);

			/* Setup GPIO pin GPIO_USART2_RE_TX on GPIO port B for transmit. */
			gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
				      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

			/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
			gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
				      GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

			/* Setup UART parameters. */
			usart_set_baudrate(USART2, ulWantedBaud);
			usart_set_databits(USART2, 8);
			usart_set_stopbits(USART2, USART_STOPBITS_1);
			usart_set_parity(USART2, USART_PARITY_NONE);
			usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
			usart_set_mode(USART2, USART_MODE_TX_RX);

			/* Enable USART1 Receive interrupt. */
			USART_CR1(USART2) |= USART_CR1_RXNEIE;

//			usart_enable_rx_dma(USART2);
//			usart_enable_tx_dma(USART2);

			/* Finally enable the USART. */
			usart_enable(USART2);

			/* Everything is ok. */
			lReturn = pdPASS;
		}	
		else
		{
			/* Nothing to do unless more than two ports are supported. */
		}
	}
	
	return lReturn;
}
/*-----------------------------------------------------------*/

signed long xSerialGetChar( long lPort, signed char *pcRxedChar, portTickType xBlockTime )
{
long lReturn = pdFAIL;

	if( lPort < serNUM_COM_PORTS ) 
	{
		if( xQueueReceive( xRxedChars[ lPort ], pcRxedChar, xBlockTime ) == pdPASS )
		{
			lReturn = pdPASS;
		}
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

long lSerialPutString( long lPort, const char * const pcString, unsigned long ulStringLength )
{
long lReturn;
unsigned long ul;

	if( lPort < serNUM_COM_PORTS )
	{
		lReturn = pdPASS;

		for( ul = 0; ul < ulStringLength; ul++ )
		{
			if( xQueueSend( xCharsForTx[ lPort ], &( pcString[ ul ] ), serPUT_STRING_CHAR_DELAY ) != pdPASS )
			{
				/* Cannot fit any more in the queue.  Try turning the Tx on to 
				clear some space. */
				usart_enable_tx_interrupt(xUSART[lPort]);
				vTaskDelay( serPUT_STRING_CHAR_DELAY );
				ul--;
				/* Go back and try again. */
				continue;
			}
		}

		usart_enable_tx_interrupt(xUSART[lPort]);
	}
	else
	{
		lReturn = pdFAIL;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

signed long xSerialPutChar( long lPort, signed char cOutChar, portTickType xBlockTime )
{
long lReturn;

	if( xQueueSend( xCharsForTx[ lPort ], &cOutChar, xBlockTime ) == pdPASS )
	{
		lReturn = pdPASS;
		usart_enable_tx_interrupt(xUSART[lPort]);
	}
	else
	{
		lReturn = pdFAIL;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

void usart1_isr(void)
{
long xHigherPriorityTaskWoken = pdFALSE;
char cChar;

	if( usart_get_flag(USART1, USART_SR_TXE) == true)
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( xCharsForTx[ 0 ], &cChar, &xHigherPriorityTaskWoken ) )
		{
			/* A character was retrieved from the buffer so can be sent to the
			THR now. */
			usart_send(USART1, (uint8_t) cChar);
		}
		else
		{
			usart_disable_tx_interrupt(USART1);
		}		
	}
	
	if( usart_get_flag(USART1, USART_SR_RXNE) == true)
	{
		cChar = (char) usart_recv(USART1);
		xQueueSendFromISR( xRxedChars[ 0 ], &cChar, &xHigherPriorityTaskWoken );
	}	
	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

void usart2_isr(void)
{

	long xHigherPriorityTaskWoken = pdFALSE;
	char cChar;

		if( usart_get_flag(USART2, USART_SR_TXE) == true)
		{
			/* The interrupt was caused by the THR becoming empty.  Are there any
			more characters to transmit? */
			if( xQueueReceiveFromISR( xCharsForTx[ 1 ], &cChar, &xHigherPriorityTaskWoken ) )
			{
				/* A character was retrieved from the buffer so can be sent to the THR now. */
				usart_send(USART2, (uint8_t) cChar);
			}
			else
			{
				usart_disable_tx_interrupt(USART2);
			}
		}

		if( usart_get_flag(USART2, USART_SR_RXNE) == true)
		{
			cChar = (char) usart_recv(USART2);
			xQueueSendFromISR( xRxedChars[ 1 ], &cChar, &xHigherPriorityTaskWoken );
		}

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}
