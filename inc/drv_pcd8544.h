//#include "stm32f10x_lib.h"
#include "../inc/arm_comm.h"
#include "font.h"


#ifndef  __DRV_PCD8544_H
#define  __DRV_PCD8544_H

#define X_PIX_SIZE    84
#define Y_PIX_SIZE    48

#define MAX_SPI_FREQ  (2 MHZ)
extern FontType_t Lucida_Console_6_8_6;

#define FONT      Lucida_Console_6_8_6
/*************************************************************************
 * Function Name: PCD8544_PowerUp
 * Parameters: const Int8U * pData
 *
 * Return: none
 *
 * Description: Power up initialization
 *
 *************************************************************************/
void PCD8544_PowerUp (const Int8U * pData);

/*************************************************************************
 * Function Name: PCD8544_StrShow
 * Parameters: Int8U X, Int8U Y, const Int8U * pData
 *
 * Return: none
 *
 * Description: Show zero terminate string
 *
 *************************************************************************/
void PCD8544_StrShow (Int8U X, Int8U Y, const Int8U * pData);

/*************************************************************************
 * Function Name: PCD8544_LoadData
 * Parameters: Int8U X, Int8U Y, Int32U Size, const Int8U * pData
 *
 * Return: none
 *
 * Description: Load raw data in the LDC data memory
 *
 *************************************************************************/
void PCD8544_LoadData (Int8U X, Int8U Y, Int32U Size, const Int8U * pData);

/*************************************************************************
 * Function Name: PCD8544_SetAddr
 * Parameters: Int8U X, Int8U Y
 *
 * Return: none
 *
 * Description: Set X and Y address of RAM
 *
 *************************************************************************/
//void PCD8544_SetAddr (Int8U X, Int8U Y);
//static void PCD8544_SetAddr (Int8U X, Int8U Y);

#endif  /* __DRV_PCD8544_H */
