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
 * fsmc_func.c
 *
 *  Created on: 28 сент. 2014 г.
 *      Author: eric
 */

#include "fsmc_func.h"
#include <libopencm3/stm32/fsmc.h>

/**
  * @brief  Initializes the FSMC NOR/SRAM Banks according to the specified
  *         parameters in the FSMC_NORSRAMInitStruct.
  * @param  FSMC_NORSRAMInitStruct : pointer to a FSMC_NORSRAMInitTypeDef
  *         structure that contains the configuration information for
  *        the FSMC NOR/SRAM specified Banks.
  * @retval None
  */
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct)
{
  /* Check the parameters */
//  IS_FSMC_NORSRAM_BANK(FSMC_NORSRAMInitStruct->FSMC_Bank);
//  IS_FSMC_MUX(FSMC_NORSRAMInitStruct->FSMC_DataAddressMux);
//  IS_FSMC_MEMORY(FSMC_NORSRAMInitStruct->FSMC_MemoryType);
//  IS_FSMC_MEMORY_WIDTH(FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth);
//  IS_FSMC_BURSTMODE(FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode);
//  IS_FSMC_ASYNWAIT(FSMC_NORSRAMInitStruct->FSMC_AsynchronousWait);
//  IS_FSMC_WAIT_POLARITY(FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity);
//  IS_FSMC_WRAP_MODE(FSMC_NORSRAMInitStruct->FSMC_WrapMode);
//  IS_FSMC_WAIT_SIGNAL_ACTIVE(FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive);
//  IS_FSMC_WRITE_OPERATION(FSMC_NORSRAMInitStruct->FSMC_WriteOperation);
//  IS_FSMC_WAITE_SIGNAL(FSMC_NORSRAMInitStruct->FSMC_WaitSignal);
//  IS_FSMC_EXTENDED_MODE(FSMC_NORSRAMInitStruct->FSMC_ExtendedMode);
//  IS_FSMC_WRITE_BURST(FSMC_NORSRAMInitStruct->FSMC_WriteBurst);
//  IS_FSMC_ADDRESS_SETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime);
//  IS_FSMC_ADDRESS_HOLD_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime);
//  IS_FSMC_DATASETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime);
//  IS_FSMC_TURNAROUND_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration);
//  IS_FSMC_CLK_DIV(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision);
//  IS_FSMC_DATA_LATENCY(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency);
//  IS_FSMC_ACCESS_MODE(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode);

  /* Bank1 NOR/SRAM control register configuration */
//  FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank] =
//            (uint32_t)FSMC_NORSRAMInitStruct->FSMC_DataAddressMux |
//            FSMC_NORSRAMInitStruct->FSMC_MemoryType |
//            FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth |
//            FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode |
//            FSMC_NORSRAMInitStruct->FSMC_AsynchronousWait |
//            FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity |
//            FSMC_NORSRAMInitStruct->FSMC_WrapMode |
//            FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive |
//            FSMC_NORSRAMInitStruct->FSMC_WriteOperation |
//            FSMC_NORSRAMInitStruct->FSMC_WaitSignal |
//            FSMC_NORSRAMInitStruct->FSMC_ExtendedMode |
//            FSMC_NORSRAMInitStruct->FSMC_WriteBurst;

  FSMC_BCR(FSMC_NORSRAMInitStruct->FSMC_Bank) =
          (uint32_t)FSMC_NORSRAMInitStruct->FSMC_DataAddressMux |
          FSMC_NORSRAMInitStruct->FSMC_MemoryType |
          FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth |
          FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode |
          FSMC_NORSRAMInitStruct->FSMC_AsynchronousWait |
          FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity |
          FSMC_NORSRAMInitStruct->FSMC_WrapMode |
          FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive |
          FSMC_NORSRAMInitStruct->FSMC_WriteOperation |
          FSMC_NORSRAMInitStruct->FSMC_WaitSignal |
          FSMC_NORSRAMInitStruct->FSMC_ExtendedMode |
          FSMC_NORSRAMInitStruct->FSMC_WriteBurst;

  if(FSMC_NORSRAMInitStruct->FSMC_MemoryType == FSMC_MemoryType_NOR)
  {
//    FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank] |= (uint32_t)BCR_FACCEN_Set;
    FSMC_BCR(FSMC_NORSRAMInitStruct->FSMC_Bank) |= (uint32_t)BCR_FACCEN_Set;
  }

  /* Bank1 NOR/SRAM timing register configuration */
//  FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank+1] =
//            (uint32_t)FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime |
//            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime << 4) |
//            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime << 8) |
//            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration << 16) |
//            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision << 20) |
//            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency << 24) |
//             FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode;
  FSMC_BCR(FSMC_NORSRAMInitStruct->FSMC_Bank+1) =
		  (uint32_t)FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime |
          (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime << 4) |
          (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime << 8) |
          (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration << 16) |
          (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision << 20) |
          (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency << 24) |
           FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode;


  /* Bank1 NOR/SRAM timing register for write configuration, if extended mode is used */
  if(FSMC_NORSRAMInitStruct->FSMC_ExtendedMode == FSMC_ExtendedMode_Enable)
  {
//    assert_param(IS_FSMC_ADDRESS_SETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime));
//    assert_param(IS_FSMC_ADDRESS_HOLD_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime));
//    assert_param(IS_FSMC_DATASETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime));
//    assert_param(IS_FSMC_CLK_DIV(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision));
//    assert_param(IS_FSMC_DATA_LATENCY(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency));
//    assert_param(IS_FSMC_ACCESS_MODE(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode));

//    FSMC_Bank1E->BWTR[FSMC_NORSRAMInitStruct->FSMC_Bank] =
//              (uint32_t)FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime |
//              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime << 4 )|
//              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime << 8) |
//              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision << 20) |
//              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency << 24) |
//               FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode;
    FSMC_BWTR(FSMC_NORSRAMInitStruct->FSMC_Bank) =
            (uint32_t)FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime |
            (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime << 4 )|
            (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime << 8) |
            (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision << 20) |
            (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency << 24) |
             FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode;
  }
  else
  {
//    FSMC_Bank1E->BWTR[FSMC_NORSRAMInitStruct->FSMC_Bank] = 0x0FFFFFFF;
    FSMC_BWTR(FSMC_NORSRAMInitStruct->FSMC_Bank) = 0x0FFFFFFF;
  }
}

/**
  * @brief  Enables or disables the specified NOR/SRAM Memory Bank.
  * @param  FSMC_Bank: specifies the FSMC Bank to be used
  *   This parameter can be one of the following values:
  *     @arg FSMC_Bank1_NORSRAM1: FSMC Bank1 NOR/SRAM1
  *     @arg FSMC_Bank1_NORSRAM2: FSMC Bank1 NOR/SRAM2
  *     @arg FSMC_Bank1_NORSRAM3: FSMC Bank1 NOR/SRAM3
  *     @arg FSMC_Bank1_NORSRAM4: FSMC Bank1 NOR/SRAM4
  * @param  NewState: new state of the FSMC_Bank. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FSMC_NORSRAMCmd(uint32_t FSMC_Bank, FunctionalState NewState)
{
//  assert_param(IS_FSMC_NORSRAM_BANK(FSMC_Bank));
//  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected NOR/SRAM Bank by setting the PBKEN bit in the BCRx register */
//    FSMC_Bank1->BTCR[FSMC_Bank] |= BCR_MBKEN_Set;
	  FSMC_BCR(FSMC_Bank) |= BCR_MBKEN_Set;
  }
  else
  {
    /* Disable the selected NOR/SRAM Bank by clearing the PBKEN bit in the BCRx register */
//    FSMC_Bank1->BTCR[FSMC_Bank] &= BCR_MBKEN_Reset;
	  FSMC_BCR(FSMC_Bank) |= BCR_MBKEN_Reset;
  }
}

