/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


/* Define memory regions. Uncomment one: */
/* STM32F10X_HD */

  MEMORY
  {
 	 rom (rx)  : ORIGIN = 0x08000000, LENGTH = 512K-0x8000   	
 	 ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K   		
 	 EEMUL (rwx) : ORIGIN = 0x08000000+126K, LENGTH = 2K
 	 }
 

/* STM32F10X_MD */
/*
MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K-0x4000   	
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K   			
}
*/
/* Include the common ld script. */
INCLUDE libs/libopencm3/lib/libopencm3_stm32f1.ld

