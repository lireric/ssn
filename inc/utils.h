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

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ssn.h"
#include <assert.h>
//#include <errno.h>

/**
 *
 * Generic useful macros.
 */


#ifndef EINVAL
  #define EINVAL	22
#endif

#ifndef M_PI
  #define M_PI		3.14159265358979323846
#endif

#if 0
  /** Packed attribute for structure definitions. */
  #define __packed __attribute__ ((__packed__))
#endif

/** Return the bigger value of the two arguments. */
#define MAX(A,B)	((A)>=(B) ? (A) : (B))

/** Return the smaller value of the two arguments. */
#define MIN(A,B)	((A)<=(B) ? (A) : (B))

/** Return the number of elements in the given array @a A. */
#define ARRAY_NUMELEM(A)	(sizeof((A))/sizeof((A)[0]))


/**
 * Simple helper function for converting hexadecimal digit @a c encoded in
 * ASCII to its binary representation.
 *
 * @param c ASCII HEX character that will be converted.
 *
 * @return Zero or positive value indicating the binary representation of @a c
 * if conversion was successful, negative otherwise.
 */
static inline int hex2bin_nibble(int c)
{
	int bin;

	if((c>='0') && (c<='9')) {
		bin = c - '0';
	} else if((c>='A') && (c<='F')) {
		bin = c - ('A' - 0xA);
	} else if((c>='a') && (c<='f')) {
		bin = c - ('a' - 0xA);
	} else {
		bin = -EINVAL;
	}
	return bin;
}


/**
 * Simple helper function for converting binary 4-bit value @a nb to its
 * ASCII hexadecimal representation.
 *
 * @param nb Binary nibble to convert to HEX. Must be in the range [0;15].
 *
 * @return An ASCII character for the HEX representation of the nibble @a nb.
 *
 * @warning This function DOES NOT return negative values on error. Instead it
 * raises an assertion error on invalid input values. The idea is that when
 * calling this function user will have to strip the needed nibble anyway,
 * so there is no need for runtime error checking here.
 */
static inline int bin2hex_nibble(int nb)
{
	assert((nb>=0) && (nb<=0xf));
	if(nb<=9) {
		return nb + '0';
	} else {
		return nb + ('a'-0xA);
	}
}


/**
 * Simple macro for decoding a nibble to its ASCII HEX representation. Same
 * functionality as @a bin2hex_nibble() but this one can be used for
 * forming constant initialization data.
 *
 * @param nb Data nibble to be converted.
 * @return ASCII HEX representation of @a nb.
 */
#define BIN2HEX_NIBBLE(nb)	(((nb)<=9) ? ((nb)+'0') : ((nb)+'A'-0xa))

/**
 * Calculate CRC32. Code taken from the public domain.
 *
 * @param buf Buffer, holding the bytes which CRC32 will be calculated.
 * @param len Number of bytes in @a buf.
 * @param crc Initial CRC value, typically ~0ul.
 * @return The CRC32 of the given data bytes.
 */
static inline uint32_t crc32_be(uint8_t *buf, unsigned int len, uint32_t crc)
{
	unsigned int i;
	
	while (len--) {
		crc ^= *buf++ << 24;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & 0x80000000) ? 0x04c11db7:0);
	}
	return crc;
}

char* strncpy0(char* dest, const char* src, size_t size);

/**
 * Return the next token, delimited by a character from @a delim.
 * The parameter @a sp is updated to point past the first found delimiter, if
 * found. Otherwise sp is not touched.
 */
extern char *strnext(char **sp, const char *delim);


/** Critical section handling for Cortex M3, courtesy of Stanimir Bonev. */
extern volatile int critical_section_counter;

static inline void enter_critical_section(void)
{
  if(critical_section_counter == 0)
  {
	  __asm__("CPSID i");
  }
  // avoid lost of one count in case of simultaneously calling from both places
  ++critical_section_counter;
}

static inline void exit_critical_section(void)
{
  if(--critical_section_counter == 0)
  {
	  __asm__("CPSIE i");
  }
}

uint32_t get_port_by_name(char* name);
uint32_t get_rcc_by_port(uint32_t nPort);
uint32_t conv2d(const char* p);
uint32_t convHex2d(const char* p);
int32_t	GetNumbersValue(char* pcSrcString);
int32_t	fillCommandStruct(char* pcBuf, sSSNCommand* xSSNCommand);
uint8_t parseCommaString(char* psChannels, uint8_t* pChannelArray, uint8_t nArrayMaxSize);


uint16 ATOI(char* str,uint16 base); 			/* Convert a string to integer number */
uint32 ATOI32(char* str,uint16 base); 			/* Convert a string to integer number */
void itoa(uint16 n,uint8* str, uint8 len);
int ValidATOI(char* str, int base, int* ret); 		/* Verify character string and Convert it to (hexa-)decimal. */
char C2D(u_char c); 					/* Convert a character to HEX */

uint16 swaps(uint16 i);
uint32 swapl(uint32 l);

void replacetochar(char * str, char oldchar, char newchar);

void mid(int8* src, int8* s1, int8* s2, int8* sub);
//void inet_addr_(unsigned char* addr,unsigned char *ip);

#ifdef __cplusplus
}
#endif

#endif	/* UTILS_H */

