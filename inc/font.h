#include "../inc/arm_comm.h"

#ifndef __FONT_H
#define __FONT_H

typedef struct _FontType_t {
  Int32U H_Size;
  Int32U V_Size;
  Int32U CharacterOffset;
  Int32U CharactersNuber;
  pInt8U pFontStream;
  pInt8U pFontDesc;
} FontType_t, *pFontType_t;

extern const unsigned char TextStream0[];

#endif // __FONT_H
