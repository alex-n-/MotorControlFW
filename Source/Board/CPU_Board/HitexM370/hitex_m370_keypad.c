/* THE SOURCE CODE AND ITS
 * RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA CORPORATION MAKES NO OTHER
 * WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR, STATUTORY AND DISCLAIMS ANY
 * AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY, SATISFACTORY QUALITY, NON
 * INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. THE SOURCE CODE AND
 * DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION RESERVES THE RIGHT TO
 * INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER REVISIONS OF IT, AND TO
 * MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR THE PRODUCTS OR
 * TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME. TOSHIBA CORPORATION SHALL NOT BE
 * LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGE OR LIABILITY ARISING
 * FROM YOUR USE OF THE SOURCE CODE OR ANY DOCUMENTATION, INCLUDING BUT NOT
 * LIMITED TO, LOST REVENUES, DATA OR PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL
 * OR CONSEQUENTIAL NATURE, PUNITIVE DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS
 * ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF
 * ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM
 * FOR SUCH DAMAGE IS BASED UPON WARRANTY, CONTRACT, TORT, NEGLIGENCE OR
 * OTHERWISE. (C)Copyright TOSHIBA CORPORATION 2011 All rights reserved
 */

#include "config.h"
#if (defined BOARD_HITEX_M370 && defined USE_KEYPAD)

#include TMPM_GPIO_HEADER_FILE
#include BOARD_KEYPAD_HEADER_FILE

/*
 =======================================================================================================================
 =======================================================================================================================
 */

/* GPIO settings for Keypad purpose */
static const GPIO_InitTypeDef GPIO_Init_Struct =
{
  GPIO_INPUT_MODE,          /* Enable Keypad as input */
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

void KEYPAD_Init(void)
{
  GPIO_Init(GPIO_PH,
            GPIO_BIT_2 |
            GPIO_BIT_3 |
            GPIO_BIT_4 |
            GPIO_BIT_5 |
            GPIO_BIT_6 |
            GPIO_BIT_7,   &GPIO_Init_Struct);
}

/*
 =======================================================================================================================
 =======================================================================================================================
 */
unsigned char KEYPAD_GetState(void)
{
  return ((~(GPIO_ReadData(GPIO_PH) >> 2)) & 0x3f);     /* Lower 2 Bits are used for LED */
}

#endif
