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

#ifdef USE_LED

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#include TMPM_GPIO_HEADER_FILE
#include BOARD_LED_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

/* GPIO settings for LED purpose */
#ifndef DEBUG
static const GPIO_InitTypeDef GPIO_Init_Struct =
{
  GPIO_OUTPUT_MODE,         /* Enable LEDs as output */
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};
#endif /* DEBUG */

void LED_Init(void)
{
#ifndef DEBUG
  int i;
  
  GPIO_Init(GPIO_PB, GPIO_BIT_3
                   | GPIO_BIT_4
                   | GPIO_BIT_5
                   | GPIO_BIT_6, &GPIO_Init_Struct);
  
  /* Switch all leds off by default (init is on) */
  for (i=0;i<4;i++)
    LED_SetState(i,LED_OFF);
#endif /* DEBUG */  
}

void LED_Toggle(unsigned char led)
{
#ifndef DEBUG  
  switch(led)
  {
  case LED_NO_0:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_3, (~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_3) & 0x01));
    break;
  case LED_NO_1:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_4, (~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_4) & 0x01));
    break;
  case LED_NO_2:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_5, (~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_5) & 0x01));
    break;
  case LED_NO_3:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_6, (~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_6) & 0x01));
    break;
  }
#endif /* DEBUG */
}

void LED_SetState(unsigned char led, unsigned char state)
{
#ifndef DEBUG  
  switch(led)
  {
  case LED_NO_0:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_3, (~state)&0x01 );
    break;
  case LED_NO_1:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_4, (~state)&0x01);
    break;
  case LED_NO_2:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_5, (~state)&0x01);
    break;
  case LED_NO_3:
    GPIO_WriteDataBit(GPIO_PB, GPIO_BIT_6, (~state)&0x01);
    break;
  }
#endif /* DEBUG */  
}

unsigned char LED_GetState(unsigned char led)
{
  unsigned char state = 0;

#ifndef DEBUG  
  switch(led)
  {
  case LED_NO_0:
    state = ~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_3);
    break;
  case LED_NO_1:
    state = ~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_4);
    break;
  case LED_NO_2:
    state = ~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_5);
    break;
  case LED_NO_3:
    state = ~GPIO_ReadDataBit(GPIO_PB, GPIO_BIT_6);
    break;
  }
#endif /* DEBUG */
  
  return state;
}
#endif
