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
#if (defined BOARD_HITEX_M370 && defined USE_LED)

#include "FreeRTOS.h"
#include "task.h"

#include TMPM_GPIO_HEADER_FILE
#include BOARD_LED_HEADER_FILE


  /* GPIO settings for LED purpose */
static const GPIO_InitTypeDef GPIO_Init_Struct =
{
  GPIO_OUTPUT_MODE,         /* Enable LEDs as output */
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

void LED_Init(void)
{
  int i;

  GPIO_Init(GPIO_PF, GPIO_BIT_0 | GPIO_BIT_1, &GPIO_Init_Struct);
  GPIO_Init(GPIO_PH, GPIO_BIT_0 | GPIO_BIT_1, &GPIO_Init_Struct);
  
  /* Switch all leds off by default (init is on) */
  for (i=0;i<4;i++)
    LED_SetState(i,LED_OFF);
}

void LED_Toggle(unsigned char led)
{
  switch(led)
  {
  case LED_NO_0:
    GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_0, (~GPIO_ReadDataBit(GPIO_PF, GPIO_BIT_0) & 0x01));
    break;
  case LED_NO_1:
    GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_1, (~GPIO_ReadDataBit(GPIO_PF, GPIO_BIT_1) & 0x01));
    break;
  case LED_NO_2:
    GPIO_WriteDataBit(GPIO_PH, GPIO_BIT_0, (~GPIO_ReadDataBit(GPIO_PH, GPIO_BIT_0) & 0x01));
    break;
  case LED_NO_3:
    GPIO_WriteDataBit(GPIO_PH, GPIO_BIT_1, (~GPIO_ReadDataBit(GPIO_PH, GPIO_BIT_1) & 0x01));
    break;
  }
}

void LED_SetState(unsigned char led, unsigned char state)
{
  switch(led)
  {
  case LED_NO_0:
    GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_0, (~state)&0x01 );
    break;
  case LED_NO_1:
    GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_1, (~state)&0x01);
    break;
  case LED_NO_2:
    GPIO_WriteDataBit(GPIO_PH, GPIO_BIT_0, (~state)&0x01);
    break;
  case LED_NO_3:
    GPIO_WriteDataBit(GPIO_PH, GPIO_BIT_1, (~state)&0x01);
    break;
  }
}

unsigned char LED_GetState(unsigned char led)
{
  unsigned char state = 0;

  switch(led)
  {
  case LED_NO_0:
    state = ~GPIO_ReadDataBit(GPIO_PF, GPIO_BIT_0);
    break;
  case LED_NO_1:
    state = ~GPIO_ReadDataBit(GPIO_PF, GPIO_BIT_1);
    break;
  case LED_NO_2:
    state = ~GPIO_ReadDataBit(GPIO_PH, GPIO_BIT_0);
    break;
  case LED_NO_3:
    state = ~GPIO_ReadDataBit(GPIO_PH, GPIO_BIT_1);
    break;
  }
  return state;
}
#endif
