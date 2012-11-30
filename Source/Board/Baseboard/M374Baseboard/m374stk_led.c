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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifdef USE_LED
#include BOARD_LED_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#include "board.h"

static xSemaphoreHandle LEDSemaphore;

/*! \brief LED Toggle
  *
  * Toggle an LED
  *
  * @param  led:  LED Number
  *
  * @retval None
*/
void LED_Toggle(uint8_t led)
{
  uint8_t status;

  if (BoardRevision == 1)                                                       /* Don't use on Revision 1 Boards */
    return;

  xSemaphoreTake(LEDSemaphore, 1000);
  status = SPI_GetLEDStatus();
  status = (status & (0x0f &~(1 << led))) | (~status & (1 << led));
  SPI_SetLEDStatus(status);
  xSemaphoreGive(LEDSemaphore);
}

/*! \brief LED set state
  *
  * Set the state of an LED
  *
  * @param  led:   LED Number
  * @param  state: State to set
  *
  * @retval None
*/
void LED_SetState(uint8_t led, uint8_t state)
{
  uint8_t status;

  if (BoardRevision == 1)                                                       /* Don't use on Revision 1 Boards */
    return;

  xSemaphoreTake(LEDSemaphore, 1000);
  status = SPI_GetLEDStatus();
  status = (status & (0x0f &~(1 << led))) | (state << led);
  SPI_SetLEDStatus(status);
  xSemaphoreGive(LEDSemaphore);
}

/*! \brief LED get state
  *
  * Get the state of an LED
  *
  * @param  led:   LED Number
  *
  * @retval actual state
*/
uint8_t LED_GetState(uint8_t led)
{
  uint8_t status;

  if (BoardRevision == 1)                                                       /* Don't use on Revision 1 Boards */
    return 0;

  xSemaphoreTake(LEDSemaphore, 1000);
  status = SPI_GetLEDStatus();
  status = status & (1 << led);
  xSemaphoreGive(LEDSemaphore);
  return(status != 0);
}

/*! \brief LED Init
  *
  * Initialize the LED access
  *
  * @retval None
*/
void LED_Init(void)
{
  if (BoardRevision == 1)                                                       /* Don't use on Revision 1 Boards */
    return;
  LEDSemaphore = xSemaphoreCreateMutex();
}

#endif /* USE_LED */
