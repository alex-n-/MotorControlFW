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

#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include TMPM_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE
#include TMPM_ADC_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "eeprom.h"
#include "spi.h"
#include "motorctrl.h"
#include "debug.h"


static void eeprom_latch_in(void)
{
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, ENABLE);                     /* Deselect CS */
  vTaskDelay( 1/ portTICK_RATE_MS);                                             /* Wait for 1 ms */
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, DISABLE);                    /* Select CS */
}


/*! \brief  Detect Board Revision
  *
  * Revision 1 doesn't have an EEProm
  *
  * @retval Version Number of Board
*/
uint8_t BOARD_Detect_Revision(void)
{
  unsigned char status;
  
  SPI_SelectDevice(SPI_DEVICE_EEPROM);                                          /* Select SPI Device  (block access for other drivers) */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WREN);                                /* Send write enable to eeprom */
  eeprom_latch_in();                                                            /* Latch data into eeprom */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRITE);                               /* Send write command */
  eeprom_latch_in();                                                            /* Latch data into eeprom */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_RDSR);                                /* Read out status register */
  status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);                                  /* read out data */
  status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  eeprom_latch_in();

  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRDI);                                /* Disable write enable */

  SPI_DeselectDevice();                                                         /* Give SPI access back for other Tasks */
  
  return (status==STATUS_WEL?1:2);                                              /* If there is an EEProm it's revision 2 otherwise 1 */  
}
