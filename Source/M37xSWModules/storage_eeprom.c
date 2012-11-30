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

#include <string.h>

#include "config.h"

#ifdef USE_CONFIG_STORAGE_EEPROM

#include BOARD_BOARD_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include TMPM_UART_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#include "debug.h"
#include "storage_backend.h"
#include "eeprom.h"
#include "board.h"

#define STORAGE_POSITION 16

/*! \brief  Read EEProm Config
  *
  * Read out stored parameter sets from EEProm and write them to the parameter sets in RAM
  *
  * @param  None
  * @retval Success
*/
int storage_backend_load(uint8_t *buf, int size, int len)
{
  int ret;
  
#ifdef BOARD_HITEX_M370 
  if (BoardRevision == 1) {
    dprintf("board revision == 1 unsupported\n");
    return -1;
  }
#endif  

  if (len > EEPROM_BYTE_SIZE - STORAGE_POSITION) {
    dprintf("len %d > EEPROM_BYTE_SIZE - STORAGE_POSITION\n", len);
    return -1;
  }
  
  ret = EEPROM_Read(STORAGE_POSITION, buf, len);
  if (ret) {
    dprintf("EEPROM_Read() failed\n");
  }

  return ret;
}

/*! \brief  Write Config to EEProm
  *
  * Write the Configuration Parameters from RAM to EEProm
  *
  * @param  channel_number: parameters of which channel to write
  * @retval Success
*/
int storage_backend_save(uint8_t *buf, int size, int len)
{
  int ret;
  
#ifdef BOARD_HITEX_M370 
  if (BoardRevision == 1) {
    dprintf("board revision == 1 unsupported\n");
    return -1;
  }
#endif  

  if (len > EEPROM_BYTE_SIZE - STORAGE_POSITION) {
    dprintf("len %d > EEPROM_BYTE_SIZE - STORAGE_POSITION\n", len);
    return -1;
  }
  
  ret = EEPROM_Write(STORAGE_POSITION, buf, len);
  if (ret) {
    dprintf("EEPROM_Write() failed\n");
  }

  return ret;
}

/*! \brief  Clear EEProm
  *
  * Clear the EEProm content by writing 0xff to all cells
  *
  * @param  None
  * @retval None
*/
int storage_backend_clear(uint8_t *buf, int size)
{
  unsigned int  pos;
  unsigned char clear_page[EEPROM_PAGESIZE];

#ifdef BOARD_HITEX_M370 
  if (BoardRevision == 1) {
    dprintf("board revision == 1 unsupported\n");
    return -1;
  }
#endif  
  
  memset (clear_page,0xff,sizeof(clear_page));                                  /* create array with all 0xff */

  for (pos=0;pos<EEPROM_BYTE_SIZE;pos+=EEPROM_PAGESIZE)                         /* write the clear array to EEProm */
    EEPROM_Write(pos,clear_page,sizeof(clear_page));

  return 0;
}

#endif
