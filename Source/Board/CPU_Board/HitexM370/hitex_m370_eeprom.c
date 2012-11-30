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
#if (defined BOARD_HITEX_M370 && defined USE_CONFIG_STORAGE_EEPROM)

#include "FreeRTOS.h"
#include "task.h"

#include TMPM_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "eeprom.h"
#include "spi.h"

/*! \brief EEProm latch data in
  *
  * @retval None
*/
static void eeprom_latch_in(void)
{
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, ENABLE);                     /* Deselect CS */
  vTaskDelay( 1/ portTICK_RATE_MS);                                             /* Wait for 1 ms */
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, DISABLE);                    /* Select CS */
}

/*! \brief EEProm paged write
  *
  * Write data in paged mode to EEProm
  *
  * @param  pos:  Starting byte in EEProm
  * @param  data: Data to write
  * @param  size: Size of Data 
  *
  * @retval None
*/
static void eeprom_paged_write(uint8_t pos, uint8_t* data, uint8_t size)
{
  unsigned char status, i;

  SPI_SelectDevice(SPI_DEVICE_EEPROM);                                          /* Select SPI Device  (block access for other drivers) */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WREN);                                /* Send write enable to eeprom */
  eeprom_latch_in();                                                            /* Latch data into eeprom */

  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRITE);                               /* Send write command */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, pos);                                     /* transmit start position of data */
  for(i = 0; i < size; i++)                                                     /* write data */
    SPI_TransmitByte(BOARD_SPI_CHANNEL, *data++);

  eeprom_latch_in();                                                            /* Latch data into eeprom */
  
  do                                                                            /* Wait until write has finished */
  {
    SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_RDSR);                              /* Read out status register */
    status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);                                /* read out data */
    status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);
    eeprom_latch_in();

    vTaskDelay(1/ portTICK_RATE_MS);                                            /* Wait 1 ms before ask again */
  } while((status & STATUS_WIP) == STATUS_WIP);                                 /* Wait till Write In Progress is cleared */

  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRDI);                                /* Disable write enable */

  SPI_DeselectDevice();                                                         /* Give SPI access back for other Tasks */
}

/*! \brief EEProm write
  *
  * Write data to EEProm
  *
  * @param  pos:  Starting byte in EEProm
  * @param  data: Data to write
  * @param  size: Size of Data 
  *
  * @retval None
*/
int8_t EEPROM_Write(uint16_t pos, uint8_t* data, uint16_t size)
{
  unsigned char rest_of_first_page,
                full_pages,
                amount_on_first_page,
                amount_on_last_page,
                i;
  
  if((pos + size) > EEPROM_BYTE_SIZE)                                           /* Check for writing over the end of EEProm - prevent overwriting of data */
    return -1;

  /* Calculate byte amounts / full pages for paged write */
  rest_of_first_page = EEPROM_PAGESIZE - pos % EEPROM_PAGESIZE;
  
  if (size <= rest_of_first_page)
    amount_on_first_page=size;
  else
    amount_on_first_page=rest_of_first_page;
  
  size -= amount_on_first_page;
  
  full_pages = (size) / EEPROM_PAGESIZE;

  amount_on_last_page = size - full_pages * EEPROM_PAGESIZE;

  /* Write data to first page to write */
  if(amount_on_first_page > 0)
  {
    eeprom_paged_write(pos, data, amount_on_first_page);
    pos  += rest_of_first_page;
    data += rest_of_first_page;
  }

  /* Write full pages */
  if(full_pages > 0)
  {
    for(i = 0; i < full_pages; i++)
    {
      eeprom_paged_write(pos, data, EEPROM_PAGESIZE);
      pos  += EEPROM_PAGESIZE;
      data += EEPROM_PAGESIZE;
    }
  }
  
  /* Write rest of bytes */
  if(amount_on_last_page > 0)
    eeprom_paged_write(pos, data, amount_on_last_page);

  return 0;
}

/*! \brief EEProm read
  *
  * Read Data from EEProm
  *
  * @param  pos:  Starting byte in EEProm
  * @param  data: Data buffer
  * @param  size: Size of Data to read
  *
  * @retval error code
*/
int8_t EEPROM_Read(uint16_t pos, uint8_t* data, uint16_t size)
{
  unsigned int  i;

  if((pos + size) > EEPROM_BYTE_SIZE)                                           /* Check for reading after end of EEProm - prevent of reading some not wanted data */
    return -1;

  /* Select SPI Device  (block access for other drivers) */
  SPI_SelectDevice(SPI_DEVICE_EEPROM);

  /* Read out data */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_READ);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, pos);
  SPI_ReceiveByte(BOARD_SPI_CHANNEL);                                           /* Dummy read to get buffer in line */

  for(i = 0; i < size; i++)
  {
    *data = SPI_ReceiveByte(BOARD_SPI_CHANNEL);
    data++;
  }

  /* Give SPI access back for other Tasks */
  SPI_DeselectDevice();

  return 0;
}

#endif
