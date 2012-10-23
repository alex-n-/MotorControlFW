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
#ifdef BOARD_HITEX_M370

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include TMPM_GPIO_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include BOARD_LCD_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "spi.h"

static unsigned long    spi_speed;

static xSemaphoreHandle SPI_ChannelMutex;

static void GetChannelAccess(void)
{
  xSemaphoreTake(SPI_ChannelMutex, 1000);
}

static void ReleaseChannelAccess(void)
{
  xSemaphoreGive(SPI_ChannelMutex);
}

/* For LCD its just PE0-7 for the data as output */
static const GPIO_InitTypeDef GPIO_Init_Struct_LCD =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

/* Setup the CS on PA7 */
static const GPIO_InitTypeDef GPIO_Init_Struct_CS =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

void SPI_SelectDevice(SPI_DeviceSelect device)
{
  SPI_InitTypeDef   SPI_InitValues;

  SPI_InitValues.speed = spi_speed;
  SPI_InitValues.mode  = SPI_ENABLE_RX | SPI_ENABLE_TX;
  /* Block concurrent access by semaphore */ 
  GetChannelAccess();

  /*
   * As eeprom and LCD is shared on Port E on the Hitex Board, we have to route the
   * LCD access through the SPI Device Layer (bad board design)
   */
  switch(device)
  {
  case SPI_DEVICE_EEPROM:

    /* Setup the SPI stuff on PE0-2 */
    GPIO_SetOutput          (GPIO_PE,  GPIO_BIT_0
                                     | GPIO_BIT_2);
    GPIO_SetInput           (GPIO_PE,  GPIO_BIT_1);
    GPIO_SetOutputEnableReg (GPIO_PE,  GPIO_BIT_0
                                     | GPIO_BIT_2, ENABLE);
    GPIO_EnableFuncReg      (GPIO_PE, GPIO_FUNC_REG_1,  GPIO_BIT_0
                                                      | GPIO_BIT_1
                                                      | GPIO_BIT_2);

    SPI_Enable(BOARD_SPI_CHANNEL);
    SPI_Init(BOARD_SPI_CHANNEL, &SPI_InitValues);

    GPIO_Init(GPIO_PA, GPIO_BIT_7, &GPIO_Init_Struct_CS);

    /* CS is low active */
    GPIO_WriteDataBit(GPIO_PA, GPIO_BIT_7, DISABLE);

    break;

  case SPI_DEVICE_LCD:
    GPIO_Init(LCD_DATA_PINS, LCD_DATA_MASK, &GPIO_Init_Struct_LCD);
    GPIO_Init(LCD_INSTRUCTION_PINS, LCD_INSTRUCTION_MASK, &GPIO_Init_Struct_LCD);
    break;
  }
}

void SPI_DeselectDevice(void)
{
  /*
   * For deselecting it's enought to reverse the special settings for the SPI, as
   * the Port will completely resetup when accesse
   */
  GPIO_DisableFuncReg(GPIO_PE, GPIO_FUNC_REG_1,  GPIO_BIT_0
                                               | GPIO_BIT_1 
                                               | GPIO_BIT_2);

  /* CS of eeprom to high */
  GPIO_WriteDataBit(GPIO_PA, GPIO_BIT_7, ENABLE);
  ReleaseChannelAccess();
}

void SPI_TransmitByte(TSB_SC_TypeDef* SPIx, uint8_t byte)
{
  WorkState tmp;

  SPI_SetTxData(BOARD_SPI_CHANNEL, byte);
  SPI_GetRxData(BOARD_SPI_CHANNEL);

  /* Wait until Data is send */
  tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
  while(tmp == BUSY)
    tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
}

uint8_t SPI_ReceiveByte(TSB_SC_TypeDef* SPIx)
{
  WorkState tmp;

  SPI_SetTxData(BOARD_SPI_CHANNEL, 0x00);
  /* Wait until Data is send */
  tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
  while(tmp == BUSY)
    tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);

  return SPI_GetRxData(BOARD_SPI_CHANNEL);
}

void SPI_DeviceInit(uint32_t speed)
{
  SPI_ChannelMutex = xSemaphoreCreateMutex();

  /*
   * Just stor the spi speed for further usage, as the access is totally different
   * to the M374STK board
   */
  spi_speed = speed;
}

#endif
