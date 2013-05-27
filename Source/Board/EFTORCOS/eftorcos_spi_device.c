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

#include TMPM_GPIO_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#include BOARD_SPI_HEADER_FILE

#include "spi.h"

static xSemaphoreHandle SPI_ChannelMutex;

/*! \brief SPI Get Channel Access
  *
  * Lock the access to the SPI bus
  *
  * @retval None
*/
static void GetChannelAccess(void)
{
  xSemaphoreTake(SPI_ChannelMutex, 1000);
}

/*! \brief SPI Release Channel Access
  *
  * Free the access to the SPI bus
  *
  * @retval None
*/
static void ReleaseChannelAccess(void)
{
  xSemaphoreGive(SPI_ChannelMutex);
}

/* Setup the CS on PA7 */
static const GPIO_InitTypeDef GPIO_Init_Struct_CS =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,                                                          /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,                                                      /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,                                                        /* Enable/disable pull down */
};

/*! \brief Select SPI device
  *
  * Set the CS signal for an SPI device
  *    
  * @param  device:  Device to be set
  *
  * @retval None
*/
void SPI_SelectDevice(SPI_DeviceSelect device)
{
  GetChannelAccess();
  switch (device) 
  {
  case SPI_DEVICE_CAN:  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_2, DISABLE); /* CAN */
  break;
  case  SPI_DEVICE_AMP_V_I_PH_U: GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_6, DISABLE); /* PGAs */
  break;
  case SPI_DEVICE_AMP_V_I_PH_V: GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_6, DISABLE); /* PGAs */
  break;
  case SPI_DEVICE_AMP_V_I_PH_W: GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_6, DISABLE); /* PGAs */
  break;
  }
}

/*! \brief Deselect SPI device
  *
  * @retval None
*/
void SPI_DeselectDevice(void)
{
  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_2, ENABLE); /* CAN */
  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_6, ENABLE); /* PGAs */
  ReleaseChannelAccess();
}

/*! \brief SPI Transmit Byte
  *
  * Transmit Byte via SPI
  *
  * @param  SPIx: SPI Channel
  * @param  byte: byte to transmit
  *
  * @retval None
*/
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

/*! \brief SPI Receive Byte
  *
  * Receive Byte via SPI
  *
  * @param  byte: byte to transmit
  *
  * @retval received byte
*/
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

/*! \brief SPI Transmit Bytes
  *
  * Transmit Bytes via SPI
  *
  * @param  SPIx:   SPI Channel
  * @param  bytes:  pointer to bytes
  * @param  number: number of bytes to transmit
  *
  * @retval none
*/
void SPI_TransmitBytes(TSB_SC_TypeDef* SPIx, uint8_t* bytes, uint8_t number)
{
  uint8_t   i;
  WorkState tmp;

  for (i=0;i<number;i++)
  {
    SPI_SetTxData(BOARD_SPI_CHANNEL, *bytes++);
    SPI_GetRxData(BOARD_SPI_CHANNEL);
  }

  /* Wait until Data is send */
  tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
  while(tmp == BUSY)
    tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
}

/*! \brief SPI Receive Bytes
  *
  * Receive Bytes via SPI
  *
  * @param  SPIx:   SPI Channel
  * @param  bytes:  pointer to bytes storage
  * @param  number: number of bytes to receive
  *
  * @retval none
*/
void SPI_ReceiveBytes(TSB_SC_TypeDef* SPIx, uint8_t* bytes, uint8_t number)
{
  uint8_t   i;
  WorkState tmp;

  for(i=0;i<number;i++)
  {
    SPI_SetTxData(BOARD_SPI_CHANNEL, 0x00);
    *bytes++=SPI_GetRxData(BOARD_SPI_CHANNEL);
  }
  /* Wait until Data is send */
  tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
  while(tmp == BUSY)
    tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);

  return;
}

/*! \brief SPI Init
  *
  * Initalize the SPI bus
  *
  * @param  spi_speed: speed of SPI bus
  *
  * @retval none
*/
void SPI_DeviceInit(uint32_t spi_speed)
{
  SPI_InitTypeDef   SPI_InitValues;

  SPI_InitValues.speed = spi_speed;
  SPI_InitValues.mode  = SPI_ENABLE_RX | SPI_ENABLE_TX;

  GPIO_SetOutput(GPIO_PA, GPIO_BIT_4 | GPIO_BIT_5);
  GPIO_SetInput(GPIO_PA, GPIO_BIT_6);
  GPIO_SetOutputEnableReg(GPIO_PA, GPIO_BIT_4 | GPIO_BIT_5, ENABLE);
  GPIO_EnableFuncReg(GPIO_PA, GPIO_FUNC_REG_1, GPIO_BIT_4 | GPIO_BIT_5 | GPIO_BIT_6);

  SPI_Enable(BOARD_SPI_CHANNEL);
  SPI_Init(BOARD_SPI_CHANNEL, &SPI_InitValues);

  GPIO_Init(GPIO_PE, GPIO_BIT_2 | GPIO_BIT_6, &GPIO_Init_Struct_CS);    /* CAN / PGAs */
  
  SPI_ChannelMutex = xSemaphoreCreateMutex();
}
