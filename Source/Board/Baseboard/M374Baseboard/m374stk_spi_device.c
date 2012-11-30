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

static void GetChannelAccess(void)
{
  xSemaphoreTake(SPI_ChannelMutex, 1000);
}

static void ReleaseChannelAccess(void)
{
  xSemaphoreGive(SPI_ChannelMutex);
}

/* As we cant read back the actual values we have to save them */
static uint8_t  shift_reg_value = 0;

static void SPI_UpdateShiftRegister(void)
{
  WorkState tmp;

  /* Put BOARD_SPI_CHANNEL_ALE low to not disturb any devices */
  GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_0, DISABLE);

  /* Send data to serial shift */
  SPI_SetTxData(BOARD_SPI_CHANNEL, shift_reg_value);

  /*
   * Dummy read out the RX Buffer - otherwise the transfer will stall when the RX
   * buffer is full
   */
  SPI_GetRxData(BOARD_SPI_CHANNEL);
  /* Wait until Data is send */
  tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);
  while(tmp == BUSY)
    tmp = SPI_GetBufState(BOARD_SPI_CHANNEL, SPI_TX);

  /* Timing is hard at the border - so wait additionally 1 ms */
  vTaskDelay(1 / portTICK_RATE_MS);

  /*
   * Latch in data ;
   * Put BOARD_SPI_CHANNEL_ALE high to latch in data
   */
  GPIO_WriteDataBit(GPIO_PF, GPIO_BIT_0, ENABLE);
}

uint8_t SPI_GetLEDStatus(void)
{
  return shift_reg_value >> 4;
}

void SPI_SetLEDStatus(uint8_t status)
{
  GetChannelAccess();
  shift_reg_value = (shift_reg_value & 0x0f) | status << 4;
  SPI_UpdateShiftRegister();
  ReleaseChannelAccess();
}

void SPI_SelectDevice(SPI_DeviceSelect device)
{
  GetChannelAccess();
  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_2, ENABLE);
  shift_reg_value = (shift_reg_value & 0xf0) | device;
  SPI_UpdateShiftRegister();
  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_2, DISABLE);
}

void SPI_DeselectDevice(void)
{
  GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_2, ENABLE);
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

/* GPIO settings for LED purpose */
static const GPIO_InitTypeDef GPIO_Init_Struct =
{
  GPIO_OUTPUT_MODE,         /* Enable LEDs as output */
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

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

  GPIO_Init(GPIO_PF, GPIO_BIT_0, &GPIO_Init_Struct);    /* CS for Shift Register */
  GPIO_Init(GPIO_PE, GPIO_BIT_2, &GPIO_Init_Struct);    /* SS for 3/8 Decoder */

  SPI_ChannelMutex = xSemaphoreCreateMutex();
}
