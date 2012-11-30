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

#include "spi.h"
#include "tx03_common.h"
#include "board.h"

/*! \brief  Enable SPI channel
  *
  * @param  SPIx:       channel to disable
  * @retval None
*/
void SPI_Enable(TSB_SC_TypeDef* SPIx)
{
  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */
  SPIx->EN |= EN_SIOE_SET;                                                      /* Set SCxEN<SIOE> to enable SPIx */
}

/*! \brief  Disable SPI channel
  *
  * @param  SPIx:       channel to disable
  * @retval None
*/
void SPI_Disable(TSB_SC_TypeDef* SPIx)
{
  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */
  SPIx->EN &= EN_SIOE_CLEAR;                                                    /* Clear SCxEN<SIOE> to disable SPIx */
}

/*! \brief  Check data available / buffer free 
  *
  * @param  SPIx:       channel to test
  * @param  Direction:  receive or transmit direction
  * @retval Workstate
*/
WorkState SPI_GetBufState(TSB_SC_TypeDef* SPIx, uint8_t Direction)
{
  uint8_t   tmp = 0U;
  WorkState retval = BUSY;

  /* Check the parameters */
  assert_param(IS_SPI_PERIPH(SPIx));
  assert_param(IS_SPI_TRX(Direction));

  tmp = ((uint8_t) (SPIx->MOD2 & MOD2_BUF_MASK));
  switch(Direction)
  {
  case SPI_TX:
    if((tmp & MOD2_TBEMP_FLAG) == MOD2_TBEMP_FLAG)
      /* Return Tx buffer empty if the flag is set */
      retval = DONE;
    else
      /* Do nothing */
    break;

  case SPI_RX:
    if((tmp & MOD2_RBFLL_FLAG) == MOD2_RBFLL_FLAG)
      /* Return Rx buffer full if the flag is set */
      retval = DONE;
    else
      /* Do nothing */
    break;

  default:  /* Do nothing */
    break;
  }

  return retval;
}

/*! \brief  Perform a SW reset of SPI channel
  *
  * @param  SPIx:       channel to reset
  * @retval None
*/
void SPI_SWReset(TSB_SC_TypeDef* SPIx)
{
  uint32_t  tmp = 0U;

  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */

  /* Write software-reset command */
  tmp = SPIx->MOD2;
  tmp &= MOD2_SWRST_MASK;
  SPIx->MOD2 = tmp | MOD2_SWRST_CMD1;
  tmp &= MOD2_SWRST_MASK;
  SPIx->MOD2 = tmp | MOD2_SWRST_CMD2;
}


/*! \brief  Get error state of the SPI channel
  *
  * @param  SPIx:       channel to use
  * @retval Received Data
*/
uint8_t SPI_GetRxData(TSB_SC_TypeDef* SPIx)
{
  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */
  return (SPIx->BUF & 0xFFU);                                                   /* Return received data */
}

/*! \brief  Get error state of the SPI channel
  *
  * @param  SPIx:       channel to use
  * @param  Data:       data to send
  * @retval None
*/
void SPI_SetTxData(TSB_SC_TypeDef* SPIx, uint32_t Data)
{
  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */
  SPIx->BUF = Data & 0xFFU;
}

/*! \brief  Get error state of the SPI channel
  *
  * @param  SPIx:       channel to configure
  * @retval SPI_Err
*/
SPI_Err SPI_GetErrState(TSB_SC_TypeDef* SPIx)
{
  uint8_t tmp = 0U;
  SPI_Err retval = SPI_NO_ERR;

  assert_param(IS_SPI_PERIPH(SPIx));                                            /* Check the parameters */

  tmp = ((uint8_t) (SPIx->CR & CR_ERROR_MASK));
  switch(tmp)
  {
  case CR_OERR_FLAG:                                                            /* Check overrun flag */
    retval = SPI_OVERRUN;
    break;
  case CR_FERR_FLAG:                                                            /* Check framing flag */
    retval = SPI_FRAMING_ERR;
    break;
  default:
    if(tmp != 0U)                                                               /* more than one error */
      retval = SPI_ERRS;
    break;
  }
  return retval;
}

/*! \brief  Initialize the SPI channel
  *
  * @param  SPIx:       channel to configure
  * @param  InitStruct: configuration data for the channel (such as speed)
  * @retval None  
*/
void SPI_Init(TSB_SC_TypeDef* SPIx, SPI_InitTypeDef* InitStruct)
{
  uint32_t tmp = 0;
  /* Get the peripheral I/O clock frequency */
  SPIx->BRCR = T0 / 2 / InitStruct->speed;

  /* Use baud rate generator */
  SPIx->MOD0 |= MOD0_SC_BRG;

  SPIx->MOD2 |= MOD2_WBUF_SET | MOD2_DRCHG_SET;

  tmp = SPIx->MOD1;

  /* Enable or disable transmission or reception */
  switch(InitStruct->mode)
  {
  case SPI_ENABLE_RX:
    SPIx->MOD0 |= SPI_ENABLE_RX;
    tmp &= MOD1_TXE_CLEAR;
    break;

  case SPI_ENABLE_TX:
    tmp |= SPI_ENABLE_RX;
    SPIx->MOD0 &= MOD0_RXE_CLEAR;
    break;

  default:
    SPIx->MOD0 |= SPI_ENABLE_RX;
    tmp |= SPI_ENABLE_TX | MOD1_FULL_DUPLEX;
    break;
  }

  tmp &= MOD1_CLEAR;
  SPIx->MOD1 = tmp;
}
