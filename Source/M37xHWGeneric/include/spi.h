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

#ifndef _SPI_H_
#define _SPI_H_

#include "tx03_common.h"
#include "config.h"
#include TMPM_HEADER_FILE
/*
 * @addtogroup TX03_Periph_Driver @{ ;
 * @addtogroup SPI @{ ;
 * @defgroup SPI_Exported_Types @{
 */
#define EN_SIOE_SET         ((uint32_t) 0x00000001)
#define EN_SIOE_CLEAR       ((uint32_t) 0x00000000)
#define CR_PARITY_MASK      ((uint32_t) 0x0000009F)
#define CR_ERROR_MASK       ((uint32_t) 0x0000001C)
#define CR_OERR_FLAG        ((uint8_t) 0x10)
#define CR_PERR_FLAG        ((uint8_t) 0x08)
#define CR_FERR_FLAG        ((uint8_t) 0x04)
#define MOD0_CTSE_MASK      ((uint32_t) 0x000000BF)
#define MOD0_RXE_CLEAR      ((uint32_t) 0x000000DF)
#define MOD0_WU_SET         ((uint32_t) 0x00000010)
#define MOD0_WU_CLEAR       ((uint32_t) 0x000000EF)
#define MOD0_SM_MASK        ((uint32_t) 0x000000F3)
#define MOD0_SC_MASK        ((uint32_t) 0x000000FC)
#define MOD0_SC_BRG         ((uint32_t) 0x00000001)
#define MOD1_I2SC_SET       ((uint32_t) 0x00000080)
#define MOD1_I2SC_CLEAR     ((uint32_t) 0x0000007F)
#define MOD1_TXE_CLEAR      ((uint32_t) 0x000000EF)
#define MOD1_CLEAR          ((uint32_t) 0x000000FE)
#define MOD1_HALF_DUPLEX_RX ((uint32_t) 0x00000020)
#define MOD1_HALF_DUPLEX_TX ((uint32_t) 0x00000040)
#define MOD1_FULL_DUPLEX    ((uint32_t) 0x00000060)
#define MOD2_BUF_MASK       ((uint32_t) 0x000000C0)
#define MOD2_TBEMP_FLAG     ((uint8_t) 0x80)
#define MOD2_RBFLL_FLAG     ((uint8_t) 0x40)
#define MOD2_SBLEN_MASK     ((uint32_t) 0x000000EF)
#define MOD2_DRCHG_MASK     ((uint32_t) 0x000000F7)
#define MOD2_DRCHG_SET      ((uint32_t) 0x00000008)
#define MOD2_WBUF_SET       ((uint32_t) 0x00000004)
#define MOD2_SWRST_MASK     ((uint32_t) 0x000000FC)
#define MOD2_SWRST_CMD1     ((uint32_t) 0x00000002)
#define MOD2_SWRST_CMD2     ((uint32_t) 0x00000001)
#define BRCR_BRADDE_SET     ((uint32_t) 0x00000040)
#define BRCR_BRCK_MASK      ((uint32_t) 0x000000CF)
#define BRCR_BRS_MASK       ((uint32_t) 0x000000F0)
#define BRCR_CLEAR          ((uint32_t) 0x0000007F)
#define BRADD_BRK_MASK      ((uint32_t) 0x00000000)
#define FCNF_TFIE_SET       ((uint32_t) 0x00000008)
#define FCNF_TFIE_CLEAR     ((uint32_t) 0x00000017)
#define FCNF_RFIE_SET       ((uint32_t) 0x00000004)
#define FCNF_RFIE_CLEAR     ((uint32_t) 0x0000001B)
#define FCNF_RXTXCNT_SET    ((uint32_t) 0x00000002)
#define FCNF_RXTXCNT_CLEAR  ((uint32_t) 0x0000001D)
#define FCNF_CNFG_SET       ((uint32_t) 0x00000001)
#define FCNF_CNFG_CLEAR     ((uint32_t) 0x0000001E)
#define TRXFC_TRXFCS_SET    ((uint32_t) 0x00000080)
#define TRXT_TU_ROR_MASK    ((uint32_t) 0x00000080)
#define TRXT_TRLV_MASK      ((uint32_t) 0x00000007)
#define TRXST_FIFO_EMPTY    ((uint32_t) 0x00000000)
#define TRXST_FIFO_1B       ((uint32_t) 0x00000001)
#define TRXST_FIFO_2B       ((uint32_t) 0x00000002)
#define TRXST_FIFO_3B       ((uint32_t) 0x00000003)
#define TRXST_FIFO_4B       ((uint32_t) 0x00000004)


typedef enum
{
  SPI_20_MHZ = 20000000,
  SPI_10_MHZ = 10000000,
  SPI_5_MHZ  =  5000000,
  SPI_1_MHZ  =  1000000,
}SPI_SPEED;
  
/*
 -----------------------------------------------------------------------------------------------------------------------
  @brief SPI Init Structure definition
 -----------------------------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint32_t  speed;
  uint32_t  mode;                                                               /* !< This member configures the SPI communication speed. */
} SPI_InitTypeDef;

/*
 * @} ;
 * End of group SPI_Exported_Types ;
 * @defgroup SPI_Exported_Constants @{
 */
#define SPI0                  TSB_SC0
#define SPI1                  TSB_SC1
#ifdef __TMPM_370__
#define SPI2                  TSB_SC2
#endif
#define SPI3                  TSB_SC3
#ifdef __TMPM_370__
#define IS_SPI_PERIPH(param)  (((param) == SPI0) || ((param) == SPI1) || ((param) == SPI2) || ((param) == SPI3))
#endif
#ifdef __TMPM_372__
#define IS_SPI_PERIPH(param)  (((param) == SPI0) || ((param) == SPI1) || ((param) == SPI3))
#endif
#ifdef __TMPM_373__
#define IS_SPI_PERIPH(param)  (((param) == SPI0) || ((param) == SPI1) || ((param) == SPI3))
#endif
#ifdef __TMPM_374__
#define IS_SPI_PERIPH(param)  (((param) == SPI0) || ((param) == SPI1) || ((param) == SPI3))
#endif

#define SPI_ENABLE_RX ((uint32_t) 0x00000020)
#define SPI_ENABLE_TX ((uint32_t) 0x00000010)
#define IS_SPI_MODE(param) \
    (((param) == SPI_ENABLE_RX) || ((param) == SPI_ENABLE_TX) || ((param) == (SPI_ENABLE_TX | SPI_ENABLE_RX)))
#define SPI_RX            ((uint32_t) 0x00000020)
#define SPI_TX            ((uint32_t) 0x00000040)
#define IS_SPI_TRX(param) (((param) == SPI_RX) || ((param) == SPI_TX))


typedef enum
{
  SPI_NO_ERR      = 0U,
  SPI_OVERRUN     = 1U,
  SPI_PARITY_ERR  = 2U,
  SPI_FRAMING_ERR = 3U,
  SPI_ERRS        = 4U
} SPI_Err;

void      SPI_Enable      (TSB_SC_TypeDef* SPIx);
void      SPI_Disable     (TSB_SC_TypeDef* SPIx);
WorkState SPI_GetBufState (TSB_SC_TypeDef* SPIx, uint8_t Direction);
void      SPI_SWReset     (TSB_SC_TypeDef* SPIx);
void      SPI_Init        (TSB_SC_TypeDef* SPIx, SPI_InitTypeDef* InitStruct);
uint8_t   SPI_GetRxData   (TSB_SC_TypeDef* SPIx);
void      SPI_SetTxData   (TSB_SC_TypeDef* SPIx, uint32_t Data);
SPI_Err   SPI_GetErrState (TSB_SC_TypeDef* SPIx);

#endif /* _SPI_H_ */
