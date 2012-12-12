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
#ifdef USE_HSDSO

#include BOARD_BOARD_HEADER_FILE
#include TMPM_UART_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#include "debug.h"
#include "hsdso.h"
#include "dso.h"
#include "crc8.h"
#include "ve.h"
#include "motorctrl.h"

static int is_configured = 0;
static int is_enabled = 0;
static int in_progress = 0;

#define MAX_NUM_ENTRIES 32
#define NUM_SIZE_HEADER 5 /* Start-Byte, Checksum, Event, Timestamp */
#define SIZE_XFER_ARRAY (NUM_SIZE_HEADER+(MAX_NUM_ENTRIES*2))
static uint8_t g_xfer_buf[SIZE_XFER_ARRAY];
static uint8_t g_capture_buf[(MAX_NUM_ENTRIES*2)];
static int g_xfer_offset;
static int16_t *g_log_ptr;
static uint32_t g_selected_values = 0;
static int g_count = 0;
static int g_missed = 0;
static uint16_t g_timestamp;
static int g_PWMFrequency;

static const GPIO_InitTypeDef portConfigTX =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

static const UART_InitTypeDef uartdefault =
{
  115200,                                                                       /* Baud Rate */
  UART_DATA_BITS_8,                                                             /* Data Bits */
  UART_STOP_BITS_1,                                                             /* Stop Bits */
  UART_NO_PARITY,                                                               /* Parity */
  UART_ENABLE_TX,                                                               /* Mode */
  UART_NONE_FLOW_CTRL,                                                          /* Flow COntrol */
};

static void configure_hsdso(void)
{
  GPIO_Init(HSDSO_PORT, HSDSO_TX, &portConfigTX);
  GPIO_EnableFuncReg(HSDSO_PORT, HSDSO_FUNC_REG, HSDSO_TX);
    
  UART_SWReset(HSDSO_CHANNEL);
  UART_Enable(HSDSO_CHANNEL);
  UART_Init(HSDSO_CHANNEL, &uartdefault);

  /* custom configuration for 5Mbps config using fsys */
  {
    uint32_t tmp = 0U;
    tmp = HSDSO_CHANNEL->MOD0;
    tmp &= ~0x3;
    tmp |= 0x2;
    HSDSO_CHANNEL->MOD0 = tmp;
  }

  /* select half-duplex tx */
  {
    uint32_t tmp = 0U;
    tmp = HSDSO_CHANNEL->MOD1;
    tmp &= ~((1 << 5) | (1 << 6));
    tmp |= (1 << 6);
    HSDSO_CHANNEL->MOD1 = tmp;
  }
  
  /* configure & enable fifo */
  {
    HSDSO_CHANNEL->TFC  = (1 << 6);
    HSDSO_CHANNEL->FCNF = (1 << 3) | (1 << 0);
  }
}

void HSDSO_TX_IRQ_HANDLER(void)
{
  assert_param(is_enabled);
  
  while(1) {
 
    /* check if fifo is full, if so exit */
    if ((HSDSO_CHANNEL->TST & 0x7) == 0x4)
      break;

    /* check if we have transferred all bytes */
    if (g_xfer_offset >= (NUM_SIZE_HEADER+(g_count*2))) {
      /* signal end of transfer when fifo has underrun */
      if ((HSDSO_CHANNEL->TST & 0x7) == 0x0) {
        NVIC_DisableIRQ(HSDSO_TX_IRQ);
        in_progress = 0;
      }
      break;
    }

    UART_SetTxData(HSDSO_CHANNEL, g_xfer_buf[g_xfer_offset++]);
  }
}

int HsDSO_Disable(int motor_nr)
{
  if (!is_enabled) {
    dprintf("motor_nr %d already disabled\n", motor_nr);
    return 0;
  }
  
  NVIC_DisableIRQ(HSDSO_TX_IRQ);
  is_enabled = 0;
  in_progress = 0;
  
  dprintf("motor_nr %d\n", motor_nr);
  return 0;
}

int HsDSO_Enable(int motor_nr, uint32_t selected_values, uint8_t spread_factor)
{
  uint32_t cnt;
  
  if (!is_configured) {
    is_configured = 1;
    configure_hsdso();
  }

  if (!selected_values) {
    dprintf("motor_nr %d, !selected_values\n", motor_nr);
    return -1;   
  }

  g_selected_values = selected_values;
  g_count = 0;

  for (cnt = g_selected_values; cnt; cnt >>= 1UL)
    g_count += cnt & 1;

  if (g_count > MAX_NUM_ENTRIES) {
    dprintf("motor_nr %d, g_count %d > MAX_NUM_ENTRIES\n", motor_nr, g_count);
    return -1;   
  }
  
  g_log_ptr = NULL;
  g_PWMFrequency = SystemValues[motor_nr].PWMFrequency;
  
  is_enabled = 1;
  
  /* emtpy FIFO */
  HSDSO_CHANNEL->TFC |= (1 << 7);
    
  dprintf("motor_nr %d, g_selected_values 0x%08lx, g_count %d\n", motor_nr, g_selected_values, g_count);
  return 0;
}

#define LOG_IF_SELECTED(reg, check, shift) \
        if (g_selected_values & (uint32_t)check) \
          *pos++ = reg >> shift;

void HsDSO_Log_PMD(int offset)
{
  int16_t *pos;
  
  if (!is_enabled)
    return;

  pos = (int16_t *)&g_capture_buf[0];
  LOG_IF_SELECTED(TEE_VEC->TMPREG0, Va,     17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG1, Vb,     17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG2, Vc,     17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG3, Valpha, 16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG4, Vbeta,  16);
  g_log_ptr = pos;
}

void HsDSO_Log_VE(int offset)
{
  TEE_VE_TypeDef* pVEx = TEE_VE0;
  int16_t *pos;

  g_timestamp = (g_timestamp + 1) % g_PWMFrequency;
  
  if (!is_enabled)
    return;

  if (!g_log_ptr)
    return;

  if (in_progress) {
    g_missed++;
    return;
  }
  
  if (offset)
    pVEx = TEE_VE1;

  pos = g_log_ptr;
  g_log_ptr = NULL;

  LOG_IF_SELECTED(pVEx->ID,                     Id,         16);
  LOG_IF_SELECTED(pVEx->IDREF,                  Id_Ref,      0);
  LOG_IF_SELECTED(pVEx->CIDKP,                  Id_kp,       0);
  LOG_IF_SELECTED(pVEx->CIDKI,                  Id_ki,       0);
  LOG_IF_SELECTED(pVEx->IQ,                     Iq,         16);
  LOG_IF_SELECTED(pVEx->IQREF,                  Iq_Ref,      0);
  LOG_IF_SELECTED(pVEx->CIQKP,                  Iq_kp,       0);
  LOG_IF_SELECTED(pVEx->CIQKI,                  Iq_ki,       0);
  LOG_IF_SELECTED(pVEx->VD,                     Vd,         16);
  LOG_IF_SELECTED(pVEx->VQ,                     Vq,         16);
  LOG_IF_SELECTED(pVEx->VDIH,                   Vdi,        16);
  LOG_IF_SELECTED(pVEx->VQIH,                   Vqi,        16);
  LOG_IF_SELECTED(pVEx->THETA,                  Theta,       1);
  LOG_IF_SELECTED(VE_Omega[offset].part.reg,    Omega,       0);
  LOG_IF_SELECTED(pVEx->SIN,                    SIN_Theta,   0);
  LOG_IF_SELECTED(pVEx->COS,                    COS_Theta,   0);
  LOG_IF_SELECTED(pVEx->SECTOR,                 Sector,      0);
  LOG_IF_SELECTED(pVEx->VDC,                    VDC,         0);
  LOG_IF_SELECTED(TEE_VEC->TMPREG0,             Ia,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG1,             Ib,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG2,             Ic,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG3,             Ialpha,     16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG4,             Ibeta,      16);
  LOG_IF_SELECTED(VE_Id_reference[offset],      SW_Id_Ref,   0);
  LOG_IF_SELECTED(VE_Iq_reference[offset],      SW_Iq_Ref,   0);
  LOG_IF_SELECTED(VE_Id[offset],                SW_Id,       0);
  LOG_IF_SELECTED(VE_Iq[offset],                SW_Iq,       0);

  g_xfer_buf[0] = 0x11;
  g_xfer_buf[2] = g_missed & 0xff;
  g_xfer_buf[3] = g_timestamp & 0xff;
  g_xfer_buf[4] = (g_timestamp >> 8) & 0xff;
  memcpy(&g_xfer_buf[5], g_capture_buf, g_count*2);
  g_xfer_buf[1] = crc_update(0x00, &g_xfer_buf[2], 3+(g_count*2));
  g_missed = 0;
  g_xfer_offset = 0;
  in_progress = 1;

  UART_SetTxData(HSDSO_CHANNEL, g_xfer_buf[g_xfer_offset++]);
  NVIC_EnableIRQ(HSDSO_TX_IRQ);
}

#endif
