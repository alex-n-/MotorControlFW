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
#include <stdio.h>

#include "config.h"
#ifdef USE_SERIAL_COMMUNICATION

#include BOARD_BOARD_HEADER_FILE

#ifdef USE_LED
#include BOARD_LED_HEADER_FILE
#endif /* USE_LED */

#include TMPM_UART_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#ifdef USE_RGB_LED
#include BOARD_RGB_LED_HEADER_FILE
#endif /* USE_RGB_LED */    

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "action.h"
#include "protocol.h"
#include "crc8.h"
#include "debug.h"
#include "board.h"

__packed struct header
{
  uint8_t xon;
  uint8_t cmd;
};

__packed struct footer
{
  uint8_t crc;
  uint8_t xoff;
};

static uint8_t          g_q[MAX_SIZE_Q];
static uint8_t          g_a[sizeof(struct header) + MAX_SIZE_A + sizeof(struct footer)];

enum rx_state { rx_idle = 0, rx_cmd, rx_rcv, rx_checksum, rx_eot, };

struct rx_machine
{
  enum rx_state state;
  xSemaphoreHandle s;
  uint8_t cmd;
  uint8_t checksum;
  uint8_t *buf;
  int cnt;
  const struct rpc *f;
};

static struct rx_machine g_rx_machine;

enum tx_state { tx_cmd = 0, tx_send, tx_checksum, tx_eot, tx_idle };

struct tx_machine
{
  enum tx_state state;
  xSemaphoreHandle s;
  uint8_t cmd;
  uint8_t checksum;
  uint8_t *buf;
  int len;
  int cnt;
};

static struct tx_machine g_tx_machine;

struct rpc
{
  int (*f) (void*q, void*a);
  int size_q;
  int size_a;
};

#define MAKE_FUNC(ff) \
  { \
    (int(*) (void*, void*)) ff, sizeof(struct ff ## _q), sizeof(struct ff ## _a), \
  }

/* Setup the function names that should be called on the different actions */
static const struct rpc functions[] = { MAKE_FUNC(StartMotor),
                                  MAKE_FUNC(StopMotor),
                                  MAKE_FUNC(SetMotorParameter),
                                  MAKE_FUNC(GetMotorParameter),
                                  MAKE_FUNC(SetPIControl),
                                  MAKE_FUNC(GetPIControl),
                                  MAKE_FUNC(GetChannelDependand),
                                  MAKE_FUNC(SetSystemDependand),
                                  MAKE_FUNC(GetSystemDependand),
                                  MAKE_FUNC(SetMotorSet),
                                  MAKE_FUNC(GetMotorState),
                                  MAKE_FUNC(StoreParameters),
                                  MAKE_FUNC(ClearParameters),
                                  MAKE_FUNC(ConfigureUARTSpeed),
                                  MAKE_FUNC(GetDSOSnaphot),
                                  MAKE_FUNC(ConfigDSOLog),
                                  MAKE_FUNC(GetDSOLogData),
                                  MAKE_FUNC(GetSystemLoad),
                                  MAKE_FUNC(GetBoardInfo),
                                  MAKE_FUNC(ConfigureHsDSO),
                                  MAKE_FUNC(GetFWVersion),
                                  MAKE_FUNC(DoTurn),
                                  MAKE_FUNC(GetTurnNumber),
                                  MAKE_FUNC(GetTemperature),
                                  MAKE_FUNC(GetErrorState),
                                  MAKE_FUNC(SetChannelDependand),
                                  MAKE_FUNC(GetDCLinkVoltage),
                                };


#define NUM_RPC_FUNCTIONS (sizeof(functions)/sizeof(functions[0]))

static void tx_state_machine(void)
{
  struct tx_machine *tx = &g_tx_machine;

  NVIC_ClearPendingIRQ(SERIAL_COMMUNICATION_TX_IRQ);

  switch(tx->state) {
  case tx_cmd:
    UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, tx->cmd);
    tx->state = tx_send;
    tprintf("tx cmd 0x%02x\n", tx->cmd);
    break;
  case tx_send:
    UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, tx->buf[tx->cnt]);
    tprintf("tx buf 0x%02x @ %d\n", tx->buf[tx->cnt], tx->cnt);
    tx->cnt++;
    if(tx->cnt == tx->len) {
      tx->state = tx_checksum;
      break;
    }
    break;
  case tx_checksum:
    UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, tx->checksum);
    tprintf("tx cs 0x%02x\n", tx->checksum);
    tx->state = tx_eot;
    break;
  case tx_eot:
    UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, 0x13);
    tprintf("tx eot\n");
    tx->state = tx_idle;
    break;
  case tx_idle:
    {
      portBASE_TYPE xHigherPriorityTaskWoken;
      tprintf("tx idle\n");
      NVIC_DisableIRQ(SERIAL_COMMUNICATION_TX_IRQ);
      NVIC_EnableIRQ(SERIAL_COMMUNICATION_RX_IRQ);
      xSemaphoreGiveFromISR(tx->s, &xHigherPriorityTaskWoken);
      break;
    }
  default:
    tprintf("tx error\n");
    tx->state = tx_idle;
    break;
  }
}

#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_suppress = Pe550
#elif defined __KEIL__
#pragma diag_suppress = 550
#endif
static int send_generic (int cmd, uint8_t *buf, int len)
{
  struct tx_machine *tx = &g_tx_machine;
  WorkState w;

  w = UART_GetBufState(SERIAL_COMMUNICATION_CHANNEL, UART_TX);
  assert_param(w == DONE);
 
  tx->cmd = cmd;
  tx->buf = buf;
  tx->cnt = 0;
  tx->len = len;
  tx->state = tx_cmd;
  tx->checksum = crc_update(cmd, buf, len);

  NVIC_EnableIRQ(SERIAL_COMMUNICATION_TX_IRQ);

  UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, 0x11);
  xSemaphoreTake(tx->s, portMAX_DELAY);

  return 0;
}
#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_default = Pe550
#elif defined __KEIL__
#endif

static int send_error (int ret)
{
  uint8_t buf = ret;
  return send_generic(0xee, &buf, sizeof(buf));
}

static void rx_state_machine(void)
{
  uint8_t b = UART_GetRxData(SERIAL_COMMUNICATION_CHANNEL);
  struct rx_machine *rx = &g_rx_machine;
  
  NVIC_ClearPendingIRQ(SERIAL_COMMUNICATION_RX_IRQ);

  rprintf("rx %02x\n", b);
  
  switch(rx->state) {
  case rx_idle:
    if (b == 0x11) {
      rx->state = rx_cmd;
      break;
    }
    rprintf("rx bot\n");
    break;
  case rx_cmd:
    if (b >= NUM_RPC_FUNCTIONS) {
      rx->state = rx_idle;
      rprintf("rx cmd out of bounds\n");
      break;
    }
    rx->cmd = b;  
    rx->f = &functions[rx->cmd];
    rx->state = rx_rcv;
    rx->cnt = 0;
    rx->buf = &g_q[0];
    break;
  case rx_rcv:
    rx->buf[rx->cnt] = b;
    rx->cnt++;
    if (rx->cnt < rx->f->size_q)
      break;
    rx->state = rx_checksum;
    break;
  case rx_checksum:
    rx->checksum = b;  
    rx->state = rx_eot;  
    break;
  case rx_eot:
    rx->state = rx_idle;

    if (b == 0x13) {
      portBASE_TYPE xHigherPriorityTaskWoken;
      NVIC_DisableIRQ(SERIAL_COMMUNICATION_RX_IRQ);
      xSemaphoreGiveFromISR(rx->s, &xHigherPriorityTaskWoken);
      break;
    }
    rprintf("rx eot\n");
    break;
  default:
    /* something bad has happenend */
    assert_param(0);
    break;
  }
}

/*! \brief  IRQ Handler RX Channel
  *
  * @param  None
  * @retval Success
*/
void SERIAL_COMMUNICATION_RX_HANDLER (void)
{
  rx_state_machine();
}

/*! \brief  IRQ Handler TX Channel
  *
  * @param  None
  * @retval Success
*/
void SERIAL_COMMUNICATION_TX_HANDLER (void)
{
  tx_state_machine();
}

static const UART_InitTypeDef uartdefault = 
{
  115200,                                  /* Baud Rate */
  UART_DATA_BITS_8,                        /* Data Bits */
  UART_STOP_BITS_1,                        /* Stop Bits */
  UART_NO_PARITY,                          /* Parity */
  UART_ENABLE_RX | UART_ENABLE_TX,         /* Mode */
  UART_NONE_FLOW_CTRL,                     /* Flow COntrol */
};

static const GPIO_InitTypeDef portConfigTX =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

static const GPIO_InitTypeDef portConfigRX =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

/*! \brief  Initialize the UART
  *
  * @param  None
  * @retval None
*/
static void uart_init (void)
{
  GPIO_Init(SERIAL_COMMUNICATION_PORT,                                          /* Set up TX Port */
            SERIAL_COMMUNICATION_TX,
            &portConfigTX);
  GPIO_Init(SERIAL_COMMUNICATION_PORT,                                          /* Set up RX Port */
            SERIAL_COMMUNICATION_RX,
            &portConfigRX);

  GPIO_EnableFuncReg(SERIAL_COMMUNICATION_PORT,                                 /* Enable UART on the Port Pins */
                     SERIAL_COMMUNICATION_FUNCTION_REGISTER,
                     SERIAL_COMMUNICATION_TX | SERIAL_COMMUNICATION_RX);

  UART_SWReset(SERIAL_COMMUNICATION_CHANNEL);                                   /* Do a SW reset to clear out any crap */

  /* Configure Port for 115200 bps */
  UART_Enable(SERIAL_COMMUNICATION_CHANNEL);
  UART_Init(SERIAL_COMMUNICATION_CHANNEL, &uartdefault);
}

/*! \brief  Initialize the Protocol
  *
  * @param  None
  * @retval Success
*/
static int protocol_init (void)
{
  struct rx_machine *rx = &g_rx_machine;
  struct tx_machine *tx = &g_tx_machine;

  vSemaphoreCreateBinary(rx->s);                                                /* create semaphore for receive */
  vSemaphoreCreateBinary(tx->s);                                                /* create semaphore for transmit */
  xSemaphoreTake(rx->s, 0);                                                     /* take semaphore for receive  directly that task has to wait */
  xSemaphoreTake(tx->s, 0);                                                     /* take semaphore for transmit directly that task has to wait */

  uart_init();                                                                  /* initialize the uart */

  return 0;
}

/*! \brief  Handle Question / Answer 
  *
  * Handle one Question or Answer from Communication Protocol
  *
  * @param  None
  * @retval Success
*/
static int protocol_handle_one_q_a (void)
{
  struct rx_machine *rx = &g_rx_machine;
//  struct tx_machine *tx = &g_tx_machine;

  const struct rpc *f;
  int ret;
  uint8_t checksum;

  /* enable to be sure */  
  NVIC_EnableIRQ(SERIAL_COMMUNICATION_RX_IRQ);

  ret = xSemaphoreTake(rx->s, 1000 / portTICK_RATE_MS);
  if(ret == pdFALSE) {
    rprintf("timeout @ xSemaphoreTake() @ RX\n");
    return -1;
  }
  
  checksum = crc_update(rx->cmd, rx->buf, rx->cnt);
  if (checksum != rx->checksum) {
        dprintf("rx cs mismatch\n");
        send_error(ret);
        return -1;
  }

  f = rx->f;

  ret = f->f(rx->buf, &g_a[0]);
  if(ret)
  {
    dprintf("function %d failed @ error %d\n", rx->cmd, ret);
    send_error(ret);
    return -1;
  }

  /* send the regular answer */
  ret = send_generic(rx->cmd, &g_a[0], f->size_a);
  if(ret)
  {
    dprintf("send_generic() failed @ error %d\n", ret);
    return -1;
  }
  
  return 0;
}

#ifdef USE_BLUETOOTH2
#include <string.h>

static int g_bt_disable_discoverability;

static int bt_send_receive(char *s, char *r)
{
  int i;

  for (i = 0; i < strlen(s); i++) {
    /* wait until tx buffer is empty */
    while(!(SERIAL_COMMUNICATION_CHANNEL->MOD2 & 0x80))
      ;
    UART_SetTxData(SERIAL_COMMUNICATION_CHANNEL, s[i]);
  }
  
  for (i = 0; i < strlen(r); i++) {
    uint8_t c;
    /* wait until rx buffer is full */
    while(!(SERIAL_COMMUNICATION_CHANNEL->MOD2 & 0x40))
      ;
    c = UART_GetRxData(SERIAL_COMMUNICATION_CHANNEL);
    if (c != r[i]) {
      dprintf("mismatch @ %d\n", i);
      goto error;
    }
  }
  return 0;
  
error:
  dprintf("receive failed\n");
  /* fixme: flush the rx buffer? */
  return -1;
}

static int bt_set_discoverability(int enable)
{
  int ret = 0;

  NVIC_DisableIRQ(SERIAL_COMMUNICATION_RX_IRQ);

  if (enable) {
    g_bt_disable_discoverability = 60;
  }

#define ACTION_ENTER_CMD_MODE "$$$"
#define ACTION_EXIT_CMD_MODE "---\n\r"
#define ACTION_WAKE "W\n\r"
#define SET_ENABLE_INQUIRY_SCAN_WINDOW "SI,0200\n\r"
#define SET_DISABLE_INQUIRY_SCAN_WINDOW "SI,0000\n\r"

  ret = bt_send_receive(ACTION_ENTER_CMD_MODE, "CMD\r\n");
  if (ret) {
    dprintf("bt_send_receive() @ ACTION_ENTER_CMD_MODE failed\n");
    goto out;
  }

  if (enable) {
    ret = bt_send_receive(SET_ENABLE_INQUIRY_SCAN_WINDOW, "AOK\r\n");
  } else {
    ret = bt_send_receive(SET_DISABLE_INQUIRY_SCAN_WINDOW, "AOK\r\n");
  }
  if (ret) {
    dprintf("bt_send_receive() @ SET_xxx_INQUIRY_SCAN_WINDOW failed\n");
    goto out;
  }

  ret = bt_send_receive(ACTION_WAKE, "Wake\r\n");
  if (ret) {
    dprintf("bt_send_receive() @ ACTION_WAKE failed\n");
    goto out;
  }

  ret = bt_send_receive(ACTION_EXIT_CMD_MODE, "END\r\n");
  if (ret) {
    dprintf("bt_send_receive() @ ACTION_EXIT_CMD_MODE failed\n");
    goto out;
  }
  
out:
  NVIC_ClearPendingIRQ(SERIAL_COMMUNICATION_RX_IRQ);
  NVIC_ClearPendingIRQ(SERIAL_COMMUNICATION_TX_IRQ);
  
  dprintf("enable %d, ret %d\n", enable, ret);
  return ret;
}
#endif

/*! \brief  System Load Worker Task
  *
  * @param  pvParameters: None
  * @retval None
*/
void ProtocolTask( void *pvParameters )
{
  while (INIT_Done==0)                                                          /* Wailt until HW setup has finished */
    vTaskDelay( 100 / portTICK_RATE_MS );
  
  protocol_init();

#ifdef USE_BLUETOOTH2
  vTaskDelay( 1000 / portTICK_RATE_MS );
  bt_set_discoverability(1);
#endif    
    
  for( ;; )
  {
    protocol_handle_one_q_a();                                                  /* Handle one question or answer */
#ifdef USE_LED
    LED_Toggle(LED_SIGNAL_SERIAL_COMMUNICATION_RUNNING);                        /* Tollgle LED for signal working to outside */
#endif
#ifdef USE_RGB_LED
    RGB_LED_ToggleValue(LED_RGB_GREEN);
#endif /* USE_RGB_LED */ 
    
#ifdef USE_BLUETOOTH2
    if (g_bt_disable_discoverability > 0) {
      g_bt_disable_discoverability--;
    }
    else if (g_bt_disable_discoverability == 0) {
      g_bt_disable_discoverability--;
      bt_set_discoverability(0);
    }
#endif    
  }
}

#endif
