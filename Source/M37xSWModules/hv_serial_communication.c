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

#ifdef USE_HV_COMMUNICATION

#include "tmpm370_tmrb.h"
#include "tmpm370_gpio.h"

#include "hv_serial_communication.h"

#define BAUD_RATE     9600
#define NR_OF_BITS    8
#define BUFFER_SIZE   6
#define TIMER_COUNTER 40000000/BAUD_RATE/3

#define SYNC_BYTE0 0xa5
#define SYNC_BYTE1 0x5a

static TMRB_InitTypeDef timerTMBRConfig =
{
  TMRB_INTERVAL_TIMER,
  TMRB_CLK_DIV_2,
  TIMER_COUNTER,
  TMRB_AUTO_CLEAR,
  TIMER_COUNTER,
};

static const GPIO_InitTypeDef portConfigHV_RX =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

static unsigned int   received_adc_value[2];
static unsigned char  buffer[BUFFER_SIZE];
static unsigned char  qin = 0;

static unsigned char check_transmission = 0;
static unsigned char waiting_for_stop_bit = 0;
static unsigned char transmitted_rx_bit;
static unsigned char rx_ready = 0;
static unsigned char rx_ctr;
static unsigned char bits_left_transmission;
static unsigned char internal_rx_buffer;

void INTTB00_IRQHandler(void)
{
  char  start_bit, flag_in;

  if ( waiting_for_stop_bit )
  {
    if ( --rx_ctr<=0 )
    {
      waiting_for_stop_bit = FALSE;
      rx_ready = FALSE;
      internal_rx_buffer &= 0xFF;
      
      if ((qin>=2) && (qin<BUFFER_SIZE))
        buffer[qin++] = internal_rx_buffer;
      if ((internal_rx_buffer==SYNC_BYTE0) && (qin==0))
        buffer[qin++] = internal_rx_buffer;
      if ((internal_rx_buffer==SYNC_BYTE1) && (qin==1))
        buffer[qin++] = internal_rx_buffer;
      
      if (qin==BUFFER_SIZE)
      {
        qin=0;
        check_transmission=1;
      }
      
    }
  }
  else                      // rx_test_busy
  {
    if ( rx_ready==FALSE )
    {
      start_bit = GPIO_ReadDataBit(GPIO_PK,GPIO_BIT_1);
      if ( start_bit==0 )     // Test for Start Bit
      {
        rx_ready = 1;
        internal_rx_buffer = 0;
        rx_ctr = 4;
        bits_left_transmission = NR_OF_BITS;
        transmitted_rx_bit = 1;
      }
    }
    else  // rx_busy
    {
      if ( --rx_ctr<=0 )
      {   // rcv
        rx_ctr = 3;
        flag_in = GPIO_ReadDataBit(GPIO_PK,GPIO_BIT_1);
        if ( flag_in )
          internal_rx_buffer |= transmitted_rx_bit;
        transmitted_rx_bit <<= 1;
        if ( --bits_left_transmission<=0 )
          waiting_for_stop_bit = 1;
      }
    }
  }
  
  if (check_transmission==1)
  {
    check_transmission=0;
    if ( (buffer[2]==(buffer[4]^0xff))
      && (buffer[3]==(buffer[5]^0xff)) )
      received_adc_value[buffer[2]>>7]=((buffer[2]&0x3)<<8)|buffer[3];
  }
}

uint16_t HV_Communication_GetValue(uint8_t channel_number)
{
  assert_param(channel_number>1);
  
  return received_adc_value[channel_number];
}

void HV_SerialCommunicationInit( void )
{
  GPIO_Init(GPIO_PK,
            GPIO_BIT_1,
            &portConfigHV_RX);

  TMRB_Enable(TSB_TB0);
  TMRB_Init(TSB_TB0, &timerTMBRConfig);
  NVIC_EnableIRQ(INTTB00_IRQn);
  TMRB_SetRunState(TSB_TB0, TMRB_RUN);
}

#endif /* USE_HV_COMMUNICATION */
