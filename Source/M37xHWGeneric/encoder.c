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
#include <stdlib.h>

#include "config.h"
#ifdef USE_ENCODER

#include TMPM_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#include "encoder.h"
#include "motorctrl.h"
#include "ve.h"

EncoderValues                 EncoderData[MAX_CHANNEL];               /*<! generic encoder result data */

static const GPIO_InitTypeDef portConfigENC =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

static void ENC_IOInit(unsigned char channel_number)
{
  GPIO_Port     ENCPort=GPIO_PF;
  unsigned char ENCMask=0;
  
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    ENCPort = GPIO_PD;
    ENCMask = GPIO_BIT_0 | GPIO_BIT_1 | GPIO_BIT_2;
    break;
#endif    
  case 1:
    ENCPort = GPIO_PF;
    ENCMask = GPIO_BIT_2 | GPIO_BIT_3 | GPIO_BIT_4;
    break;
  default:
    assert_param(0);
    break;
  }
  
  GPIO_Init             (ENCPort, ENCMask, &portConfigENC);
  GPIO_SetInputEnableReg(ENCPort, ENCMask, ENABLE);

  GPIO_EnableFuncReg    (ENCPort, GPIO_FUNC_REG_1, ENCMask);
  GPIO_DisableFuncReg   (ENCPort, GPIO_FUNC_REG_2, ENCMask);
}

static void IRQ_Common(unsigned char channel_number)
{
  TSB_EN_TypeDef*       pENCx = NULL;
  int32_t               ticks;
  int                   delta;
    
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pENCx = TSB_EN0;
    break;
#endif    
  case 1:
    pENCx = TSB_EN1;
    break;
  default:
    assert_param(0);
    break;
  }

  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_ENCODER_SPEED:
    EncoderData[channel_number].CW = (pENCx->TNCR & ENC_DIRECTION) >> 13;
    ticks=pENCx->CNT;
    
    delta = ticks - EncoderData[channel_number].ticksBetweenEvents;
    if ( (abs(delta)<500) && (VE_ActualStage[channel_number].main==Stage_FOC))
      EncoderData[channel_number].ticksBetweenEvents = ticks;
    else
      EncoderData[channel_number].ticksBetweenEvents = ticks;

    if (VE_ActualStage[channel_number].main!=Stage_FOC)
    {
      EncoderData[channel_number].Theta0=VE_Theta[channel_number].value;
      EncoderData[channel_number].event_nr=0;
    }    
  
    EncoderData[channel_number].event_nr++;
    if (EncoderData[channel_number].event_nr>=MotorParameterValues[channel_number].EncRes * MotorParameterValues[channel_number].EncMult)
      EncoderData[channel_number].event_nr=0;
    break;
      
  case MOTOR_ENCODER_EVENT:
    EncoderData[channel_number].event_nr=pENCx->CNT;
    if (EncoderData[channel_number].event_nr > MotorParameterValues[channel_number].EncRes * MotorParameterValues[channel_number].EncMult-1)
    {
      EncoderData[channel_number].event_nr=0;
      EncoderData[channel_number].FullTurns++;
      pENCx->TNCR |= ENC_COUNTER_CLEAR;
    }
    else if (EncoderData[channel_number].event_nr < -MotorParameterValues[channel_number].EncRes * MotorParameterValues[channel_number].EncMult+1)
    {
      EncoderData[channel_number].event_nr=0;
      EncoderData[channel_number].FullTurns--;
      pENCx->TNCR |= ENC_COUNTER_CLEAR;
    }
    
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Determine Omega (& Theta) from measured encoder interrupts
  *
  * @param  channel_number:  channel to measure
  * @retval None  
*/
void ENC_Determine_Omega(unsigned char channel_number)
{
  long long calculation = (long long)80000000
     / EncoderData[channel_number].ticksBetweenEvents
     / (MotorParameterValues[channel_number].EncRes)
     * FIXPOINT_15
     * MotorParameterValues[channel_number].PolePairs
     / VE_HzMax[channel_number];

  VE_Omega[channel_number].value = ((uint16_t)calculation)<<16;

  if (EncoderData[channel_number].CW==0)
    VE_Omega[channel_number].value*=-1;
  
  VE_Theta[channel_number].value += (int32_t) (VE_Omega[channel_number].part.reg
                                           * (VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));

  if (EncoderData[channel_number].event_nr==0)
    VE_Theta[channel_number].value=EncoderData[channel_number].Theta0;
}

/*! \brief  Interrupt handler of Encoder Channel 0
  *
  * @param  None
  * @retval None  
*/
void INTENC0_IRQHandler(void)
{
  IRQ_Common(0);
}

/*! \brief  Interrupt handler of Encoder Channel 1
  *
  * @param  None
  * @retval None  
*/
void INTENC1_IRQHandler(void)
{
  IRQ_Common(1);
}

/*! \brief  Initialize the Encoder
  *
  * @param  channel_number: channel to configure
  * @retval None  
*/
void ENC_Init(unsigned char channel_number)
{
  TSB_EN_TypeDef*   pENCx = NULL;

  /*! sanity check */
  if((MotorParameterValues[channel_number].Encoder  == MOTOR_NO_ENCODER)
   ||(MotorParameterValues[channel_number].EncRes   == 0)
   ||(MotorParameterValues[channel_number].EncMult  == 0))
  {
    MotorParameterValues[channel_number].Encoder = MOTOR_NO_ENCODER;  /* Disable Encoder usage */
    switch (channel_number)
    {
#ifdef __TMPM_370__
    case 0:
      NVIC_DisableIRQ(INTENC0_IRQn);
      break;
#endif
    case 1:
      NVIC_DisableIRQ(INTENC1_IRQn);
      break;
    default:
      assert_param(0);
      break;
    }
    return;
  }
  
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pENCx = TSB_EN0;
    break;
#endif
  case 1:
    pENCx = TSB_EN1;
    break;
  default:
    assert_param(0);
    break;
  }

  ENC_IOInit(channel_number);                                         /*! setup IO */

  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_ENCODER_SPEED:
    pENCx->TNCR = ENC_IRQ_ENABLE
                | ENC_NOISEFILTER_31
                | ENC_MODE_SENSOR_TIME         
                | ENC_RUN_ENABLE
                | ENC_3PHASE_ENABLE;
    break;
  case MOTOR_ENCODER_EVENT:
    pENCx->TNCR = ENC_IRQ_ENABLE
                | ENC_NOISEFILTER_31
                | ENC_MODE_SENSOR_EVENT         
                | ENC_RUN_ENABLE
                | ENC_3PHASE_ENABLE;
    break;
    
  default:
    assert_param(0);
    break;
  }
  
  __DSB();                                                            /* ! flush the pipeline */

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    NVIC_SetPriority(INTENC0_IRQn, INTERRUPT_PRIORITY_ENCODER);       /*! set the encoder interrupt priority */
    NVIC_EnableIRQ(INTENC0_IRQn);                                     /*! enable the interrupt */
    break;
#endif
  case 1:
    NVIC_SetPriority(INTENC1_IRQn, INTERRUPT_PRIORITY_ENCODER);       /*! set the encoder interrupt priority */
    NVIC_EnableIRQ(INTENC1_IRQn);                                     /*! enable the interrupt */
    break;
  default:
    assert_param(0);
    break;
  }
}
#endif /* USE_ENCODER */
