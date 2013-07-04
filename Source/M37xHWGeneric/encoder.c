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
#include "board.h"

extern EncoderValues  EncoderData[MAX_CHANNEL];                                 /*<! generic encoder result data */
static uint8_t        input_nr[MAX_CHANNEL] = {2,2};

static const GPIO_InitTypeDef portConfigENC =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

static const uint16_t HallSectorTable[6][2] =
{
  {0x4, 30},
  {0x6, 90},
  {0x2,150},
  {0x3,210},
  {0x1,270},
  {0x5,330},
};

static void ENC_IOInit(uint8_t channel_number)
{
  GPIO_Port ENCPort=GPIO_PF;
  uint8_t   ENCMask=0;
  
  switch (channel_number)
  {
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    ENCPort = GPIO_PD;
    if (input_nr[channel_number] >= 2)
      ENCMask |= GPIO_BIT_0 | GPIO_BIT_1;
    if ((input_nr[channel_number] == 1) || (input_nr[channel_number] == 3))
      ENCMask |= GPIO_BIT_2;
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    ENCPort = GPIO_PF;
    if (input_nr[channel_number] >= 2)
      ENCMask |= GPIO_BIT_2 | GPIO_BIT_3;
    if ((input_nr[channel_number] == 1) || (input_nr[channel_number] == 3))
      ENCMask |= GPIO_BIT_4;
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

/*! \brief  Get the Theta value from Encoder
  *
  * @param  channel_number:  channel to measure
  * @retval None  
*/
void ENC_GetTheta(unsigned char channel_number)
{
  GPIO_Port ENCPort=GPIO_PF;
  uint8_t   i=0;
  uint8_t   ENCStatus=0;
  
  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_HALL_UVW_SPEED:
  case MOTOR_HALL_UVW_EVENT:
  case MOTOR_HALL_UV_SPEED:
  case MOTOR_HALL_UV_EVENT:
    break;
  default:
    return;
    break;
  }
    
  switch (channel_number)
  {
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    ENCPort = GPIO_PD;
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    ENCPort = GPIO_PF;
    break;
  default:
    assert_param(0);
    break;
  }

  ENCStatus = GPIO_ReadData(ENCPort);
  
  if (channel_number == 1)                                                      /* Shift U Signal to bit 0 */
    ENCStatus = ENCStatus>>2;

  for(i=0;i<sizeof(HallSectorTable)/2* sizeof(uint16_t);i++)
    if (HallSectorTable[i][0] == ENCStatus)
      break;
  
//  VE_Theta[channel_number]. = HallSectorTable[i][1];
  
  printf("%x\n",ENCStatus);
  return;
}

static void IRQ_Common(unsigned char channel_number)
{
  TSB_EN_TypeDef* pENCx   = NULL;
  static uint8_t  hall_nr = 0;
    
  switch (channel_number)
  {
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    pENCx = TSB_EN0;
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    pENCx = TSB_EN1;
    break;
  default:
    assert_param(0);
    break;
  }

  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_HALL_UV_SPEED:
  case MOTOR_HALL_UVW_SPEED:
    EncoderData[channel_number].ticksBetweenEvents[hall_nr] = pENCx->CNT;
    EncoderData[channel_number].CW                          = (pENCx->TNCR & ENC_DIRECTION) >> 13;
    pENCx->TNCR=pENCx->TNCR | ENC_COUNTER_CLEAR;

    if ((hall_nr++)>=input_nr[channel_number]-1)
      hall_nr=0;

    ENC_GetTheta(channel_number);
    
    if (EncoderData[channel_number].CW == 1)
      EncoderData[channel_number].event_nr++;
    else
      EncoderData[channel_number].event_nr++;
    
    if (EncoderData[channel_number].event_nr > MotorParameterValues[channel_number].PolePairs * 2 * input_nr[channel_number] - 1)
    {
      EncoderData[channel_number].event_nr=0;
      if (VE_ActualStage[channel_number].main != Stage_FOC)
        EncoderData[channel_number].Theta0 = VE_Theta[channel_number].value;
      else
        VE_Theta[channel_number].value = EncoderData[channel_number].Theta0;
    }      
    else if (EncoderData[channel_number].event_nr < -MotorParameterValues[channel_number].PolePairs * 2 * input_nr[channel_number] + 1)
    {
      EncoderData[channel_number].event_nr=0;
      if (VE_ActualStage[channel_number].main != Stage_FOC)
        EncoderData[channel_number].Theta0 = VE_Theta[channel_number].value;
      else
        VE_Theta[channel_number].value = EncoderData[channel_number].Theta0;
    }      

    break;

  case MOTOR_SINGLE_PULSE_SPEED:
    EncoderData[channel_number].ticksBetweenEvents[0] = pENCx->CNT;
    pENCx->TNCR=pENCx->TNCR | ENC_COUNTER_CLEAR;

    if (VE_ActualStage[channel_number].main != Stage_FOC)
      EncoderData[channel_number].Theta0 = VE_Theta[channel_number].value;
    else
      VE_Theta[channel_number].value = EncoderData[channel_number].Theta0;
    break;
    
  case MOTOR_INCENC_AB_SPEED:
  case MOTOR_INCENC_ABZ_SPEED:
    break;

  case MOTOR_HALL_UV_EVENT:
  case MOTOR_HALL_UVW_EVENT:
    EncoderData[channel_number].event_nr=pENCx->CNT;
    if (EncoderData[channel_number].event_nr > MotorParameterValues[channel_number].PolePairs * 2 * input_nr[channel_number] - 1)
    {
      EncoderData[channel_number].event_nr=0;
      EncoderData[channel_number].FullTurns++;
      pENCx->TNCR |= ENC_COUNTER_CLEAR;
    }
    else if (EncoderData[channel_number].event_nr < -MotorParameterValues[channel_number].PolePairs * 2 * input_nr[channel_number] + 1)
    {
      EncoderData[channel_number].event_nr=0;
      EncoderData[channel_number].FullTurns--;
      pENCx->TNCR |= ENC_COUNTER_CLEAR;
    }
    break;

  case MOTOR_INCENC_AB_EVENT:
  case MOTOR_INCENC_ABZ_EVENT:
    EncoderData[channel_number].event_nr=pENCx->CNT;
    break;


  case MOTOR_SINGLE_PULSE_EVENT:
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
  uint64_t ENC_OmegaCalc;
void ENC_Determine_Omega_Theta(unsigned char channel_number)
{

  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_HALL_UV_SPEED:
  case MOTOR_HALL_UVW_SPEED:
    ENC_OmegaCalc = ((uint64_t) CPU_CLOCK
                                * FIXPOINT_15
                                / ( EncoderData[channel_number].ticksBetweenEvents[0]
                                   +EncoderData[channel_number].ticksBetweenEvents[1]
                                   +EncoderData[channel_number].ticksBetweenEvents[2]
                                  )
                                / 2
                                / VE_HzMax[channel_number]       
                                );
    if (EncoderData[channel_number].CW==0)
      ENC_OmegaCalc*=-1;

    VE_Omega[channel_number].value = ((uint16_t)ENC_OmegaCalc)<<16;
    VE_Theta[channel_number].value += (int32_t) ( VE_Omega[channel_number].part.reg
                                               *( VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));
    
    break;

  case MOTOR_SINGLE_PULSE_SPEED:
    ENC_OmegaCalc = ((uint64_t) CPU_CLOCK
                                * FIXPOINT_15
                                / ( EncoderData[channel_number].ticksBetweenEvents[0]
                                   +EncoderData[channel_number].ticksBetweenEvents[1]
                                   +EncoderData[channel_number].ticksBetweenEvents[2]
                                  )
                                / 2
                                / VE_HzMax[channel_number]       
                                );
    
    if (VE_Omega[channel_number].part.reg<=0)
      ENC_OmegaCalc*=-1;
    break;
    
  case MOTOR_INCENC_AB_SPEED:
  case MOTOR_INCENC_ABZ_SPEED:
    break;
  default:
    break;
  }
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
  TSB_EN_TypeDef* pENCx    = NULL;
  uint32_t        tncr     = 0;

  if(MotorParameterValues[channel_number].Encoder  == MOTOR_NO_ENCODER)
  {
    switch (channel_number)
    {
#if defined __TMPM_370__  || defined __TMPM_376__
    case 0:
      NVIC_DisableIRQ(INTENC0_IRQn);
      break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
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
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    pENCx = TSB_EN0;
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    pENCx = TSB_EN1;
    break;
  default:
    assert_param(0);
    break;
  }

  tncr = ENC_IRQ_ENABLE
       | ENC_NOISEFILTER_31
       | ENC_RUN_ENABLE;

  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_HALL_UVW_SPEED:
  case MOTOR_HALL_UVW_EVENT:
  case MOTOR_INCENC_ABZ_SPEED:
  case MOTOR_INCENC_ABZ_EVENT:
    tncr    |= ENC_3PHASE_ENABLE;
    input_nr[channel_number] = 3;
    break;
  case MOTOR_HALL_UV_SPEED:
  case MOTOR_HALL_UV_EVENT:
  case MOTOR_INCENC_AB_SPEED:
  case MOTOR_INCENC_AB_EVENT:
    input_nr[channel_number] = 2;
    break;
  case MOTOR_SINGLE_PULSE_SPEED:
  case MOTOR_SINGLE_PULSE_EVENT:
    input_nr[channel_number] = 1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  switch (MotorParameterValues[channel_number].Encoder)
  {
  case MOTOR_HALL_UV_SPEED:
  case MOTOR_HALL_UVW_SPEED:
  case MOTOR_SINGLE_PULSE_SPEED:
    tncr |= ENC_MODE_SENSOR_TIME;
    break;
  case MOTOR_HALL_UV_EVENT:
  case MOTOR_HALL_UVW_EVENT:
    tncr |= ENC_MODE_SENSOR_EVENT;
    break;
  case MOTOR_INCENC_ABZ_SPEED:
  case MOTOR_INCENC_ABZ_EVENT:
    tncr |= ENC_ZPHASE_DETECTED;
  case MOTOR_INCENC_AB_SPEED:
  case MOTOR_INCENC_AB_EVENT:
    pENCx->RELOAD = MotorParameterValues[channel_number].IncRotEncCnt-1;
  case MOTOR_SINGLE_PULSE_EVENT:
    tncr |= ENC_MODE_ENCODE;
    break;
  default:
    assert_param(0);
    break;
  }
    
  pENCx->TNCR = tncr;
    
  ENC_IOInit(channel_number);                                                   /*! setup IO */
  
  __DSB();                                                                      /* ! flush the pipeline */

  switch (channel_number)
  {
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    NVIC_SetPriority(INTENC0_IRQn, INTERRUPT_PRIORITY_ENCODER);                 /*! set the encoder interrupt priority */
    NVIC_EnableIRQ(INTENC0_IRQn);                                               /*! enable the interrupt */
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    NVIC_SetPriority(INTENC1_IRQn, INTERRUPT_PRIORITY_ENCODER);                 /*! set the encoder interrupt priority */
    NVIC_EnableIRQ(INTENC1_IRQn);                                               /*! enable the interrupt */
    break;
  default:
    assert_param(0);
    break;
  }
}
#endif /* USE_ENCODER */
