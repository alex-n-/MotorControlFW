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

#ifdef USE_EXTERNAL_SPEED_CONTROL

#include <stdlib.h>

#include TMPM_GPIO_HEADER_FILE
#include TMPM_ADC_HEADER_FILE
#include TMPM_TIMER_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "external_speed_control.h"
#include "ve.h"
#include "motorctrl.h"
#include "board.h"


static uint8_t ve_state    = 0;
static uint8_t ve_oldstate = 0;


static TMRB_InitTypeDef timerTMBRConfigADC =
{
  TMRB_INTERVAL_TIMER,
  TMRB_CLK_DIV_32,
  0xffff,
  TMRB_AUTO_CLEAR,
  0xffff,
};

static TMRB_InitTypeDef timerTMBRConfigPWM =
{
  TMRB_INTERVAL_TIMER,
  TMRB_CLK_DIV_2,
  0xffff,
  TMRB_FREE_RUN,
  0xffff,
};

static const GPIO_InitTypeDef portConfigInputPD =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_ENABLE,
};

static const GPIO_InitTypeDef portConfigOutput =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_ENABLE,
  GPIO_PULLDOWN_DISABLE,
};

/*! \brief  ADC Speed Control Capture IRQ
  *
  * Calculate speed of motor from ADC Value
  *
  * @retval None
*/
void SPEED_CONTROL_ADC_HANDLER(void)
{
    static uint8_t counter;
    ADC_Result      result;

    /* This function is called every 26ms */
    /* Limit update frequency to 100 ms */
    if (counter++ < 4)
      return;
    
    counter = 0;
    
    if ((SystemValues[1].ExternalSpeedCtrl==ESC_ADC) && (INIT_Done==1))
    {
      result=ADC_GetConvertResult(SPEED_CONTROL_ADC_CHANNEL, SPEED_CONTROL_ADC_REG);

      if (result.Bit.ADResult==0)
        ve_state=0;
      else
        ve_state=1;

      if (ve_state != ve_oldstate)
      {
        ve_oldstate = ve_state;
        if (ve_state)
          VE_Start(ESC_CHANNEL);
        else
          VE_Stop(ESC_CHANNEL);
      }

      MotorSetValues[ESC_CHANNEL].TargetSpeed = (int32_t) ( result.Bit.ADResult
                                                          * MotorParameterValues[1].HzLimit
                                                          * SECONDS_PER_MINUTE
                                                          / MotorParameterValues[ESC_CHANNEL].PolePairs
                                                          / 0xfff);

      if (GPIO_ReadDataBit(SPEED_CONTROL_CWCCW_PORT, SPEED_CONTROL_CWCCW_PIN))
        MotorSetValues[ESC_CHANNEL].TargetSpeed *= -1;
    }
    
    if (MotorErrorField[ESC_CHANNEL].Error!=VE_ERROR_NONE)
      GPIO_WriteDataBit(SPEED_CONTROL_FAULT_PORT,SPEED_CONTROL_FAULT_PIN,ENABLE);
    else
      GPIO_WriteDataBit(SPEED_CONTROL_FAULT_PORT,SPEED_CONTROL_FAULT_PIN,DISABLE);
   
}

/*! \brief  PWM Speed Control Capture IRQ
  *
  * Calculate speed of motor from PWM Duty
  *
  * @retval None
*/    

    int16_t         ratio;
    uint16_t        counter0,counter1;
    static uint16_t counter2;
    int16_t         total,low;


void SPEED_CONTROL_PWM_HANDLER(void)
{
    counter0 = TMRB_GetCaptureValue(SPEED_CONTROL_PWM_TMRB, TMRB_CAPTURE_0);
    counter1 = TMRB_GetCaptureValue(SPEED_CONTROL_PWM_TMRB, TMRB_CAPTURE_1);
    
    low   = counter1-counter0;
    total = counter1-counter2;

    counter2 = TMRB_GetUpCntValue  (SPEED_CONTROL_PWM_TMRB);

    if ( (total<0) || (low<0) )
      return;
    
    if ((SystemValues[1].ExternalSpeedCtrl==ESC_PWM) && (INIT_Done==1))
    {
      ratio= 1000 - ((1000*(total-low))/total);

      if (ratio>950)
        ratio=1000;
      if (ratio<50)
        ratio=0;
      
      if (ratio==0)
        ve_state=0;
      else
        ve_state=1;

      if (ve_state != ve_oldstate)
      {
        ve_oldstate = ve_state;
        if (ve_state)
          VE_Start(ESC_CHANNEL);
        else
          VE_Stop(ESC_CHANNEL);
      }
    
    MotorSetValues[ESC_CHANNEL].TargetSpeed = (int32_t) ( ratio
                                                        * MotorParameterValues[1].HzLimit
                                                        * SECONDS_PER_MINUTE
                                                        / MotorParameterValues[ESC_CHANNEL].PolePairs
                                                        / 1000);

    if (GPIO_ReadDataBit(SPEED_CONTROL_CWCCW_PORT, SPEED_CONTROL_CWCCW_PIN))
      MotorSetValues[ESC_CHANNEL].TargetSpeed *= -1;
  }
}


/*! \brief  External speed control Init
  *
  * Initalize ports ADC / Timers for External Speed Control 
  *
  * @retval None
*/
void EXTERNAL_SPEED_CONTROL_Init( void )
{
  GPIO_Init(SPEED_CONTROL_CWCCW_PORT,                                           /*< Configure Port for CW/CCW selection */
            SPEED_CONTROL_CWCCW_PIN,
            &portConfigInputPD);

  GPIO_Init(SPEED_CONTROL_PWM_PORT,                                             /* Configure Port for PWM speed control */
            SPEED_CONTROL_PWM_PIN,
            &portConfigInputPD);
  
  GPIO_Init(SPEED_CONTROL_FAULT_PORT,                                           /*< Configure Port for fault signaling */
            SPEED_CONTROL_FAULT_PIN,
            &portConfigOutput);

  GPIO_Init(SPEED_CONTROL_FG_PORT,                                              /*< Configure Port for speed signaling by 3 signal changes per electrical turn */
            SPEED_CONTROL_FG_PIN,
            &portConfigOutput);

  GPIO_EnableFuncReg(SPEED_CONTROL_PWM_PORT,
                     GPIO_FUNC_REG_1,
                     SPEED_CONTROL_PWM_PIN);
  
  ADC_Enable(SPEED_CONTROL_ADC_CHANNEL);                                        /* Set up ADC for speed control */
  ADC_SetClk(SPEED_CONTROL_ADC_CHANNEL,
             ADC_HOLD_FIX,
             ADC_FC_DIVIDE_LEVEL_2);
  ADC_SetTimerTrg(SPEED_CONTROL_ADC_CHANNEL,
                  SPEED_CONTROL_ADC_REG,
                  TRG_ENABLE(SPEED_CONTROL_ADC_REG));
  NVIC_EnableIRQ(SPEED_CONTROL_ADC_IRQ);
      
  TMRB_Enable(TSB_TB5);                                                         /* Set up TB5 for ADC Trigger */
  TMRB_Init(TSB_TB5, &timerTMBRConfigADC);
  TMRB_SetRunState(TSB_TB5, TMRB_RUN);
  
  TMRB_Enable(SPEED_CONTROL_PWM_TMRB);                                          /* Set up Timer for capture PWM signal */
  TMRB_Init(SPEED_CONTROL_PWM_TMRB,&timerTMBRConfigPWM);
  TMRB_SetCaptureTiming(SPEED_CONTROL_PWM_TMRB,
                        TMRB_CAPTURE_IN_RISING_FALLING);
  TMRB_SetRunState(SPEED_CONTROL_PWM_TMRB, TMRB_RUN);

  NVIC_SetPriority(SPEED_CONTROL_PWM_IRQ, 0);
  NVIC_EnableIRQ(SPEED_CONTROL_PWM_IRQ);
}


#endif /* USE_EXTERNAL_SPEED_CONTROL */
