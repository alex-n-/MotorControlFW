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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "config.h"
#include TMPM_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#include BOARD_GAIN_HEADER_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "debug.h"

#include "board.h"
#include "pmd.h"
#include "adc.h"
#include "motorctrl.h"
#include "ve.h"
#include "dso.h"
#include "hsdso.h"
#include "encoder.h"
#include "stall_detect.h"
#include "hv_serial_communication.h"
#include "board.h"
#include "turn_control.h"
#include "external_speed_control.h"

#include BOARD_BOARD_HEADER_FILE

#ifdef USE_LED
#include BOARD_LED_HEADER_FILE
#endif /* USE_LED */

#ifdef USE_RGB_LED
#include BOARD_RGB_LED_HEADER_FILE
#endif /* USE_RGB_LED */    

#ifdef USE_USER_CALLBACKS
#include "user_callbacks.h"
#endif /* USE_USER_CALLBACKS */

/*********************************************************************
************************ Global Data *********************************
*********************************************************************/
MotorSetSettings        MotorSetValues[MAX_CHANNEL];
MotorStateSettings      MotorStateValues[MAX_CHANNEL];
MotorParameters         MotorParameterValues[MAX_CHANNEL];
MotorError              MotorErrorField[MAX_CHANNEL];
PIControlSettings       PIControl[MAX_CHANNEL];
ChannelDependandValues  ChannelValues[MAX_CHANNEL];
SystemDependandValues   SystemValues[MAX_CHANNEL];
uint8_t                 ParameterChange[MAX_CHANNEL];

int16_t                 VE_Id[MAX_CHANNEL];                                     /*< [A/maxA] d-axis current */
int16_t                 VE_Iq[MAX_CHANNEL];                                     /*< [A/maxA] q-axis current */
int16_t                 VE_Id_reference[MAX_CHANNEL];                           /*< [A/maxA] Reference of d-axis current */
int16_t                 VE_Iq_reference[MAX_CHANNEL];                           /*< [A/maxA] Reference of q-axis current */
FRACTIONAL              VE_Omega[MAX_CHANNEL];                                  /*< [Hz/maxHz] Omega(speed): Electrical angle   */
FRACTIONAL              VE_Theta[MAX_CHANNEL];                                  /*< [---] Electrical angle */
FRACTIONAL              VE_Omega_command[MAX_CHANNEL];                          /*< [Hz/maxHz] Command of Motor omega (electrical angle)  */
int16_t                 VE_Theta_command[MAX_CHANNEL];                          /*< [---] Command of motor theta */
VEStage                 VE_ActualStage[MAX_CHANNEL];                            /*< Actual stage of VE */
PreCalc                 VE_PreCalc[MAX_CHANNEL];                                /*< Pre calculated values for further computation */
uint16_t                VE_HzMax[MAX_CHANNEL];                                  /*< Scaling factor for calculations */
uint32_t                VE_a_max[MAX_CHANNEL];                                  /*< Scaling factor for calculations */
uint32_t                VE_v_max[MAX_CHANNEL];                                  /*< Scaling factor for calculations */

/*********************************************************************
************************ Local Data **********************************
*********************************************************************/
static xTaskHandle      xVETask[MAX_CHANNEL];                                   /*< VE Task Handle */
static xTaskHandle      xShutdownHandle[MAX_CHANNEL];                           /*< Local storage of VE Task Handle needed for gentle shutdown */
static uint8_t          VE_PerformShutdown[MAX_CHANNEL];                        /*< Flag for Shutdown requested */
static uint32_t         VE_PerformShutdownTargetSpeed[MAX_CHANNEL];             /*< Remember Target Speed before Shutdown has initiated for recover */
static uint16_t	        VE_StageCounter[MAX_CHANNEL];                           /*< Stage counter (for reaching the above timings) */
static int32_t          VE_Vd[MAX_CHANNEL];                                     /*< [V/maxV] d-axis Voltage */
static int16_t          VE_Id_command[MAX_CHANNEL];                             /*< [A/maxA] d-axis current command */
static int16_t          VE_Iq_command[MAX_CHANNEL];                             /*< [A/maxA] q-axis current command */
static FRACTIONAL       VE_Iq_reference_I[MAX_CHANNEL];                         /*< reference command current */
static int16_t          VE_Omega_Target[MAX_CHANNEL];                           /*< [Hz/maxHz] OMEGA Target */

/*! \brief  Disable Vector Engine Interrupts
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_DisableIRQ(unsigned char channel_number)
{
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    NVIC_DisableIRQ(INTVCNA_IRQn);                                              /* disable VE0 interrupt while configure */
    break;
#endif
  case 1:
    NVIC_DisableIRQ(INTVCNB_IRQn);                                              /* disable VE1 interrupt while configure */
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Enable Vector Engine Interrupts
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_EnableIRQ(unsigned char channel_number)
{
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
      NVIC_EnableIRQ(INTVCNA_IRQn);                                             /* disable VE0 interrupt while configure */
      break;
#endif
  case 1:
      NVIC_EnableIRQ(INTVCNB_IRQn);                                             /* disable VE1 interrupt while configure */
      break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Limit Omega to speed up borders
  *
  * @param  now:      Actual Omega
  * @param  target:   Target Omega
  * @param  lim_up:   Accelleration upper limit
  * @param  lim_down: Accelleration lower limit
  * @retval Cosine of Theta
*/
static int16_t Limit_Omega(int16_t now, int16_t target, int16_t lim_up, int16_t lim_down)
{
  int32_t delta;                                                                /* local data with 64 bit */

  delta = (int32_t) target - now;

  if(delta == 0)                                                                /* nothing to do : useless call */
    return(target);                                                             /* target value is reached */
  else                                                                          /* tendencies */
  {
    if((lim_down < 1) || (lim_up < 1))                                          /* sanity check fo call parameter */
      return(now);

    if(delta > 0)                                                               /* positive delta -> positive tendency */
      if(delta > lim_up)                                                        /* determine staep wise or rest */
        return(now + lim_up);                                                   /* -> returns new value up step */
      else
        return(now + (int16_t) delta);                                          /* or -> returns new value up rest */
    else                                                                        /* negative delta -> negative tendency */
    {
      delta = -delta;                                                           /* determine step wise or rest */
      if(delta > lim_down)
        return(now - lim_down);                                                 /* -> returns new value down step */
      else
        return(now - (int16_t) delta);                                          /* or -> returns new value down rest */
    }
  }
}

/*! \brief  Sine Calculation
  *
  * @param  theta: Degree as int16_t
  * @retval Sine of Theta
*/
static int16_t Sine(int16_t theta)
{
  int16_t x;                                                                    /* local swap data */
  int16_t x1;                                                                   /* local swap data */
  int32_t temp32;                                                               /* local calculation data */

  x = (int16_t) (theta & 0x7fff);                                               /* theta is unsigned now */
  if(x >= ELE_DEG(90))
  {
    x = (int16_t) (ELE_DEG(180) - x);                                           /* macro returns 360°s fraction */
  }

  x1 = x; /* x1 start data */
  temp32 = (cSine1 * (int32_t) x1) << 1;                                        /* approximating the sine value */
  x1 = (int16_t) (x1 * (int32_t) x >> (16 - 1));
  temp32 += (cSine2 * (int32_t) x1) << 1;
  x1 = (int16_t) (x1 * (int32_t) x >> (16 - 1));
  temp32 += (cSine3 * (int32_t) x1) << 1;
  x1 = (int16_t) (x1 * (int32_t) x >> (16 - 1));
  temp32 += (cSine4 * (int32_t) x1) << 1;
  x1 = (int16_t) (x1 * (int32_t) x >> (16 - 1));
  temp32 += (cSine5 * (int32_t) x1) << 1;
  temp32 <<= 3;

  if((uint32_t) temp32 >= 0x7fffffff)                                           /* limit the result */
    temp32 = 0x7fffffff;

  if(theta <= 0)                                                                /* sign the result */
    temp32 *= (-1);

  return((int16_t) (temp32 >> 16));                                             /* return sine as fixpoint15 */
}

/*! \brief  Cosine Calculation
  *
  * @param  theta: Degree as int16_t
  * @retval Cosine of Theta
*/
static int16_t Cosine(int16_t theta)
{
  return(Sine((int16_t) (ELE_DEG(90) + theta)));
}

/*! \brief  Initialize Vector Engine Registers that are depending on the changeable values
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_InitParameterDependandValues(unsigned char channel_number)
{
  TEE_VE_TypeDef*     pVEx=NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }

  pVEx->TPWM  = VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency;
  pVEx->MDPRD = T0/SystemValues[channel_number].PWMFrequency;                   /* PWM frequency */

  pVEx->CIDKI = PIControl[channel_number].Id_Ki;                                /* Initialize PI-Control value */
  pVEx->CIDKP = PIControl[channel_number].Id_Kp;                                /* Initialize PI-Control value */
  pVEx->CIQKI = PIControl[channel_number].Iq_Ki;                                /* Initialize PI-Control value */
  pVEx->CIQKP = PIControl[channel_number].Iq_Kp;                                /* Initialize PI-Control value */

}
/*! \brief  Initialize the Vector Engine
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Init(unsigned char channel_number)
{
  TEE_VE_TypeDef*     pVEx=NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pVEx    = TEE_VE0;
    NVIC_SetPriority(INTVCNA_IRQn, INTERRUPT_PRIORITY_VE);
    break;
#endif
  case 1:
    pVEx    = TEE_VE1;
    NVIC_SetPriority(INTVCNB_IRQn, INTERRUPT_PRIORITY_VE);
    break;
  default:
    assert_param(0);
    break;
  }
  
  TEE_VEC->EN |= VE_ENABLE;                                                     /* VE Enable */

  pVEx->FMODE = (channel_number << 6) |                                         /* Always set this bits (madatory due to TD) */
                (channel_number << 4);

  switch (ChannelValues[channel_number].measurement_type)
  {
  case CURRENT_SHUNT_1:
    pVEx->FMODE |=  VE_FMODE_CURR_DETECT_1SHUNT_DOWN |
                    VE_FMODE_2PHASE_MODULATION;
    break;

  case CURRENT_SHUNT_3:
    pVEx->FMODE |=  VE_FMODE_3PHASE_MODULATION;
    break;

  case CURRENT_SENSOR_2:
    pVEx->FMODE |=  VE_FMODE_CURR_DETECT_2SENSOR |
                    VE_FMODE_3PHASE_MODULATION;
    break;
  }

  TEE_VEC->REPTIME |= (VE_REPTIME_1 << (channel_number*4) );                    /* Repeat time =1time */
  TEE_VEC->TRGMODE |= ( (1<<channel_number) << (channel_number*2) );

  VE_InitParameterDependandValues(channel_number);

  TEE_VEC->CPURUNTRG |= (1<<channel_number);

  __DSB();                                                                      /* flush the pipeline */
  __enable_irq();                                                               /* enable global interrupts */

}

/*! \brief  Do some precalculations of constants to reduce cpu load while running
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_PrecalculateValues(unsigned char channel_number)
{
  VE_a_max[channel_number] =   2500 * 1000
                             / ChannelValues[channel_number].sensitivity_current_measure
                             * 10
                             / gaintable[ChannelValues[channel_number].gain_current_measure];
  
  VE_v_max[channel_number] =   5000
                             / ChannelValues[channel_number].sensitivity_voltage_measure;
    
  VE_HzMax[channel_number]
    =   MotorParameterValues[channel_number].HzLimit                            /* put HzMax scaling factor 20 % over HzLimit */
      + MotorParameterValues[channel_number].HzLimit / 5; 
  
  VE_PreCalc[channel_number].WaitTime_Position
    = (uint16_t)(            MotorParameterValues[channel_number].InitDelay
                /            VE_PERIOD_TIME);                                   /* Time of Positioning (Force) */

  VE_PreCalc[channel_number].R_mult_amax_div_vmax
    = (int32_t)(           FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].Resistance
                *          VE_a_max[channel_number]
                /          VE_v_max[channel_number]
                /          1000000);                                            /* Resistance and current are in (mX) */

  VE_PreCalc[channel_number].InductanceDropoutFactor
    = (int32_t)(           FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].Inductance
                *          PAI2
                /          100000000
                *          VE_a_max[channel_number]
                *          VE_HzMax[channel_number]
                /          1000000000
                /          VE_v_max[channel_number]);

  VE_PreCalc[channel_number].PositionKiPreCalculation
    = (int32_t)(           FIXPOINT_15
                * (int64_t)PIControl[channel_number].Position_Ki
                *          VE_v_max[channel_number]
                /          VE_HzMax[channel_number]
                /          SystemValues[channel_number].PWMFrequency);

  VE_PreCalc[channel_number].PositionKpPreCalculation
    = (int32_t)(           FIXPOINT_15
                * (int64_t)PIControl[channel_number].Position_Kp
                *          VE_v_max[channel_number]
                /          VE_HzMax[channel_number]);

  VE_PreCalc[channel_number].ChangeFrq_normed_to_HzMax
    = (int16_t)(           FIXPOINT_15
                         * MotorParameterValues[channel_number].HzChange
                         / VE_HzMax[channel_number]);

  VE_PreCalc[channel_number].IqLim_normed_to_amax
    = (int16_t)(           FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].IqLim
                /          VE_a_max[channel_number]);

  VE_PreCalc[channel_number].IdLim_normed_to_amax
    = (int32_t)(           FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].IdLim
                /          VE_a_max[channel_number]);

  VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency
    = (int32_t)(           FIXPOINT_16
                * (int64_t)VE_HzMax[channel_number])
                /          SystemValues[channel_number].PWMFrequency;

  /* rad/sec² -> Hz/s : [rad]/2*pi=[Hz/s] : 125rad/s²=20Hz/s² */
  /* use this limits to define new maximum and minimum change rates
   * Speed up/down limit at Force */
  VE_PreCalc[channel_number].SpeedUpLimit
    = (int16_t)((int64_t)FIXPOINT_15
                *        VE_PERIOD_TIME
                *        MotorParameterValues[channel_number].MaxAngAcc
                /        VE_HzMax[channel_number]
                /        (PAI2/100000));

  if (VE_PreCalc[channel_number].SpeedUpLimit<1)
    VE_PreCalc[channel_number].SpeedUpLimit=1;
  
  /* set start current (Id) command (div 100 for low power range) */
  VE_PreCalc[channel_number].IdCurrentForInitposition
    = (uint16_t)(        FIXPOINT_15
                *        MotorParameterValues[channel_number].IdStart
                /        VE_a_max[channel_number]);

  /* set start Current (Iq) command (div 100 for low power range) */
  VE_PreCalc[channel_number].IqCurrentForInitposition
    = (uint16_t)(        FIXPOINT_15
                *        MotorParameterValues[channel_number].IqStart
                /        VE_a_max[channel_number]);
}

/*! \brief  Calculate Omega
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_Omega(uint8_t channel_number)
{
  int32_t VE_temp1;                                                             /*< intermediate (temp) values for better debugging */
  int32_t VE_temp2;
  int32_t VE_temp3;
  int32_t VE_temp4;
  int32_t VE_Ed_PI;                                                             /*< intermediate calculation of position */
  int32_t VE_Ed_I;                                                              /*< intermediate calculation of measured current over R */
  int16_t VE_Ed;                                                                /*< intermediate calculation of d-axis induced voltage */
  int32_t VE_Omega_Calculated;                                                  /*< calculated Omega */

  /* equations :\n
   * -# Vd = R*Id - OMEGAest * Iq + Ed  ->\n
   * -# Ed = Vd - R * Id + OMEGAest * Lq * Iq */

  /* Calculate the Dropout of Motor resistance\n
   *   R_Ed = R_Vd - Motor_R * R_Id + R_omega * Motor_Lq * R_Iq \ n
   * = temp1 * V_MAX = FIXPOINT_15 * Motor_R * R_Id * A_MAX */
  VE_temp1 = ((int32_t)VE_Id[channel_number] * VE_PreCalc[channel_number].R_mult_amax_div_vmax)<<1;

  /* Calculate the Dropout of inductance\n
   * temp2 * V_MAX = FIXPOINT_15 * 2Pi * R_omega * Hz_MAX * Motor_Lq / (1000 * R_Iq * A_MAX) */
  VE_temp2 = (int16_t)VE_PreCalc[channel_number].InductanceDropoutFactor;
  VE_temp2 = (((VE_Omega[channel_number].part.reg * VE_temp2)<<1) + ROUND_BIT15)>>16;
  VE_temp2 = (VE_temp2 * VE_Iq[channel_number])<<1;

  /* calculate the d-axis induced voltage */
  VE_Ed = (int16_t)((VE_Vd[channel_number] - VE_temp1 + VE_temp2 + ROUND_BIT15)>>16);

  /* Calculate the estimated position
   * OMEGAest = OMWEGAref - K * ed\n
   * OmegaErr(I) * Hz_MAX = FIXPOINT_15 * Position_Ki * CtrlPrd * R_Ed * V_MAX */
  VE_temp3 = (VE_PreCalc[channel_number].PositionKiPreCalculation * (int32_t)VE_Ed)<<1;

  /* the calculation was unsigned: correct Saturation */
  if(VE_Omega_command[channel_number].part.reg < 0)
    VE_temp3 *= (-1);

  VE_Ed_I = - VE_temp3;                 /* Saturation subtraction */

  /* Calculate the speed from proportional part:
   * Speed calculation value = Speed command value + Error I value
   * OmegaErr(P) * Hz_MAX = FIXPOINT_15 * Position_Kp * R_Ed * V_MAX */
  VE_temp4 = (VE_PreCalc[channel_number].PositionKpPreCalculation * (int32_t)VE_Ed)<<1;

  /* the calculation was unsigned: */
  if(VE_Omega_command[channel_number].part.reg < 0)
    VE_temp4 *= (-1);
  VE_Ed_PI = VE_Ed_I - VE_temp4;             /* error PI value */

  /* Calculate the speed from integrative part:\n
   * Speed calculation value = Speed command value + Error PI value */
  VE_Omega_Calculated = VE_Omega_command[channel_number].value + VE_Ed_PI;

  /* set new omega from calculation */
  VE_Omega[channel_number].value      = VE_Omega_Calculated;
}

/*! \brief  Calculate Iq-Ref while in Change Up
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_R_Iq_ref_NO_Current_Control (uint8_t channel_number)
{
  /* Code without Speed control. R_Iq_com -> R_Iq_ref */
  VE_Iq_reference[channel_number]         = VE_Iq_command[channel_number];
  VE_Iq_reference_I[channel_number].value = VE_Iq_command[channel_number]<<16;

  /* normalizing and limiting this values to maximum current */
  if(VE_Iq_command[channel_number] >= (VE_PreCalc[channel_number].IqLim_normed_to_amax))
  {
    VE_Iq_reference[channel_number]         = (int16_t)(VE_PreCalc[channel_number].IqLim_normed_to_amax);
    VE_Iq_reference_I[channel_number].value = (VE_PreCalc[channel_number].IqLim_normed_to_amax<<16);
  }
  else if(VE_Iq_command[channel_number] <= -(VE_PreCalc[channel_number].IqLim_normed_to_amax))
  {
    VE_Iq_reference[channel_number]         = -(int16_t)(VE_PreCalc[channel_number].IqLim_normed_to_amax);
    VE_Iq_reference_I[channel_number].value = -(VE_PreCalc[channel_number].IqLim_normed_to_amax<<16);
  }
}

/*! \brief  Calculate Iq-Ref while in FOC
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_Iq_ref_Current_Control (uint8_t channel_number)
{
  int32_t VE_Temp_P;
  int32_t VE_Temp_I;
  int16_t VE_Omega_Deviation;

  /* calculate the speed deviation\n
   * Omega(Dev) = Omega(command) - Omega(calculated) */
  VE_Omega_Deviation = (int16_t)((VE_Omega_command[channel_number].value - VE_Omega[channel_number].value)>>16);
  /* normalize the deviation to maximum speed and frequency based on current */
  VE_Temp_I =  (int32_t)( VE_Omega_Deviation
              *(int64_t)( FIXPOINT_16
                        * PIControl[channel_number].Speed_Ki
                        * VE_HzMax[channel_number]
                        / VE_a_max[channel_number]
                        / SystemValues[channel_number].PWMFrequency));

  VE_Iq_reference_I[channel_number].value += VE_Temp_I;                         /* integrate to Iq(reference) */
  
  /* keep this in range */
  if(VE_Iq_reference_I[channel_number].value > (VE_PreCalc[channel_number].IqLim_normed_to_amax<<16))  
    VE_Iq_reference_I[channel_number].value  = (VE_PreCalc[channel_number].IqLim_normed_to_amax<<16);
  else if(VE_Iq_reference_I[channel_number].value < -(VE_PreCalc[channel_number].IqLim_normed_to_amax<<16))
    VE_Iq_reference_I[channel_number].value       = -(VE_PreCalc[channel_number].IqLim_normed_to_amax<<16);

  /* proportional part of the control algorithm normalized on base of current */
  VE_Temp_P =  (int32_t)( VE_Omega_Deviation
              *(int64_t)( FIXPOINT_16
                        * PIControl[channel_number].Speed_Kp
                        * VE_HzMax[channel_number]
                        / VE_a_max[channel_number]));

  /* add the proportional part and and divide for correction */
  VE_Iq_reference[channel_number] = (int16_t)((VE_Iq_reference_I[channel_number].value + VE_Temp_P)>>16);

  /* keep this in range */
  if( VE_Iq_reference[channel_number] > VE_PreCalc[channel_number].IqLim_normed_to_amax)
    VE_Iq_reference[channel_number]   = VE_PreCalc[channel_number].IqLim_normed_to_amax;
  else if( VE_Iq_reference[channel_number] < -VE_PreCalc[channel_number].IqLim_normed_to_amax)
    VE_Iq_reference[channel_number]        = -VE_PreCalc[channel_number].IqLim_normed_to_amax;

}

/*! \brief  Calculate and limit Id-Ref
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_Id_ref(uint8_t channel_number)
{
  /* calculate the determination of R_Id_ref */
  VE_Id_reference[channel_number] = VE_Id_command[channel_number];

  /* normalizing and limiting this values to maximum current */
  if(VE_Id_command[channel_number]   >= VE_PreCalc[channel_number].IdLim_normed_to_amax)
     VE_Id_reference[channel_number]  = (int16_t)(VE_PreCalc[channel_number].IdLim_normed_to_amax);
  else if(VE_Id_command[channel_number] <= -VE_PreCalc[channel_number].IdLim_normed_to_amax)
     VE_Id_reference[channel_number]     = -(int16_t)(VE_PreCalc[channel_number].IdLim_normed_to_amax);
}

/*! \brief  Determine the Omega and Theta during FOC
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Determine_Omega_Theta(uint8_t channel_number)
{
#ifdef USE_ENCODER
  if (MotorParameterValues[channel_number].Encoder == MOTOR_ENCODER_SPEED)
    ENC_Determine_Omega(channel_number);
  else
#endif
  {
    VE_Calculate_Omega(channel_number);
    VE_Theta[channel_number].value += (int32_t) ( VE_Omega[channel_number].part.reg
                                               *( VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));
  }
}

/*! \brief  Calculate the Omeag and Theta for Forced Commutation
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Forced_Omega_Theta(uint8_t channel_number)
{
  VE_Omega[channel_number].value  = VE_Omega_command[channel_number].value;
  VE_Theta[channel_number].value += (int32_t) ( VE_Omega[channel_number].part.reg
                                             *( VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));
}

/*! \brief  Calculate new target speed
  *
  * Calculate the new setting Omega if the user gives a new target speed (in rpm)
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void calculate_new_target_speed(uint8_t channel_number)
{
  static int32_t  old_target=0;                                                 /* remember old value */
  int32_t         targetspeed60;                                                /* local target speed (multiplied with 60) */

  if (MotorSetValues[channel_number].TargetSpeed == old_target)
    return;                                                                     /* nothing new to calculate */

  old_target = MotorSetValues[channel_number].TargetSpeed;                      /* remember new setting */

  targetspeed60 =   MotorSetValues[channel_number].TargetSpeed
                  * MotorParameterValues[channel_number].PolePairs;

  /* limit the speed parameter to it's maximum (unsigned) */
  if (targetspeed60 > SECONDS_PER_MINUTE * MotorParameterValues[channel_number].HzLimit)
    targetspeed60 = SECONDS_PER_MINUTE * MotorParameterValues[channel_number].HzLimit;

  /* limit the speed parameter to it's maximum (unsigned) */
  if (targetspeed60 < -SECONDS_PER_MINUTE * MotorParameterValues[channel_number].HzLimit)
    targetspeed60 = -SECONDS_PER_MINUTE * MotorParameterValues[channel_number].HzLimit;

  /* check  CW only specified by the motor parameter */
  if (( MotorParameterValues[channel_number].Direction == MOTOR_CW_ONLY )  && ( targetspeed60 < 0))
    targetspeed60=0;

  /* check CCW only specified by the motor parameter */
  if (( MotorParameterValues[channel_number].Direction == MOTOR_CCW_ONLY ) && ( targetspeed60 > 0))
    targetspeed60=0;

  /* convert target speed from rpm to Hz:
   * 1Hz = 60sec = 1/60sec, targetspeed includes the pole count */
  VE_Omega_Target[channel_number] = (int32_t)( (int64_t) targetspeed60
                                                       * FIXPOINT_15
                                                       / VE_HzMax[channel_number]
                                                       / SECONDS_PER_MINUTE);
}

/*! \brief  Compute the values for the GUI
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void collect_data_for_display (uint8_t channel_number)
{
  int64_t         av_out;                                                       /* average value output */
  uint32_t        VE_Current;
  static uint8_t  av_count[MAX_CHANNEL];                                        /* average build counter value output */
  static int64_t  VE_SpeedAverage[MAX_CHANNEL][5];                              /*< output for GUI */

  /* Calculate rpm average over 5 ms */
  VE_SpeedAverage[channel_number][av_count[channel_number]++] = VE_Omega[channel_number].part.reg/5;
  if (av_count[channel_number] > 4)
  {
    av_count[channel_number] = 0;
  }
  av_out = VE_SpeedAverage[channel_number][0] + VE_SpeedAverage[channel_number][1] +
           VE_SpeedAverage[channel_number][2] + VE_SpeedAverage[channel_number][3] +
           VE_SpeedAverage[channel_number][4];

  /* Calculater current */
  /* VE_Iq_reference_I[channel_number].part.reg is integrate part of the current - */
  /* normalized to A_MAX and FIXPOINT_15 */
  VE_Current = (uint32_t)( abs(VE_Iq_reference_I[channel_number].part.reg)
                         * VE_a_max[channel_number] / FIXPOINT_15 );

  /* Calculate actual rpm */
  /* VE_Omega (-> av_out) is electrical speed normalized with HzMax and FIXPOINT_15 */
  /* therefor it's dependant of: */
  /* HzMax, Polepairs and FIXPOINT_15 */
  MotorStateValues[channel_number].ActualSpeed  = (
                                                   (av_out
                                                    * SECONDS_PER_MINUTE
                                                    * VE_HzMax[channel_number]
                                                    / FIXPOINT_15
                                                    / MotorParameterValues[channel_number].PolePairs
                                                   )
                                                   * 2+1)/2;

  /* Target speed in rpm */
  MotorStateValues[channel_number].TargetSpeed  = MotorSetValues[channel_number].TargetSpeed;

  /* Current in mA */
  MotorStateValues[channel_number].Current      = VE_Current;

  /* Torque in Ncm */
  /* TorqueFactor is in mNm */
  /* VE_Current is in mA */
  MotorStateValues[channel_number].Torque       = MotorParameterValues[channel_number].TorqueFactor
                                                  * VE_Current
                                                  / 10;
}

static void VE_PMD_NormalOutput(uint8_t channel_number)
{
  TEE_VEPMD_TypeDef * pVEPMDx = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pVEPMDx = TEE_VEPMD0;
    break;
#endif
  case 1:
    pVEPMDx = TEE_VEPMD1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  pVEPMDx->OUTCR = 0x1ff;
}

static void VE_PMD_SwitchOff(uint8_t channel_number)
{
  TEE_VEPMD_TypeDef * pVEPMDx = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pVEPMDx = TEE_VEPMD0;
    break;
#endif
  case 1:
    pVEPMDx = TEE_VEPMD1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  pVEPMDx->OUTCR = 0;
}

/*! \brief  Handle dynamic change of parameters
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_HandleParameterChange(unsigned char channel_number)
{
  VE_InitParameterDependandValues(channel_number);
  VE_PrecalculateValues(channel_number);
  ENC_Init(channel_number);
  PMD_HandleParameterChange(channel_number);
  VE_Calculate_Id_ref(channel_number);
  
  ParameterChange[channel_number]=0;
}

/*! \brief  Emergency Stage
  *
  * Shut down the Motor as fast as possible
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Emergency(uint8_t channel_number)
{
  //PMD_SwitchOff(channel_number);
  VE_PMD_SwitchOff(channel_number);    
  VE_Theta_command[channel_number]              = 0;
  VE_Theta[channel_number].value                = 0;
  MotorStateValues[channel_number].ActualSpeed  = 0;
  MotorStateValues[channel_number].Current      = 0;
  MotorStateValues[channel_number].Torque       = 0;
#ifdef USE_RGB_LED
  RGB_LED_SetValue(LED_RGB_RED);
#endif /* USE_RGB_LED */ 
}

/*! \brief  Stop Stage
  *
  * Switch off the Motor
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Stop(uint8_t channel_number)
{
  VE_DisableIRQ(channel_number);

  //PMD_SwitchOff(channel_number);
  VE_PMD_SwitchOff(channel_number);    

#ifdef USE_EMERGENCY_SIGNAL  
  PMD_EmergencyReset(channel_number);
#endif /* USE_EMERGENCY_SIGNAL */
#ifdef USE_OVERVOLTAGE_SIGNAL  
  PMD_OvervoltageReset(channel_number);
#endif /* USE_OVERVOLTAGE_SIGNAL */
  MotorErrorField[channel_number].Error &= 0x1;                                 /* Reset all errors except overtemperature */

  VE_Id_command[channel_number]          = 0;                                   /* reset all control process data */
  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;
  VE_StageCounter[channel_number]        = 0;                                   /* stage counter to limit time */

  if (SystemValues[1].ShutdownMode != SHUTDOWN_GENTLE)                          /* Use old Theta value in case that we know it */
  {
    VE_Theta_command[channel_number] = 0;
    VE_Theta[channel_number].value   = 0;
  }
}

/*! \brief  Measure Zerocurrent
  *
  * Measure the current value when no current is running
  * Needed to determine the Zero Crossing Point
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_MeasureZeroCurrent(uint8_t channel_number)
{
  /* This part has been moved to the IRQ to minimize the time for the Zero current Measurement */
}

/*! \brief  Initposition Stage
  *
  * Put the Motor to a defined position by putting current to a polepair
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Initposition(uint8_t channel_number)
{
  switch(VE_ActualStage[channel_number].sub)
  {                                                                             /* initial substage */
  case Substage_Step0:
    VE_PMD_NormalOutput(channel_number);    
    VE_Id_command[channel_number]    = 0;                                       /* Id is zero */
    VE_Theta_command[channel_number] = 0;                                       /* set position */
    VE_ActualStage[channel_number].sub= Substage_Step1;                         /* next step */
    /*  No break. Continue to the next statements. */
  case Substage_Step1:
    VE_StageCounter[channel_number]++;                                          /* stage counter to limit time */
    if(VE_StageCounter[channel_number] >= VE_PreCalc[channel_number].WaitTime_Position) /* check timeout */
    {
      VE_ActualStage[channel_number].sub = Substage_StepEnd;                    /* set last substage */
      VE_StageCounter[channel_number]    = 0;                                   /* stage counter to limit time */
    }
    /* set current "Id" increasing as C-load */
    VE_Id_command[channel_number] += (((abs(VE_PreCalc[channel_number].IdCurrentForInitposition)<<16))
                                        / VE_PreCalc[channel_number].WaitTime_Position)>>16;
    break;
  case Substage_StepEnd:                                                        /* final stage */
    VE_ActualStage[channel_number].main  = Stage_Force;                         /* set next stage */
    VE_ActualStage[channel_number].sub   = Substage_Step0;                      /* set next substage */
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Forced Commutation Stage
  *
  * Speed up the motor constantly with no regard to the torque
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Force(uint8_t channel_number)
{
  switch(VE_ActualStage[channel_number].sub)
  {
  case Substage_Step0:                                                          /* initial substage */
    VE_Id_command[channel_number] = abs(VE_PreCalc[channel_number].IdCurrentForInitposition);
    /*  No break. Continue to the next statements.  */
    /* If reach minimum frequency, it change to cStepEnd */
    if(abs(VE_Omega_command[channel_number].part.reg) >= VE_PreCalc[channel_number].ChangeFrq_normed_to_HzMax)
      VE_ActualStage[channel_number].sub = Substage_StepEnd;                    /* limit exceeded -> final stage active */

    VE_Iq_command[channel_number] = VE_Id_command[channel_number];              /* make some output */
    /* Update drive frequency. for ever in this mode */
    VE_Omega_command[channel_number].value = Limit_Omega(VE_Omega_command[channel_number].part.reg,
                                                        VE_Omega_Target[channel_number],
                                                        VE_PreCalc[channel_number].SpeedUpLimit,
                                                        VE_PreCalc[channel_number].SpeedUpLimit)<<16;
    break;
  case Substage_StepEnd:                                                        /* continuous stage */
    VE_ActualStage[channel_number].main = Stage_ChangeUp;                       /* final set for forcing stage */
    VE_ActualStage[channel_number].sub  = Substage_Step0;                       /* set next substage */
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Cahnge Up Stage - switching from Forced Commutation to FOC
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_ChangeUp(uint8_t channel_number)
{
  int32_t id_temp,iq_temp;
  int16_t VE_Lambda_Calculation;                                                /*< intermediate calculation of position */

  switch(VE_ActualStage[channel_number].sub)
  {
  case Substage_Step0:                                                          /* initial substage */
    VE_Lambda_Calculation               = 0;
    VE_ActualStage[channel_number].sub  = Substage_Step1;

    /* No break. Continue to the next statements. */
  case Substage_Step1:                                                          /* continuous stage */

    /* recalculate lambda (speed) by omega (normalized) */
    VE_Lambda_Calculation += (abs(VE_Omega_command[channel_number].part.reg)
                               * VE_HzMax[channel_number]
                               * VE_PERIOD_TIME/1000) << 1;

    /* limit lambda */
    if(VE_Lambda_Calculation > ELE_DEG(90))
      VE_Lambda_Calculation = ELE_DEG(90);

    /* calculate initial current from lambda */
    id_temp = (VE_PreCalc[channel_number].IdCurrentForInitposition * Cosine(VE_Lambda_Calculation)) << 1;
    iq_temp = (VE_PreCalc[channel_number].IqCurrentForInitposition * Sine  (VE_Lambda_Calculation)) << 1;

    /* use absolute values and determine direction */
    id_temp = abs(id_temp);
    iq_temp = abs(iq_temp);

    if(VE_Omega_Target[channel_number] < 0)
      iq_temp *= (-1);

    VE_DisableIRQ(channel_number);

    VE_Id_command[channel_number] = id_temp>>16;                                /* set current for vector engine */
    VE_Iq_command[channel_number] = iq_temp>>16;

    VE_EnableIRQ(channel_number);

    /* Update drive frequency. */
    VE_Omega_command[channel_number].value = Limit_Omega(VE_Omega_command[channel_number].part.reg,
                                                        VE_Omega_Target[channel_number],
                                                        VE_PreCalc[channel_number].SpeedUpLimit,
                                                        VE_PreCalc[channel_number].SpeedUpLimit)<<16;

    /* check frequency exceeding defined limits */
    if (abs(VE_Omega_command[channel_number].part.reg) >= VE_PreCalc[channel_number].ChangeFrq_normed_to_HzMax)
      VE_ActualStage[channel_number].sub = Substage_Step2;                      /* call final stage */

    break;
  case Substage_Step2:
    VE_Id_command[channel_number]       = 0;                                    /* leaving this mode : Id = 0 */
    /* Update drive frequency. */
    VE_ActualStage[channel_number].sub  = Substage_StepEnd;
    VE_StageCounter[channel_number]     = 0;
    break;
  case Substage_StepEnd:                                                        /* final stage */
    VE_ActualStage[channel_number].main = Stage_FOC;                            /* set next stage */
    VE_ActualStage[channel_number].sub  = Substage_Step0;                       /* set next substage */
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  FOC (Field Oriented Commutation) Stage
  *
  * Stage to reach for best performance
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_FOC(uint8_t channel_number)
{
  switch(VE_ActualStage[channel_number].sub)
  {
  case Substage_Step0:
    VE_ActualStage[channel_number].sub = Substage_Step1;
    /* No break. Continue to the next statements. */
  case Substage_Step1:                                                          /* continuous stage */
    /* Update drive frequency. */
    VE_Omega_command[channel_number].value = Limit_Omega(VE_Omega_command[channel_number].part.reg,
                                                        VE_Omega_Target[channel_number],
                                                        VE_PreCalc[channel_number].SpeedUpLimit,
                                                        VE_PreCalc[channel_number].SpeedUpLimit)<<16;
    /* Field-weakening control */
    VE_Id_command[channel_number] = 0;

    /* check low speed rotation */
    if(abs(VE_Omega_command[channel_number].part.reg) < VE_PreCalc[channel_number].ChangeFrq_normed_to_HzMax)
    {
      VE_ActualStage[channel_number].main  = Stage_Force;                       /* terminate this stage */
      VE_ActualStage[channel_number].sub   = Substage_Step0;
    }
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Common part for the Vector Engine IRQ
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void IRQ_Common(uint8_t channel_number)
{
  TEE_VE_TypeDef*     pVEx    = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }

  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Emergency:
  case Stage_Stop:
    pVEx->MODE       = VE_MODE_OFF;                                             /* Output OFF */
    break;
  case Stage_ZeroCurrentMeasure:
    pVEx->MODE        = VE_ZEROCURRENTEN;                                       /* Input 0 current */
    TEE_VEC->TASKAPP &= ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4));           /* VE start from output control */
    TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                         & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                         | VE_ACTSCH_SCHEDULE_9<<(channel_number*4);            /* Schedule 9 */
    break;
  case Stage_Initposition:
  case Stage_Force:
  case Stage_ChangeUp:
  case Stage_FOC:
    pVEx->MODE        = VE_OUTPUTENABLE;                                        /* Output ON */
#ifndef USE_USER_CALLBACKS
    TEE_VEC->TASKAPP  = (TEE_VEC->TASKAPP                                       /* VE start from current control */
                         & ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4)))
                         | VE_TASKAPP_CURRENT_CONTROL<<(channel_number*4);
    TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                         & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                         | VE_ACTSCH_SCHEDULE_1<<(channel_number*4);            /* Schedule 1 */
#else
    if ( (VE_ActualStage[channel_number].main == Stage_Initposition)
       &&(VE_ActualStage[channel_number].sub  == Substage_Step0)
       &&(TEE_VEC->ACTSCH >>(channel_number*4) == VE_ACTSCH_SCHEDULE_9 ))
    {
      TEE_VEC->TASKAPP  = (TEE_VEC->TASKAPP                                     /* VE start from current control */
                           & ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4)))
                           | VE_TASKAPP_CURRENT_CONTROL<<(channel_number*4);
      TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                           & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                           | CallbackTable[VE_TASKAPP_CURRENT_CONTROL].NextSchedule<<(channel_number*4);
    }
    else
    {
      TEE_VEC->TASKAPP  = (TEE_VEC->TASKAPP
                           & ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4)))
                           | CallbackTable[TEE_VEC->SCHTASKRUN>>6].NextTask<<(channel_number*4);

      TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                           & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                           | CallbackTable[TEE_VEC->SCHTASKRUN>>6].NextSchedule<<(channel_number*4);
    }

    CallbackTable[TEE_VEC->SCHTASKRUN>>6].Function();

    if (TEE_VEC->TASKAPP>>(channel_number*4) != VE_TASKAPP_CURRENT_CONTROL)
    {
      TEE_VEC->CPURUNTRG	|= (1<<channel_number);                         /* VE start */
      return;
    }

#endif /* USE_USER_CALLBACKS */
    break;
  default:
    assert_param(0);
    break;
  }

#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
#endif
#ifdef BOARD_VDC_CHANNEL_0                                                      /* If there is no measurement channel for VDC, we have to give a constant value */
  if (channel_number==0)
    pVEx->VDC =(0xfff<<3)*BOARD_VDC_CHANNEL_0/VE_v_max[channel_number];         /* ADC valus is 12 bit - max is 0xfff - blown up to 15 bit */
#endif
#ifdef BOARD_VDC_CHANNEL_0_HV_COMMUNICATION                                     /* Galvanic isolation prevents direct measurement - use HV communication instead */
  if (channel_number==0)
    pVEx->VDC = HV_Communication_GetValue(0)<<5;                                /* ADC valus from Device is 10 bit - max is 0x3ff - blown up to 15 bit */
#endif

#include "pwr_undefine.h"
#include BOARD_PWR_HEADER_FILE_1
#ifdef BOARD_VDC_CHANNEL_1                                                      /* If there is no measurement channel for VDC, we have to give a constant value */
  if (channel_number==1)
    pVEx->VDC =(0xfff<<3)*BOARD_VDC_CHANNEL_1/VE_v_max[channel_number];         /* ADC valus is 12 bit - max is 0xfff - blown up to 15 bit */
#endif
#ifdef BOARD_VDC_CHANNEL_1_HV_COMMUNICATION                                     /* Galvanic isolation prevents direct measurement - use HV communication instead */
  if (channel_number==1)
    pVEx->VDC = HV_Communication_GetValue(0)<<5;                                /* ADC valus from Device 10 bit - max is 0x3ff - blown up to 15 bit */
#endif
#include "pwr_undefine.h"

  if ( (ChannelValues[channel_number].measurement_type==CURRENT_SENSOR_2)
     &&(ChannelValues[channel_number].sensor_direction==CURRENT_SENSOR_NORMAL) )
  {
    pVEx->ID = (unsigned long)(-1 * (signed long)pVEx->ID);                     /* Invert for 2 Sensor solution */
    pVEx->IQ = (unsigned long)(-1 * (signed long)pVEx->IQ);                     /* Invert for 2 Sensor solution */
  }

  VE_Vd[channel_number] = pVEx->VD;                                             /* Read Vd from VE */
  VE_Id[channel_number] = (int16_t)(pVEx->ID>>16);                              /* Read Id from VE */
  VE_Iq[channel_number] = (int16_t)(pVEx->IQ>>16);                              /* Read Iq from VE */

  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
  case Stage_Emergency:
  case Stage_ZeroCurrentMeasure:
  case Stage_Initposition:
    break;
  case Stage_Force:
    VE_Forced_Omega_Theta(channel_number);
    VE_Calculate_R_Iq_ref_NO_Current_Control(channel_number);
    break;
  case Stage_ChangeUp:
    VE_Determine_Omega_Theta(channel_number);
    VE_Calculate_R_Iq_ref_NO_Current_Control(channel_number);
    break;
  case Stage_FOC:
    VE_Determine_Omega_Theta(channel_number);
    VE_Calculate_Iq_ref_Current_Control(channel_number);
    VE_Calculate_Id_ref(channel_number);
    break;
  default:
    assert_param(0);
    break;
  }

  if (channel_number==0)
    pVEx->MCTLF = 0x1;

  switch (VE_ActualStage[channel_number].main)
  {
  /*****************************************/
  /************ STOP Stage *****************/
  /*****************************************/
  case Stage_Stop:
    pVEx->OMEGA= 0x00u;
    pVEx->ID   = 0x00u;
    pVEx->IQ   = 0x00u;
    pVEx->VD   = 0x00u;
    pVEx->VQ   = 0x00u;
    pVEx->IDREF= 0x00u;
    pVEx->IQREF= 0x00u;
    pVEx->THETA= 0x00u;
    VE_Omega_command[channel_number].value  = 0;
    break;
  /*****************************************/
  /******** Measure Zero Current ***********/
  /*****************************************/
  case Stage_ZeroCurrentMeasure:
    VE_StageCounter[channel_number]++;
    if(VE_StageCounter[channel_number] >= 4)                                    /* check timeout - 2 times on 1/PWM_Frequency */
    {
      if (VE_Theta[channel_number].value == 0)                                  /* first time of startup or nit shutdown gentle mode */
        VE_ActualStage[channel_number].main = Stage_Initposition;               /* set next stage */
      else
        VE_ActualStage[channel_number].main = Stage_Force;                      /* set next stage */

      VE_ActualStage[channel_number].sub  = Substage_Step0;                     /* set next substage */
      VE_StageCounter[channel_number]     = 0;                                  /* stage counter to limit time */
    }
    break;
  /*****************************************/
  /********** Position Stage ***************/
  /*****************************************/
  case Stage_Initposition:
    pVEx->IDREF = VE_Id_command[channel_number];                                /* Set Id */
    break;
  /*****************************************/
  /**** Forced Commutation Stage ***********/
  /*****************************************/
  case Stage_Force:
    pVEx->IDREF = VE_Id_command[channel_number];                                /* Set Id */
    pVEx->IQREF = 0x00u;                                                        /* Set Iq=0 */
    pVEx->THETA = VE_Theta[channel_number].part.reg;                            /* Theta set */
#ifdef USE_MOTOR_DISCONNECT_DETECTION    
    if( ( abs(pVEx->IAADC-pVEx->IA0)
        + abs(pVEx->IBADC-pVEx->IB0)
        + abs(pVEx->IBADC-pVEx->IC0) ) < 0x50 )
    {
      MotorErrorField[channel_number].Error |= VE_NOMOTOR;
      VE_ActualStage[channel_number].main = Stage_Emergency;
    }
#endif /* USE_MOTOR_DISCONNECT_DETECTION */ 
    break;
  /*****************************************/
  /********** ChangeUp Stage ***************/
  /*****************************************/
  case Stage_ChangeUp:
    pVEx->IDREF = VE_Id_command[channel_number];                                /* Set Id */
    pVEx->IQREF = VE_Iq_command[channel_number];                                /* Set Iq */
    pVEx->THETA = VE_Theta[channel_number].part.reg;                            /* Theta set */
    break;
  /*****************************************/
  /************* FOC Stage *****************/
  /*****************************************/
  case Stage_FOC:
    pVEx->IDREF = VE_Id_reference[channel_number];                              /* Set Id */
    pVEx->IQREF = VE_Iq_reference[channel_number];                              /* Set Iq */
    pVEx->THETA = VE_Theta[channel_number].part.reg;                            /* Theta set */
    break;
  /*****************************************/
  /************ Emergency Stage ************/
  /*****************************************/
  case Stage_Emergency:
    break;
  default:
    assert_param(0);
    break;
  }

#ifdef USE_ENCODER
  /* Count electric turns - if motor deasn't have an encoder / hall sensors */
  if (MotorParameterValues[channel_number].Encoder == MOTOR_NO_ENCODER)
  {
    static uint8_t      sector[MAX_CHANNEL];
    static uint8_t      oldsector[MAX_CHANNEL];

    sector[channel_number] = pVEx->SECTOR;                                      /* Get actual sector */

    if ((oldsector[channel_number] == 11)                                       /* Changing from 11 -> 0 means CW */
     && (sector[channel_number]==0))
      EncoderData[channel_number].event_nr++;
    else if ((oldsector[channel_number] == 0)                                   /* Changing from 0 -> 11 means CCW */
          && (sector[channel_number]==11))
      EncoderData[channel_number].event_nr--;
    oldsector[channel_number]=sector[channel_number];                           /* remember old sector */

    if (EncoderData[channel_number].event_nr > MotorParameterValues[channel_number].PolePairs-1)
    {
      EncoderData[channel_number].event_nr=0;                                   /* reset electrical turn count */
      EncoderData[channel_number].FullTurns++;                                  /* increase full turns */
      EncoderData[channel_number].CW=1;                                         /* turning CW */
    }
    else if (EncoderData[channel_number].event_nr < -MotorParameterValues[channel_number].PolePairs+1)
    {
      EncoderData[channel_number].event_nr=0;                                   /* reset electrical turn count */
      EncoderData[channel_number].FullTurns--;                                  /* increase full turns */
      EncoderData[channel_number].CW=0;                                         /* turning CCW */
    }
  }
#endif /* USE_ENCODER */

#ifdef USE_EXTERNAL_SPEED_CONTROL
  if (SystemValues[ESC_CHANNEL].ExternalSpeedCtrl!=ESC_NONE)
  {
    static uint8_t      sector;
    static uint8_t      oldsector;

    sector = pVEx->SECTOR;                                                      /* Get actual sector */

    if (oldsector !=  sector)
    {
      GPIO_WriteDataBit(SPEED_CONTROL_FG_PORT,
                        SPEED_CONTROL_FG_PIN,
                        (~GPIO_ReadDataBit(SPEED_CONTROL_FG_PORT,
                                           SPEED_CONTROL_FG_PIN) & 0x01));

      oldsector=sector;                                                         /* remember old sector */
    }
  }
#endif /* USE_EXTERNAL_SPEED_CONTROL */

  TEE_VEC->CPURUNTRG	|= (1<<channel_number);                                 /* VE start */

}

/*! \brief  IRQ Handler for Vector Engine Channel 0
  *
  * @param  None
  * @retval None
*/
#ifdef __TMPM_370__
void INTVCNA_IRQHandler(void)
{
  IRQ_Common(0);
#ifdef USE_DSO
  DSO_Log (VE_CALLING , TEE_VE0 );
#endif // USE_DSO
#ifdef USE_HSDSO
  HsDSO_Log_VE(0);
#endif // USE_HSDSO
#ifdef USE_STALL_DETECT
  STALL_Detect(0);
#endif // USE_STALL_DETECT
  if (ParameterChange[0]!=0)
    VE_HandleParameterChange(0);
}
#endif /* __TMPM_370__ */

/*! \brief  IRQ Handler for Vector Engine Channel 1
  *
  * @param  None
  * @retval None
*/
void INTVCNB_IRQHandler(void)
{
  IRQ_Common(1);
#ifdef USE_DSO
  DSO_Log (VE_CALLING , TEE_VE1 );
#endif // USE_DSO
#ifdef USE_HSDSO
  HsDSO_Log_VE(1);
#endif // USE_HSDSO
#ifdef USE_STALL_DETECT
  STALL_Detect(1);
#endif // USE_STALL_DETECT
  if (ParameterChange[1]!=0)
    VE_HandleParameterChange(1);
}

/*! \brief  Functions that have to be called on regular basis
  *
  * This is the main worker part from the Vector Engine Task
  * put to an own function for better structure
  *
  * @param  channel_number: channel number to work with
  * @retval None
*/
static void VE_Loop(uint8_t channel_number)
{
  calculate_new_target_speed(channel_number);
  collect_data_for_display(channel_number);

  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
    VE_Stage_Stop(channel_number);                                              /* active in Stop mode */
    break;
  case Stage_ZeroCurrentMeasure:
    VE_Stage_MeasureZeroCurrent(channel_number);                                /* measure zero current */
    break;
  case Stage_Initposition:
    VE_Stage_Initposition(channel_number);                                      /* Positioning mode */
    break;
  case Stage_Force:
    VE_Stage_Force(channel_number);                                             /* Forced commutation mode */
    break;
  case Stage_ChangeUp:
    VE_Stage_ChangeUp(channel_number);                                          /* Change up mode */
    break;
  case Stage_FOC:
    VE_Stage_FOC(channel_number);                                               /* Speed control by current mode */
    break;
  case Stage_Emergency:
    VE_Stage_Emergency(channel_number);                                         /* Emergency mode */
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  clear off the rest for stopping the Vector Engine
  *
  * Helper function to be able to perform a gentle shutdown
  *
  * @param  pvParameters: channel number
  * @retval None
*/
static void complete_shutdown(uint8_t channel_number)
{
  TEE_VE_TypeDef*     pVEx    = NULL;

  switch (channel_number)
  {
  case 0:
    pVEx    = TEE_VE0;
    break;
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }

  VE_ActualStage[channel_number].main = Stage_Stop;                             /* set STOP stage */
  VE_ActualStage[channel_number].sub  = Substage_Step0;
#ifdef USE_LED
  LED_SetState(LED_SIGNAL_VE_RUN_BASE+channel_number,LED_OFF);                  /* Clear FOC signal Led */
#endif
#ifdef USE_RGB_LED
  RGB_LED_RestoreValue();  
#endif /* USE_RGB_LED */ 
  
  MotorStateValues[channel_number].ActualSpeed  = 0;                            /* Clear GUI Values (but don't Error Flag) */
  MotorStateValues[channel_number].TargetSpeed  = 0;
  MotorStateValues[channel_number].Current      = 0;
  MotorStateValues[channel_number].Torque       = 0;

  xVETask[channel_number]=NULL;                                                 /* Delete Task Handle */
  VE_DisableIRQ(channel_number);
  memset (pVEx,0,sizeof(TEE_VE_TypeDef));
  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;
  VE_Stage_Stop(channel_number);

  VE_PerformShutdown[channel_number]     =0;                                    /* Shutdown has finihed */
  vTaskDelete(xShutdownHandle[channel_number]);                                 /* Kill the Task */
}

/*! \brief  Vector Engine Worker Task
  *
  * @param  pvParameters: channel number
  * @retval None
*/
void VETask(void* pvParameters)
{
  long           channel_number = (long) pvParameters;
  portTickType   xLastWakeTime;
  const          portTickType xPeriod = ( VE_PERIOD_TIME / portTICK_RATE_MS );
  static uint8_t state = 0;

  xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    VE_Loop(channel_number);
    vTaskDelayUntil( &xLastWakeTime, xPeriod );

    if (state!=VE_ActualStage[channel_number].main)
    {
      state = VE_ActualStage[channel_number].main;
      if (VE_ActualStage[channel_number].main == Stage_FOC)
      {
#ifdef USE_LED
        LED_SetState(LED_SIGNAL_VE_RUN_BASE+channel_number,LED_ON);
#endif /* USE_LED */
#ifdef USE_RGB_LED
        RGB_LED_SetValue(LED_RGB_WHITE);
#endif /* USE_RGB_LED */ 
      }
      else
      {
#ifdef USE_LED
        LED_SetState(LED_SIGNAL_VE_RUN_BASE+channel_number,LED_OFF);
#endif /* USE_LED */
#ifdef USE_RGB_LED
        RGB_LED_RestoreValue();  
#endif /* USE_RGB_LED */
      }
    }

  if ((MotorErrorField[channel_number].Error & VE_OVERTEMPERATURE )!=0)
      complete_shutdown(channel_number);

    if (VE_PerformShutdown[channel_number]==1)
    {
      switch (SystemValues[channel_number].ShutdownMode)
      {
      case SHUTDOWN_SHORT_BRAKE:
      case SHUTDOWN_DISABLE_OUTPUT:
        complete_shutdown(channel_number);
        break;
      case SHUTDOWN_GENTLE:
        MotorSetValues[channel_number].TargetSpeed=0;
        if (MotorStateValues[channel_number].ActualSpeed==0)
          complete_shutdown(channel_number);
        break;
      default:
        assert_param(0);
        break;
      }
    }
  }
}

/*! \brief  Start the Vector Engine
  *
  * @param  channel_number: channel start
  * @retval Success
*/
int8_t VE_Start(uint8_t channel_number)
{
  while (INIT_Done==0)
    vTaskDelay( 100 / portTICK_RATE_MS );

  if (MotorErrorField[channel_number].Error!=0)
    return -2;

  if (VE_PerformShutdown[channel_number]==1)
  {
    VE_PerformShutdown[channel_number]=0;
    MotorSetValues[channel_number].TargetSpeed=VE_PerformShutdownTargetSpeed[channel_number];
    return 0;
  }

  if (xVETask[channel_number]!=NULL)
    return 0;

  VE_PrecalculateValues(channel_number);                                        /* do all pre-calculations */
  VE_Init(channel_number);                                                      /* Initialize the vector engine */

  VE_EnableIRQ(channel_number);

  VE_ActualStage[channel_number].main = Stage_ZeroCurrentMeasure;               /* initial set for positioning stage */
  VE_ActualStage[channel_number].sub  = Substage_Step0;

#ifdef USE_ENCODER
  ENC_Init(channel_number);                                                     /* initialize the Encoder 1 interface */
#endif

  if (xTaskCreate(VETask,
                  (signed char*)"VE",
                  VE_TASK_STACK_SIZE,
                  (void*)(int)channel_number,
                  VE_TASK_PRIORITY,
                  &xVETask[channel_number]
                  ) != pdPASS)
  {
    dprintf("Can't create VE Task(%d)\n",channel_number);
    return -1;
  }

  return 0;
}

/*! \brief  Stop the Vector Engine
  *
  * @param  channel_number: channel stop
  * @retval None
*/
int8_t VE_Stop(uint8_t channel_number)
{
  if (xVETask[channel_number]==NULL)
    return -1;

  if (MotorErrorField[channel_number].Error!=0)
  {
    xShutdownHandle[channel_number] = xVETask[channel_number];
    complete_shutdown(channel_number);
    VE_Stage_Stop(channel_number);
    return 0;
  }

  VE_PerformShutdown[channel_number]            = 1;
  VE_PerformShutdownTargetSpeed[channel_number] = MotorSetValues[channel_number].TargetSpeed;
  xShutdownHandle[channel_number]               = xVETask[channel_number];
#ifdef USE_TURN_CONTROL
  do_turn_control[channel_number] = 0;
#endif
  return 0;
}

