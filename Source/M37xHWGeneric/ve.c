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
#include TMPM_TIMER_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#include BOARD_GAIN_HEADER_FILE

#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
#ifdef USE_HV_COMMUNICATION
#define HV_COMM 1
#endif /* USE_HV_COMMUNICATION */
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
#ifdef USE_HV_COMMUNICATION
#define HV_COMM 1
#endif /* USE_HV_COMMUNICATION */
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */

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
#include "board.h"
#include "temperature_control.h"

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
VEStage                 VE_ActualStage[MAX_CHANNEL];                            /*< Actual stage of VE */
PreCalc                 VE_PreCalc[MAX_CHANNEL];                                /*< Pre calculated values for further computation */
uint16_t                VE_HzMax[MAX_CHANNEL];                                  /*< Scaling factor for calculations */
uint32_t                VE_a_max[MAX_CHANNEL];                                  /*< Scaling factor for calculations */
uint32_t                VE_v_max[MAX_CHANNEL];                                  /*< Scaling factor for calculations */
int16_t                 VE_OmegaCalc[MAX_CHANNEL];                              /*< Calculation value of position estimator */
#ifdef USE_LOAD_DEPENDANT_SPEED_REDUCTION
int32_t                 VE_TargetSpeed_Backup[MAX_CHANNEL];
#endif /* USE_LOAD_DEPENDANT_SPEED_REDUCTION */

/*********************************************************************
************************ Local Data **********************************
*********************************************************************/
static xTaskHandle      xVETask[MAX_CHANNEL];                                   /*< VE Task Handle */
static xTaskHandle      xShutdownHandle[MAX_CHANNEL];                           /*< Local storage of VE Task Handle needed for gentle shutdown */
static uint8_t          VE_PerformShutdown[MAX_CHANNEL];                        /*< Flag for Shutdown requested */
static uint32_t         VE_PerformShutdownTargetSpeed[MAX_CHANNEL];             /*< Remember Target Speed before Shutdown has initiated for recover */
static uint16_t	        VE_StageCounter[MAX_CHANNEL];                           /*< Stage counter (for reaching the above timings) */
static int32_t          VE_Vd[MAX_CHANNEL];                                     /*< [V/maxV] d-axis Voltage */
static FRACTIONAL       VE_Iq_reference_I[MAX_CHANNEL];                         /*< reference command current */
static int16_t          VE_Omega_Target[MAX_CHANNEL];                           /*< [Hz/maxHz] OMEGA Target */
#ifdef USE_MOTOR_DISCONNECT_DETECTION
static uint16_t         disconnect_counter[MAX_CHANNEL];
#endif /* USE_MOTOR_DISCONNECT_DETECTION */

static TMRB_InitTypeDef VE_TMBRConfig =
{
  TMRB_INTERVAL_TIMER,
  TMRB_CLK_DIV_2,
  0x9c40 / VE_CONTROL_LOOP_FREQUENCY,
  TMRB_AUTO_CLEAR,
  0x9c40 / VE_CONTROL_LOOP_FREQUENCY,
};

#define VE_FOC_HYSTERESIS       5

/******************************************************************************/
/******************************************************************************/
/***************************** Helper Functions *******************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Vector Engine Interrupts
  *
  * @param  channel_number: channel to work on
  * @param  state: enable or disable
  * @retval None
*/
static void VE_IRQ(unsigned char channel_number,VE_IRQ_State state)
{
  switch (state)
  {
  case VE_IRQ_OFF:
    switch (channel_number)
    {
#if defined __TMPM_370__ || defined __TMPM_376__
    case 0:
      NVIC_DisableIRQ(INTVCNA_IRQn);                                            /* disable VE0 interrupt while configure */
      break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
    case 1:
      NVIC_DisableIRQ(INTVCNB_IRQn);                                            /* disable VE1 interrupt while configure */
      break;
    default:
      assert_param(0);
      break;
    }
    break;

  case VE_IRQ_ON:
    switch (channel_number)
    {
#if defined __TMPM_370__ || defined __TMPM_376__
    case 0:
      NVIC_EnableIRQ(INTVCNA_IRQn);                                             /* enable VE0 interrupt while configure */
      break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
    case 1:
      NVIC_EnableIRQ(INTVCNB_IRQn);                                             /* enable VE1 interrupt while configure */
      break;
    default:
      assert_param(0);
      break;
    }
    break;

  default:
    assert_param(0);
    break;
  }
}

static void VE_PMD_Output(uint8_t channel_number, VE_PMD_State state)
{
  TEE_VEPMD_TypeDef * pVEPMDx = NULL;

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pVEPMDx = TEE_VEPMD0;
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    pVEPMDx = TEE_VEPMD1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  if (state)
    pVEPMDx->OUTCR = 0x1ff;
  else
    pVEPMDx->OUTCR = 0x0;
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
  static int32_t  targetspeed60;                                                /* local target speed (multiplied with 60) */

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
  uint32_t        current;
  static uint8_t  av_count[MAX_CHANNEL];                                        /* average build counter value output */
  static int64_t  speedAverage[MAX_CHANNEL][5];                              /*< output for GUI */

  /* Calculate rpm average over 5 ms */
  speedAverage[channel_number][av_count[channel_number]++] = VE_Omega[channel_number].part.reg/5;
  if (av_count[channel_number] > 4)
  {
    av_count[channel_number] = 0;
  }
  av_out = speedAverage[channel_number][0] + speedAverage[channel_number][1] +
           speedAverage[channel_number][2] + speedAverage[channel_number][3] +
           speedAverage[channel_number][4];

  /* Calculater current normalized to A_MAX and FIXPOINT_15 */
  current = (uint32_t)(abs(VE_Iq_reference_I[channel_number].part.reg)
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
  MotorStateValues[channel_number].Current      = current;

  /* Torque in Ncm */
  /* TorqueFactor is in mNm */
  /* VE_Current is in mA */
  MotorStateValues[channel_number].Torque       = MotorParameterValues[channel_number].TorqueFactor
                                                  * current
                                                  / 10;
}

/*! \brief  Limit Omega to speed up borders
  *
  * @param  now:      Actual Omega
  * @param  target:   Target Omega
  * @param  limit:    Accelleration limit
*/
static int32_t Limit_Omega(int64_t now, int64_t target, int64_t limit)
{
  int64_t delta;                                                                /* local data with 64 bit */

  delta = target - now;

  if(delta == 0)                                                                /* nothing to do : useless call */
    return(target);                                                             /* target value is reached */
  else                                                                          /* tendencies */
  {
    if(delta > 0)                                                               /* positive delta -> positive tendency */
      if(delta > limit)                                                         /* determine staep wise or rest */
        return(now + limit);                                                    /* -> returns new value up step */
      else
        return(now + delta);                                                    /* or -> returns new value up rest */
    else                                                                        /* negative delta -> negative tendency */
    {
      delta = -delta;                                                           /* determine step wise or rest */
      if(delta > limit)
        return(now - limit);                                                    /* -> returns new value down step */
      else
        return(now - delta);                                                    /* or -> returns new value down rest */
    }
  }
}

/******************************************************************************/
/******************************************************************************/
/************************* Precalculations and Inits **************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Set up timer for VE_Loop
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_InitTimer(unsigned char channel_number)
{
  switch (channel_number)
  {
#if (defined __TMPM370__) || (defined __TMPM376__)    
  case 0:
    TMRB_Enable(VE_CHANNEL0_TMRB);
    TMRB_Init(VE_CHANNEL0_TMRB, &VE_TMBRConfig);
    NVIC_EnableIRQ(VE_CHANNEL0_TMBR_IRQ);
    break;
#endif /* (defined __TMPM370__) || (defined __TMPM376__) */    
  case 1:
    TMRB_Enable(VE_CHANNEL1_TMRB);
    TMRB_Init(VE_CHANNEL1_TMRB, &VE_TMBRConfig);
    NVIC_EnableIRQ(VE_CHANNEL1_TMBR_IRQ);
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Initialize Vector Engine PI Registers
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_InitPISettings(unsigned char channel_number)
{
  TEE_VE_TypeDef*     pVEx=NULL;

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
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

/*! \brief  Initialize Vector Engine PWM Registers
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_InitPWMSettings(unsigned char channel_number)
{
  TEE_VE_TypeDef*     pVEx=NULL;

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }

  pVEx->TPWM  = VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency;
  pVEx->MDPRD = T0/SystemValues[channel_number].PWMFrequency;                   /* PWM frequency */
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
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pVEx    = TEE_VE0;
    NVIC_SetPriority(INTVCNA_IRQn, INTERRUPT_PRIORITY_VE);
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
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

  VE_InitPWMSettings(channel_number);
  VE_InitPISettings(channel_number);
  
  TEE_VEC->CPURUNTRG |= (1<<channel_number);

  __DSB();                                                                      /* flush the pipeline */
  __enable_irq();                                                               /* enable global interrupts */
}

/*! \brief  Calculate some values for further use
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_PrecalculateChannelValues(unsigned char channel_number)
{
  VE_a_max[channel_number] =   2500 * 1000
                             / ChannelValues[channel_number].sensitivity_current_measure
                             * 10
                             / gaintable[ChannelValues[channel_number].gain_current_measure];
  
  VE_v_max[channel_number] =   5000
                             / ChannelValues[channel_number].sensitivity_voltage_measure;
  
#ifdef BOARD_VDC_CHANNEL_0
  VE_v_max[0] = BOARD_VDC_CHANNEL_0;
#endif
#ifdef BOARD_VDC_CHANNEL_1
  VE_v_max[1] = BOARD_VDC_CHANNEL_1;
#endif
}

/*! \brief  Do some precalculations of Motor constants to reduce cpu load while running
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_PrecalculateMotorValues(unsigned char channel_number)
{
  uint64_t SpeedUpLimit;
    
  VE_HzMax[channel_number]
    =   MotorParameterValues[channel_number].HzLimit                            /* put HzMax scaling factor 20 % over HzLimit */
      + MotorParameterValues[channel_number].HzLimit / 5; 
  
  VE_PreCalc[channel_number].WaitTime_Position
    = (uint16_t)(            MotorParameterValues[channel_number].PositionDelay
                *            VE_CONTROL_LOOP_FREQUENCY);                        /* Time of Positioning (Force) */

  VE_PreCalc[channel_number].WaitTime_Bootstrap
    = (uint16_t)(            ChannelValues[channel_number].BootstrapDelay
                *            VE_CONTROL_LOOP_FREQUENCY);                        /* Time of Bootstapping */  
  
  VE_PreCalc[channel_number].R_mult_amax_div_vmax
    = (int32_t)(           FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].Resistance
                *          VE_a_max[channel_number]
                /          VE_v_max[channel_number]
                /          1000000);                                            /* Resistance and current are in (mX) */

  VE_PreCalc[channel_number].InductanceDropoutFactor
    = (uint32_t)(          FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].Inductance
                *          PAI2
                /          10000
                *          VE_a_max[channel_number]
                *          VE_HzMax[channel_number]
                /          1000000000
                /          VE_v_max[channel_number]);

  VE_PreCalc[channel_number].PositionKiPreCalculation
    = (uint32_t)(          FIXPOINT_15
                * (int64_t)PIControl[channel_number].Position_Ki
                *          VE_v_max[channel_number]
                /          VE_HzMax[channel_number]
                /          SystemValues[channel_number].PWMFrequency);

  VE_PreCalc[channel_number].PositionKpPreCalculation
    = (uint32_t)(          FIXPOINT_15
                * (int64_t)PIControl[channel_number].Position_Kp
                *          VE_v_max[channel_number]
                /          VE_HzMax[channel_number]
                /          1000);

  VE_PreCalc[channel_number].ChangeFrqUp_normed_to_HzMax
    = (uint16_t)(          FIXPOINT_15
                         * MotorParameterValues[channel_number].HzChange
                         / VE_HzMax[channel_number]);
  
  VE_PreCalc[channel_number].ChangeFrqDown_normed_to_HzMax
    = (uint16_t)(          FIXPOINT_15
                         * (MotorParameterValues[channel_number].HzChange
                            - MotorParameterValues[channel_number].HzChange * VE_FOC_HYSTERESIS / 100)
                         / VE_HzMax[channel_number]);

  VE_PreCalc[channel_number].IqLim_normed_to_amax
    = (uint32_t)(          FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].IqLim
                /          VE_a_max[channel_number]);

  VE_PreCalc[channel_number].IdLim_normed_to_amax
    = (uint32_t)(          FIXPOINT_15
                * (int64_t)MotorParameterValues[channel_number].IdLim
                /          VE_a_max[channel_number]);

  VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency
    = (uint32_t)(          FIXPOINT_16
                * (int64_t)VE_HzMax[channel_number])
                /          SystemValues[channel_number].PWMFrequency;

  /* rad/sec² -> Hz/s : [rad]/2*pi=[Hz/s] : 125rad/s²=20Hz/s² */
  /* use this limits to define new maximum and minimum change rates
   * Speed up/down limit */
  
  SpeedUpLimit  = ((uint64_t)FIXPOINT_15)<<16;
  SpeedUpLimit /= VE_HzMax[channel_number];
  SpeedUpLimit *= MotorParameterValues[channel_number].MaxAngAcc;
  SpeedUpLimit *= 10;
  SpeedUpLimit /= VE_CONTROL_LOOP_FREQUENCY;
  SpeedUpLimit /= PAI2;

  VE_PreCalc[channel_number].SpeedUpLimit = SpeedUpLimit;
  
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

/******************************************************************************/
/******************************************************************************/
/************************* Position Estimators ********************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Calculate Omega BLDC
  *
  * Position (Omega) Calculator for BLDC motors
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_Omega_BLDC(uint8_t channel_number)
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
  VE_OmegaCalc[channel_number]        = VE_Omega_Calculated>>16;                /* for DSO log */
}

/******************************************************************************/
/******************************************************************************/
/************************* Runtime calculations *******************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Calculate Id-Ref and Iq-Ref
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Calculate_Id_Iq_ref(uint8_t channel_number)
{
  int32_t  VE_Temp_P;
  int32_t  VE_Temp_I;
  int16_t  VE_Omega_Deviation;
  uint16_t increase_current;

  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
  case Stage_Bootstrap:
  case Stage_Brake:
  case Stage_Emergency:
    VE_Id_reference[channel_number] = 0;
    VE_Iq_reference[channel_number] = 0;
    break;
    
  case Stage_Initposition:
    VE_Id_reference[channel_number] = 0;

    /* set current "Iq" increasing as C-load */
    increase_current = (((abs(VE_PreCalc[channel_number].IqCurrentForInitposition)<<16))
                           / VE_PreCalc[channel_number].WaitTime_Position)>>16;
    /* check minimized increase */
    if (increase_current==0)
      increase_current=1;

    if (MotorSetValues[channel_number].TargetSpeed >= 0)
      VE_Iq_reference[channel_number] += increase_current;
    else
      VE_Iq_reference[channel_number] -= increase_current;

    /* normalizing and limiting this values to maximum current */
    if(VE_Iq_reference[channel_number]   >= VE_PreCalc[channel_number].IqCurrentForInitposition)
      VE_Iq_reference[channel_number]  = (int16_t)(VE_PreCalc[channel_number].IqCurrentForInitposition);
    else if(VE_Iq_reference[channel_number] <= -VE_PreCalc[channel_number].IqCurrentForInitposition)
      VE_Iq_reference[channel_number]     = -(int16_t)(VE_PreCalc[channel_number].IqCurrentForInitposition);

    break;
    
  case Stage_Force:
    if (VE_Omega_command[channel_number].part.reg >= 0)
      VE_Iq_reference[channel_number] = VE_PreCalc[channel_number].IqCurrentForInitposition;
    else
      VE_Iq_reference[channel_number] = -VE_PreCalc[channel_number].IqCurrentForInitposition;
    
    VE_Iq_reference_I[channel_number].part.reg  = VE_Iq_reference[channel_number];
    break;
    
  case Stage_FOC:
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
    VE_Id_reference[channel_number] = 0;
  
#ifdef USE_LOAD_DEPENDANT_SPEED_REDUCTION
    if( VE_Iq_reference[channel_number] > VE_PreCalc[channel_number].IqLim_normed_to_amax)
    {
      MotorSetValues[channel_number].TargetSpeed = MotorStateValues[channel_number].ActualSpeed;
      MotorErrorField[channel_number].Error |= VE_SPEEDREDUCTION;
    }
    else if( VE_Iq_reference[channel_number] < -VE_PreCalc[channel_number].IqLim_normed_to_amax)
    {
      MotorSetValues[channel_number].TargetSpeed = MotorStateValues[channel_number].ActualSpeed;
      MotorErrorField[channel_number].Error |= VE_SPEEDREDUCTION;
    }
    else
      if (do_turn_control[channel_number]!=1)
        if ( VE_PerformShutdown[channel_number] != 1)
            MotorSetValues[channel_number].TargetSpeed = VE_TargetSpeed_Backup[channel_number];
        else
          MotorSetValues[channel_number].TargetSpeed = 0;
#endif /* USE_LOAD_DEPENDANT_SPEED_REDUCTION */

    /* normalizing and limiting this values to maximum current */
    if(VE_Iq_reference[channel_number]   >= VE_PreCalc[channel_number].IqLim_normed_to_amax)
      VE_Iq_reference[channel_number]  = (int16_t)(VE_PreCalc[channel_number].IqLim_normed_to_amax);
    else if(VE_Iq_reference[channel_number] <= -VE_PreCalc[channel_number].IqLim_normed_to_amax)
      VE_Iq_reference[channel_number]     = -(int16_t)(VE_PreCalc[channel_number].IqLim_normed_to_amax);

    break;
    
  default:
    assert_param(0);
    break;
  }  
}

/*! \brief  Calculate or set Omeag and Theta
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Omega_Theta(uint8_t channel_number)
{
  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
  case Stage_Bootstrap:
  case Stage_Brake:
  case Stage_Initposition:
    break;
    
  case Stage_Force:
    VE_Calculate_Omega_BLDC(channel_number);
    VE_Omega[channel_number].value  = VE_Omega_command[channel_number].value;
    VE_Theta[channel_number].value += (int32_t) ( VE_Omega[channel_number].part.reg
                                             *( VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));
    break;
    
  case Stage_FOC:
#ifdef USE_ENCODER
    if (MotorParameterValues[channel_number].Encoder == MOTOR_ENCODER_SPEED)
      ENC_Determine_Omega(channel_number);
    else
#endif
    {
      VE_Calculate_Omega_BLDC(channel_number);
      VE_Theta[channel_number].value += (int32_t) ( VE_Omega[channel_number].part.reg
                                               *( VE_PreCalc[channel_number].HzMax_normed_to_PWMFrequency<<1));
    }
    break;

  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Handle dynamic change of parameters
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_HandleParameterChange(unsigned char channel_number)
{
  if ((ParameterChange[channel_number] & VE_CHANGE_BOARD_PARAMS_VE) != 0)
  {
    VE_PrecalculateChannelValues(channel_number);
    ParameterChange[channel_number] &= ~VE_CHANGE_BOARD_PARAMS_VE;
  }

  if ((ParameterChange[channel_number] & VE_CHANGE_SYSTEM_PARAMS_VE) != 0)
  {
    VE_PrecalculateMotorValues(channel_number);
#ifdef USE_SW_OVER_UNDER_VOLTAGE_DETECTION
#ifdef HV_COMM
  HV_Communication_OverUndervoltageDetect(channel_number);
#else
  ADC_OverUndervoltageDetect(channel_number);
#endif /* USE_HV_COMMUNICATION */
#endif /* USE_SW_OVER_UNDER_VOLTAGE_DETECTION */
#ifdef USE_TEMPERATURE_CONTROL
  TEMPERATURE_ConfigureADCforTemperature(channel_number);
#endif
    ParameterChange[channel_number] &= ~VE_CHANGE_SYSTEM_PARAMS_VE;
  }

  if ((ParameterChange[channel_number] & VE_CHANGE_PI_PARAMS) != 0)
  {
    VE_InitPISettings(channel_number);
    ParameterChange[channel_number] &= ~VE_CHANGE_PI_PARAMS;
  }

  if ((ParameterChange[channel_number] & VE_CHANGE_MOTOR_PARAMS) != 0)
  {
    VE_PrecalculateMotorValues(channel_number);
#ifdef USE_ENCODER
    ENC_Init(channel_number);
#endif
    VE_Calculate_Id_Iq_ref(channel_number);
    ParameterChange[channel_number] &= ~VE_CHANGE_MOTOR_PARAMS;
  }
}

/******************************************************************************/
/******************************************************************************/
/************************* Control loop stages ********************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Emergency Stage
  *
  * Shut down the Motor as fast as possible
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Emergency(uint8_t channel_number)
{
  VE_PMD_Output(channel_number,PMD_OFF);
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
  VE_IRQ(channel_number,VE_IRQ_OFF);
  VE_PMD_Output(channel_number,PMD_OFF);

#ifdef USE_EMERGENCY_SIGNAL  
  PMD_EmergencyReset(channel_number);
#endif /* USE_EMERGENCY_SIGNAL */
#ifdef USE_OVERVOLTAGE_SIGNAL  
  PMD_OvervoltageReset(channel_number);
#endif /* USE_OVERVOLTAGE_SIGNAL */
  MotorErrorField[channel_number].Error &= 0x1;                                 /* Reset all errors except overtemperature */

  VE_Calculate_Id_Iq_ref(channel_number);
  
  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;
  VE_StageCounter[channel_number]        = 0;                                   /* stage counter to limit time */

  if (SystemValues[channel_number].ShutdownMode != SHUTDOWN_GENTLE)
    VE_Theta[channel_number].value = 0;
  
}

/*! \brief  Bootstrap Stage
  *
  * Bootstrap IGBTs
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Bootstrap(uint8_t channel_number)
{
  VE_Calculate_Id_Iq_ref(channel_number);

  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;

  VE_StageCounter[channel_number]++;                                            /* stage counter to limit time */

  if(VE_StageCounter[channel_number] >= VE_PreCalc[channel_number].WaitTime_Bootstrap)                                      /* check timeout */
  {
    VE_StageCounter[channel_number]     = 0;
    VE_ActualStage[channel_number].main = Stage_ZeroCurrentMeasure;             /* startup */
    VE_ActualStage[channel_number].sub  = Substage_Step0;
  }
}

/*! \brief  Brake Stage
  *
  * Shortbrake
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void VE_Stage_Brake(uint8_t channel_number)
{

  VE_Calculate_Id_Iq_ref(channel_number);

  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;
  VE_StageCounter[channel_number]        = 0;                                   /* stage counter to limit time */

  MotorStateValues[channel_number].ActualSpeed  = 0;                            /* Clear GUI Values (but don't Error Flag) */
  MotorStateValues[channel_number].TargetSpeed  = 0;
  MotorStateValues[channel_number].Current      = 0;
  MotorStateValues[channel_number].Torque       = 0;
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
    VE_PMD_Output(channel_number,PMD_ON);
    VE_ActualStage[channel_number].sub= Substage_Step1;                         /* next step */
    /*  No break. Continue to the next statements. */
  case Substage_Step1:
    VE_StageCounter[channel_number]++;                                          /* stage counter to limit time */
    if(VE_StageCounter[channel_number] >= VE_PreCalc[channel_number].WaitTime_Position) /* check timeout */
    {
      VE_ActualStage[channel_number].sub = Substage_StepEnd;                    /* set last substage */
      VE_StageCounter[channel_number]    = 0;                                   /* stage counter to limit time */
    }

    VE_Omega_Theta(channel_number);
    VE_Calculate_Id_Iq_ref(channel_number);
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

    /* If reach minimum frequency, it change to cStepEnd */
    if(abs(VE_Omega_command[channel_number].part.reg) >= VE_PreCalc[channel_number].ChangeFrqUp_normed_to_HzMax)
      VE_ActualStage[channel_number].sub = Substage_StepEnd;                    /* limit exceeded -> final stage active */

    /* Update drive frequency. for ever in this mode */
    VE_Omega_command[channel_number].value = Limit_Omega(VE_Omega_command[channel_number].value,
                                                        VE_Omega_Target[channel_number]<<16,
                                                        VE_PreCalc[channel_number].SpeedUpLimit);
    break;
    
  case Substage_StepEnd:                                                        
    VE_ActualStage[channel_number].main = Stage_FOC;                            /* final set for FOC stage */
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
  case Substage_Step0:                                                          /* continuous stage */
    /* Update drive frequency. */
    VE_Omega_command[channel_number].value = Limit_Omega(VE_Omega_command[channel_number].value,
                                                        VE_Omega_Target[channel_number]<<16,
                                                        VE_PreCalc[channel_number].SpeedUpLimit);

    /* check low speed rotation */
    if(abs(VE_Omega_command[channel_number].part.reg) < VE_PreCalc[channel_number].ChangeFrqDown_normed_to_HzMax)
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
#ifdef  USE_TEMPERATURE_CONTROL 
  static uint16_t temp_counter=0;
#endif /* USE_TEMPERATURE_CONTROL */  
  
  calculate_new_target_speed(channel_number);
  collect_data_for_display(channel_number);

#ifdef  USE_TEMPERATURE_CONTROL 
  if (temp_counter++>=1000)
  {
    temp_counter=0;
    TEMPERATURE_CheckOvertemp(channel_number);
  }
#endif /* USE_TEMPERATURE_CONTROL */  

  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
    VE_Stage_Stop(channel_number);                                              /* active in Stop mode */
    break;
  case Stage_Bootstrap:
    VE_Stage_Bootstrap(channel_number);                                         /* active in Bootstrap mode */
    break;
  case Stage_Brake:
    VE_Stage_Brake(channel_number);                                             /* active in Brake mode */
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

/******************************************************************************/
/******************************************************************************/
/*************************** IRQ part of VE ***********************************/
/******************************************************************************/
/******************************************************************************/

/*! \brief  Common part for the Vector Engine IRQ
  *
  * @param  channel_number: channel to work on
  * @retval None
*/
static void IRQ_Common(uint8_t channel_number)
{
  TEE_VE_TypeDef*     pVEx    = NULL;
#ifdef USE_MOTOR_DISCONNECT_DETECTION
  uint32_t meas_current,calc_current;
#endif /* USE_MOTOR_DISCONNECT_DETECTION */

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }

  /*****************************************/
  /************* Task selection ************/
  /*****************************************/
  switch (VE_ActualStage[channel_number].main)
  {
  /************ Emergency Stage ************/
  /*************** Stop Stage **************/
  case Stage_Emergency:
  case Stage_Stop:
    pVEx->MODE        = VE_MODE_OFF;                                            /* Output OFF */
    break;
   
  /******** Zero Current Measurement *******/
  case Stage_ZeroCurrentMeasure:
    pVEx->MODE        = VE_ZEROCURRENTEN;                                       /* Input 0 current */
    TEE_VEC->TASKAPP  = (TEE_VEC->TASKAPP                                       /* VE start from current control */
                         & ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4)))
                         | VE_TASKAPP_OUTPUT_CONTROL<<(channel_number*4);
    TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                         & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                         | VE_ACTSCH_SCHEDULE_9<<(channel_number*4);            /* Schedule 9 */
    break;

  /*********** Bootstrap Stage *************/
  /************** Break Stage **************/
  case Stage_Bootstrap:
  case Stage_Brake:
    pVEx->MODE        = VE_OUTPUTENABLE;                                        /* Output ON */
    TEE_VEC->TASKAPP  = (TEE_VEC->TASKAPP                                       /* VE start from sin/cos computation */
                         & ~(VE_TASKAPP_CLEAN_MASK<<(channel_number*4)))
                         | VE_TASKAPP_SIN_COS_COMPUTATION<<(channel_number*4);
    TEE_VEC->ACTSCH   = (TEE_VEC->ACTSCH
                         & ~(VE_ACTSCH_CLEAN_MASK<<(channel_number*4)))
                         | VE_ACTSCH_SCHEDULE_4<<(channel_number*4);            /* Schedule 9 */
    break;

  /********** Position Stage ***************/
  /******* Forced Commutation Stage ********/
  /*********** ChangeUp Stage **************/
  /************* FOC Stage *****************/
  case Stage_Initposition:
  case Stage_Force:
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
#ifdef USE_HV_COMMUNICATION                                                     /* Galvanic isolation prevents direct measurement - use HV communication instead */
  if (channel_number==0)
    pVEx->VDC = HV_Communication_GetValue(0)<<5;                                /* ADC valus from Device 10 bit - max is 0x3ff - blown up to 15 bit */
#endif
#ifdef BOARD_VDC_CHANNEL_0                                                      /* If there is no measurement channel for VDC, we have to give a constant value */
  if (channel_number==0)
    pVEx->VDC =(0xfff<<3)*BOARD_VDC_CHANNEL_0/VE_v_max[channel_number];         /* ADC valus is 12 bit - in bits 15-3 - max is 0xfff - blown up to right position */
#endif
#include "pwr_undefine.h"

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
#endif /* BOARD_PWR_HEADER_FILE_1 */
#ifdef USE_HV_COMMUNICATION                                                     /* Galvanic isolation prevents direct measurement - use HV communication instead */
  if (channel_number==1)
    pVEx->VDC = HV_Communication_GetValue(0)<<5;                                /* ADC valus from Device 10 bit - max is 0x3ff - blown up to 15 bit */
#endif
#ifdef BOARD_VDC_CHANNEL_1                                                      /* If there is no measurement channel for VDC, we have to give a constant value */
  if (channel_number==1)
    pVEx->VDC =(0xfff<<3)*BOARD_VDC_CHANNEL_1/VE_v_max[channel_number];         /* ADC valus is 12 bit - in bits 15-3 - max is 0xfff - blown up to right position */
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

  /*****************************************/
  /****** Current / Pos calculation ********/
  /*****************************************/
  switch (VE_ActualStage[channel_number].main)
  {
  case Stage_Stop:
  case Stage_Bootstrap:
  case Stage_Emergency:
  case Stage_ZeroCurrentMeasure:
  case Stage_Initposition:
  case Stage_Brake:
    break;
  case Stage_Force:
  case Stage_FOC:
    VE_Omega_Theta(channel_number);
    VE_Calculate_Id_Iq_ref(channel_number);
    break;
  default:
    assert_param(0);
    break;
  }

  switch (VE_ActualStage[channel_number].main)
  {
  /*****************************************/
  /************ Emergency Stage ************/
  /************** STOP Stage ***************/
  /*********** Bootstrap Stage *************/
  /************** Break Stage **************/
  /*****************************************/
  case Stage_Emergency:
  case Stage_Stop:
  case Stage_Bootstrap:
  case Stage_Brake:
    pVEx->OMEGA= 0x0;
    pVEx->ID   = 0x0;
    pVEx->IQ   = 0x0;
    pVEx->VD   = 0x0;
    pVEx->VQ   = 0x0;
    pVEx->IDREF= 0x0;
    pVEx->IQREF= 0x0;
    pVEx->THETA= 0x0;
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
  /************ Position Stage *************/
  /******* Forced Commutation Stage ********/
  /************* FOC Stage *****************/
  /*****************************************/
  case Stage_Force:
#ifdef USE_MOTOR_DISCONNECT_DETECTION
    meas_current = abs(pVEx->IAADC-pVEx->IA0) + abs(pVEx->IBADC-pVEx->IB0) + abs(pVEx->IBADC-pVEx->IC0);
    calc_current = (MotorParameterValues[channel_number].IdStart + MotorParameterValues[channel_number].IqStart) * VE_a_max[channel_number] / 0x7ff;
      
    if(  ( meas_current < calc_current / 5)
       &&( abs(VE_Omega_command[channel_number].part.reg) >= 1000) )
       disconnect_counter[channel_number]++;
    else
      disconnect_counter[channel_number]=0;
       
    if (disconnect_counter[channel_number] > 1000)
    {
      MotorErrorField[channel_number].Error |= VE_NOMOTOR;
      VE_ActualStage[channel_number].main = Stage_Emergency;
    }
#endif /* USE_MOTOR_DISCONNECT_DETECTION */ 
  case Stage_Initposition:
  case Stage_FOC:
    pVEx->IDREF = VE_Id_reference[channel_number];                              /* Set Id reference */
    pVEx->IQREF = VE_Iq_reference[channel_number];                              /* Set Iq reference */
    pVEx->THETA = VE_Theta[channel_number].part.reg;                            /* Theta set */
    break;

  /*****************************************/
  /***************** ERROR *****************/
  /*****************************************/
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
#if defined __TMPM_370__ || defined __TMPM_376__
void INTVCNA_IRQHandler(void)
{
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

  IRQ_Common(0);
}
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */

/*! \brief  IRQ Handler for Vector Engine Channel 1
  *
  * @param  None
  * @retval None
*/
void INTVCNB_IRQHandler(void)
{
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

  IRQ_Common(1);
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
  VE_IRQ(channel_number,VE_IRQ_OFF);
  memset (pVEx,0,sizeof(TEE_VE_TypeDef));
  VE_Omega[channel_number].value         = 0;
  VE_Omega_command[channel_number].value = 0;
  VE_Stage_Stop(channel_number);

  VE_PerformShutdown[channel_number]     = 0;                                   /* Shutdown has finihed */

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    TMRB_SetRunState(VE_CHANNEL0_TMRB, TMRB_STOP);
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    TMRB_SetRunState(VE_CHANNEL1_TMRB, TMRB_STOP);
    break;
  default:
  assert_param(0);
    break;
  }

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
  static uint8_t state = 0;

  VE_IRQ(channel_number,VE_IRQ_ON);
  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    TMRB_SetRunState(VE_CHANNEL0_TMRB, TMRB_RUN);
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    TMRB_SetRunState(VE_CHANNEL1_TMRB, TMRB_RUN);
    break;
  default:
  assert_param(0);
    break;
  }

  while(1)
  {
    
    vTaskDelay( 100 / portTICK_RATE_MS );

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
        VE_ActualStage[channel_number].main = Stage_Brake;                      /* short brake */
        VE_ActualStage[channel_number].sub  = Substage_Step0;
        break;
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

void VE_CHANNEL0_TMBR_IRQ_HANDLER(void)
{
   VE_Loop(0);
}

void VE_CHANNEL1_TMBR_IRQ_HANDLER(void)
{
   VE_Loop(1);
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
  
#ifdef  USE_TEMPERATURE_CONTROL 
  TEMPERATURE_CheckOvertemp(channel_number);
#endif /* USE_TEMPERATURE_CONTROL */  
  
  if (MotorErrorField[channel_number].Error!=0)
    return -2;

  if (VE_ActualStage[channel_number].main == Stage_Brake)
  {
    complete_shutdown(channel_number);
    VE_Stage_Stop(channel_number);
  }
  
  if (VE_PerformShutdown[channel_number]==1)
  {
    VE_PerformShutdown[channel_number]=0;
    MotorSetValues[channel_number].TargetSpeed=VE_PerformShutdownTargetSpeed[channel_number];
    return 0;
  }

  if (xVETask[channel_number]!=NULL)
    return 0;

  if ((ParameterChange[channel_number] & VE_CHANGE_BOARD_PARAMS_PMD) != 0)
  {
    ADC_Init(channel_number,(CURRENT_MEASUREMENT)ChannelValues[channel_number].measurement_type);
    PMD_HandleParameterChangeBoard(channel_number);
    ParameterChange[channel_number] &= ~VE_CHANGE_BOARD_PARAMS_PMD;
  }

  VE_InitTimer(channel_number);
  VE_PrecalculateChannelValues(channel_number);                                 /* do pre-calculations */
  VE_PrecalculateMotorValues(channel_number);                                   /* do pre-calculations */
  VE_Init(channel_number);                                                      /* Initialize the vector engine */
  
  VE_ActualStage[channel_number].main = Stage_Bootstrap;                        /* startup */
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

