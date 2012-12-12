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
#include <string.h>

#include "config.h"
#include TMPM_GPIO_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "pmd.h"
#include "motorctrl.h"
#include "dso.h"
#include "hsdso.h"
#include "ve.h"
#include "board.h"

uint8_t normal_operation=0;

#ifndef USE_OVERVOLTAGE_SIGNAL
/*! \brief  Disable Overvoltage
  *
  * Do not react on overvoltage signal
  *
  * @param  uint8_t channel_number: PMD channel to use
  * @retval None  
*/
static void PMD_OvervoltageDetectDisable(uint8_t channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  uint16_t  mdout;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  if (pPMDx->EMGSTA & PMD_EMG_STATUS_INPUT)
  {
    mdout         = pPMDx->MDOUT;
    pPMDx->MDOUT  = 0;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    
    pPMDx->EMGREL = (uint8_t) (PMD_EMG_KEY >> 8);
    pPMDx->EMGREL = (uint8_t) PMD_EMG_KEY;

    pPMDx->OVVCR &= ~PMD_EMGCR_ENABLE;
    pPMDx->MDOUT  = mdout;
  }
}

#else

/*! \brief  Reset Overvoltage
  *
  * Reset the overvoltage detection
  *
  * @param  channel_number: PMD channel to use
  * @retval None  
*/
void PMD_OvervoltageReset(uint8_t channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  uint16_t  mdout;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }
    
  if (pPMDx->EMGSTA & PMD_EMG_STATUS_INPUT)
  {
    mdout         = pPMDx->MDOUT;
    pPMDx->MDOUT  = 0;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    
    pPMDx->EMGREL = (uint8_t) (PMD_EMG_KEY >> 8);
    pPMDx->EMGREL = (uint8_t) PMD_EMG_KEY;

    pPMDx->OVVCR &= ~PMD_EMGCR_ENABLE;
    pPMDx->OVVCR |= PMD_EMGCR_ENABLE;
    pPMDx->MDOUT  = mdout;
  }
}

/*! \brief  Enable Overvoltage Detect
  *
  * Enable the overvoltage detection
  *
  * @param  channel_number: PMD channel to use
  * @retval None  
*/
static void PMD_OvervoltageEnable(uint8_t channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  pPMDx->OVVCR =  PMD_OVVCR_ENABLE
                | PMD_OVVCR_PROT_REL
                | PMD_OVVCR_PROT_MODE_LOWER;
}

/*! \brief  Interrupt handler of PMD OVV Channel 0
  *
  * @param  None
  * @retval None  
*/
void INTOVV0_IRQHandler (void)
{
  if (normal_operation==1)
  {
    MotorErrorField[0].Error |= VE_OVERVOLTAGE;
    VE_ActualStage[0].main  = Stage_Emergency;                                  /* emergency case exists */
  }
}

/*! \brief  Interrupt handler of PMD OVV Channel 1
  *
  * @param  None
  * @retval None  
*/
void INTOVV1_IRQHandler (void)
{
  if (normal_operation==1)
  {
    MotorErrorField[1].Error |= VE_OVERVOLTAGE;
    VE_ActualStage[1].main  = Stage_Emergency;                                  /* emergency case exists */
  }
}
#endif /* USE_OVERVOLTAGE_SIGNAL */

#ifndef USE_EMERGENCY_SIGNAL
/*! \brief  Disable Emergency
  *
  * Do not react on emergency signal
  *
  * @param  channel_number: PMD channel to use
  * @retval None  
*/
static void PMD_EmergencyDisable(uint8_t channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  uint16_t  mdout;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  if (pPMDx->EMGSTA & PMD_EMG_STATUS_INPUT)
  {
    mdout         = pPMDx->MDOUT;
    pPMDx->MDOUT  = 0;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    
    pPMDx->EMGREL = (uint8_t) (PMD_EMG_KEY >> 8);
    pPMDx->EMGREL = (uint8_t) PMD_EMG_KEY;

    pPMDx->EMGCR &= ~PMD_EMGCR_ENABLE;
    pPMDx->MDOUT  = mdout;
  }
}
#else

/*! \brief  Emergency Reset
  *
  * Reset the Emergency Detection
  *
  * @param  channel_number: PMD channel to use
  * @retval None  
*/
void PMD_EmergencyReset(uint8_t channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  uint16_t  mdout;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  if (pPMDx->EMGSTA & PMD_EMG_STATUS_INPUT)
  {
    mdout         = pPMDx->MDOUT;
    pPMDx->MDOUT  = 0;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    pPMDx->EMGCR |= PMD_EMGCR_PROT_REL;
    
    pPMDx->EMGREL = (uint8_t) (PMD_EMG_KEY >> 8);
    pPMDx->EMGREL = (uint8_t) PMD_EMG_KEY;

    pPMDx->EMGCR &= ~PMD_EMGCR_ENABLE;
    pPMDx->EMGCR |= PMD_EMGCR_ENABLE;
    
    pPMDx->MDOUT  = mdout;
  }
}

/*! \brief  Interrupt handler of PMD Emergency Channel 0
  *
  * @param  None
  * @retval None  
*/
void INTEMG0_IRQHandler (void)
{
  if (normal_operation==1)
  {
    MotorErrorField[0].Error |= VE_EMERGENCY;
    VE_ActualStage[0].main  = Stage_Emergency;                                  /* emergency case exists */
  }
}

/*! \brief  Interrupt handler of PMD Emergency Channel 1
  *
  * @param  None
  * @retval None  
*/
void INTEMG1_IRQHandler (void)
{
  if (normal_operation==1)
  {
    MotorErrorField[1].Error |= VE_EMERGENCY;
    VE_ActualStage[1].main  = Stage_Emergency;                                  /* emergency case exists */
  }
}
#endif /* USE_EMERGENCY_SIGNAL */

static const GPIO_InitTypeDef GPIO_Init_Struct_Output =
{
  GPIO_OUTPUT_MODE,         /* Enable as output */
  GPIO_PULLUP_DISABLE,      /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

static const GPIO_InitTypeDef GPIO_Init_Struct_Input =
{
  GPIO_INPUT_MODE,          /* Enable as input */
  GPIO_PULLUP_ENABLE,       /* Enable/disable pull up */
  GPIO_OPEN_DRAIN_DISABLE,  /* Open drain or CMOS */
  GPIO_PULLDOWN_DISABLE,    /* Enable/disable pull down */
};

/*! \brief  Initialize the PMD output pins
  *
  * Switch the according pins to PMD output
  *
  * @param  channel_number: channel to use
  * @retval None  
*/
static void PMD_IOInit(unsigned char channel_number)
{
  GPIO_Port         USE_Port = GPIO_PG;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    USE_Port = GPIO_PC;
    break;
#endif
  case 1:
    USE_Port = GPIO_PG;
    break;
  default:
    assert_param(0);
    break;
  }

  GPIO_Init(USE_Port,
            GPIO_BIT_0 |
            GPIO_BIT_1 |
            GPIO_BIT_2 |
            GPIO_BIT_3 |
            GPIO_BIT_4 |
            GPIO_BIT_5,
            &GPIO_Init_Struct_Output
            );

  GPIO_EnableFuncReg(USE_Port, GPIO_FUNC_REG_1, GPIO_BIT_0
                                              | GPIO_BIT_1
                                              | GPIO_BIT_2
                                              | GPIO_BIT_3
                                              | GPIO_BIT_4
                                              | GPIO_BIT_5);
  
  GPIO_Init(USE_Port,
            GPIO_BIT_6,
            &GPIO_Init_Struct_Input
            );
  GPIO_EnableFuncReg(USE_Port, GPIO_FUNC_REG_1, GPIO_BIT_6);

  GPIO_Init(USE_Port,
            GPIO_BIT_7,
            &GPIO_Init_Struct_Input
            );
  GPIO_EnableFuncReg(USE_Port, GPIO_FUNC_REG_1, GPIO_BIT_7);
}

/*! \brief  PMD_HandleParameterChangeSystem
  *
  * Handle change in System parameters
  *
  * @param  channel_number: channel to use
  * @retval None  
*/
static void PMD_HandleParameterChangeSystem(uint8_t channel_number)
{
  TSB_PMD_TypeDef*  pPMD  = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMD    = TSB_PMD0;
    break;
#endif
  case 1:
    pPMD    = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  pPMD->MDPRD = (T0/SystemValues[channel_number].PWMFrequency);
}

/*! \brief  Normal output for the PMD Signals
  *
  * Normal state while motor is turning
  *
  * @param  channel_number: channel to use
  * @retval None  
*/
void PMD_NormalOutput(unsigned char channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  pPMDx->MDOUT= PMD_MDOUT_PWM_U
              | PMD_MDOUT_PWM_V
              | PMD_MDOUT_PWM_W
              | PMD_MDOUT_OC_U3
              | PMD_MDOUT_OC_V3
              | PMD_MDOUT_OC_W3; 
}  

/*! \brief  Short the motor lines
  *
  * Short the motor lines to spin down the motor as fast as possible
  *
  * @param  channel_number: channel to be shorten
  * @retval None  
*/
void PMD_ShortBrake(unsigned char channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }
 
  pPMDx->MDOUT= 0;                                                              /* put the motor driver offline */
}  

/*! \brief  Switch off the PMD Signals to the Motor
  *
  * Switch off the signals to let the motor run out
  *
  * @param  channel_number: channel to be switched off
  * @retval None  
*/
void PMD_SwitchOff(unsigned char channel_number)
{
  TSB_PMD_TypeDef*    pPMDx   = NULL;
  
  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMDx   = TSB_PMD0;
    break;
#endif
  case 1:
    pPMDx   = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  pPMDx->MDOUT= 0;                                                              /* put the motor driver offline */

}

/*! \brief  Interrupt handler of PMD Channel 0
  *
  * @param  None
  * @retval None  
*/
void INTPMD0_IRQHandler (void)
{
  if ((ParameterChange[0] & VE_CHANGE_SYSTEM_PARAMS_PMD) != 0)
  {
    PMD_HandleParameterChangeSystem(0);
    ParameterChange[0] &= ~VE_CHANGE_SYSTEM_PARAMS_PMD;
  }
  
#ifdef USE_DSO
  DSO_Log (PMD_CALLING , TEE_VE0 );
#endif
#ifdef USE_HSDSO
  HsDSO_Log_PMD(0);
#endif 
}

/*! \brief  Interrupt handler of PMD Channel 1
  *
  * @param  None
  * @retval None  
*/
void INTPMD1_IRQHandler (void)
{
  if ((ParameterChange[1] & VE_CHANGE_SYSTEM_PARAMS_PMD) != 0)
  {
    PMD_HandleParameterChangeSystem(1);
    ParameterChange[1] &= ~VE_CHANGE_SYSTEM_PARAMS_PMD;
  }

#ifdef USE_DSO
  DSO_Log (PMD_CALLING , TEE_VE1 );
#endif
#ifdef USE_HSDSO
  HsDSO_Log_PMD(1);
#endif
}

/*! \brief  PMD_HandleParameterChangeBoard
  *
  * Handle change in Board parameters
  *
  * @param  channel_number: channel to use
  * @retval None  
*/
void PMD_HandleParameterChangeBoard(uint8_t channel_number)
{
  TSB_PMD_TypeDef*  pPMD  = NULL;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMD    = TSB_PMD0;
    break;
#endif
  case 1:
    pPMD    = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }

  pPMD->MDEN &= ~PMD_ENABLE;
  
  /* PMD Output Setting Register */
  pPMD->MDPOT    = PMD_PSYNC_ASYNC |
                   ChannelValues[channel_number].poll<<2 |
                   ChannelValues[channel_number].polh<<3 ;

  pPMD->DTR      = ChannelValues[channel_number].DeadTime / 100;
                                                                                /* Set Dead Time Register(100ns@80MHz) */
  switch(ChannelValues[channel_number].measurement_type)
  {
  case CURRENT_SHUNT_1:
    pPMD->TRGCR  = PMD_TRG_1SHUNT;                                              /* Trigger Control Register */
    pPMD->TRGMD  = PMD_EMG_TRG_PROT_ENABLE;                                     /* Trigger Output Mode Setting */
    pPMD->TRGSEL = PMD_TRGSEL_PMDTRG0;                                          /* Trigger Output Select Register */
    break;
  case CURRENT_SHUNT_3:
    pPMD->TRGCR  = PMD_TRG_3SHUNT;                                              /* Trigger is PWM peek */
    pPMD->TRGMD  = PMD_TRG_MODE_VARIABLE;                                       /* TRG0 change to TRGx (x:Sector No) */
    pPMD->TRGSEL = PMD_TRGSEL_PMDTRG0;                                          /* TRG No = Sector = 0r */
    break;
  case CURRENT_SENSOR_2:
    pPMD->TRGCR  = PMD_TRG_2SENSOR;                                             /* Trigger is PWM peek */
    pPMD->TRGMD  = PMD_TRG_MODE_VARIABLE;                                       /* TRG0 change to TRGx (x:Sector No) */
    pPMD->TRGSEL = PMD_TRGSEL_PMDTRG0;                                          /* TRG No = Sector = 0r */
    break;
  default:
    assert_param(0);
    break;
  }

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    NVIC_SetPriority(INTPMD0_IRQn, INTERRUPT_PRIORITY_PMD);
    NVIC_EnableIRQ(INTPMD0_IRQn);   

#ifdef USE_EMERGENCY_SIGNAL
    NVIC_SetPriority(INTPMD1_IRQn, INTERRUPT_PRIORITY_ERROR);
    NVIC_EnableIRQ(INTEMG0_IRQn);
    PMD_EmergencyReset(channel_number);
#else
    PMD_EmergencyDisable(channel_number);
#endif /* USE_EMERGENCY_SIGNAL */
    
#ifdef USE_OVERVOLTAGE_SIGNAL    
    NVIC_SetPriority(INTPMD1_IRQn, INTERRUPT_PRIORITY_ERROR);
    NVIC_EnableIRQ(INTOVV0_IRQn);
//    PMD_OvervoltageEnable(TSB_PMD0);  /* Needed for the 2 times LV board ... real HV has to be tested */
    PMD_OvervoltageReset(channel_number);
#else
    PMD_OvervoltageDetectDisable(channel_number);
#endif /* USE_OVERVOLTAGE_SIGNAL */
    break;
#endif  /* __TMPM_370__ */
    
  case 1:
    NVIC_SetPriority(INTPMD1_IRQn, INTERRUPT_PRIORITY_PMD);
    NVIC_EnableIRQ(INTPMD1_IRQn);

#ifdef USE_EMERGENCY_SIGNAL
    NVIC_SetPriority(INTPMD1_IRQn, INTERRUPT_PRIORITY_ERROR);
    NVIC_EnableIRQ(INTEMG1_IRQn);
    PMD_EmergencyReset(channel_number);
#else
    PMD_EmergencyDisable(channel_number);
#endif /* USE_EMERGENCY_SIGNAL */

#ifdef USE_OVERVOLTAGE_SIGNAL    
    NVIC_SetPriority(INTPMD1_IRQn, INTERRUPT_PRIORITY_ERROR);
    NVIC_EnableIRQ(INTOVV1_IRQn);
    PMD_OvervoltageEnable(channel_number);
    PMD_OvervoltageReset(channel_number);
#else    
    PMD_OvervoltageDetectDisable(channel_number); 
#endif /* USE_OVERVOLTAGE_SIGNAL */
    break;
  default:
    assert_param(0);
    break;
  }
  
  pPMD->MDEN |= PMD_ENABLE;
}

/*! \brief  Initialize the PMD Unit
  *
  * @param  channel_number: channel to configure
  * @param  deadtime:       Deadtime of the FETs
  * @param  shunt_type:     selects between 1-shunt & 3-shunt mode
  * @retval None  
*/
void PMD_Init (uint8_t channel_number)
{
  TSB_PMD_TypeDef* pPMD= NULL;
  normal_operation     = 0;

  switch (channel_number)
  {
#ifdef __TMPM_370__
  case 0:
    pPMD    = TSB_PMD0;
    break;
#endif
  case 1:
    pPMD    = TSB_PMD1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  PMD_IOInit(channel_number);
  
  pPMD->PORTMD   = PMD_OUT_ALL_PMD;                                             /* Port Output Mode Register */

  pPMD->MDCR     = PMD_CR_CARRIER_TRIANG |
                   PMD_CR_IRQ_PERIODE_1 |
                   PMD_CR_IRQ_COUNT_MDPRD |
                   PMD_CR_DUTY_INDEPEND |
                   PMD_CR_SYNTMD_LOW |
                   PMD_CR_PWM_EXPER_0;

  /* PMD Control Register */
  PMD_HandleParameterChangeSystem(channel_number);
                                                                                /* PWM Period Register */
  pPMD->MODESEL  = PMD_MODE_VE;                                                 /* Mode Select Register */
  pPMD->MDOUT    = 0;                                                           /* PMD Output Control Register */

  PMD_HandleParameterChangeBoard(channel_number);
  
  vTaskDelay( 10 / portTICK_RATE_MS );
  normal_operation=1;

}


