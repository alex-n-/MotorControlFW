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
#include BOARD_BOARD_HEADER_FILE
#include TMPM_HEADER_FILE
#include TMPM_ADC_HEADER_FILE

#include "board.h"
#include "adc.h"
#include "motorctrl.h"


#if defined __TMPM_370__ || defined __TMPM_376__
static ADC_MonitorTypeDef UndervoltageA = { ADC_CMPCR_0,
                                            VDC_MEASURE0_REG,
                                            VDC_MEASURE_TIMES,
                                            ADC_SMALLER_THAN_CMP_REG,
                                            0};
static ADC_MonitorTypeDef OvervoltageA  = { ADC_CMPCR_1,
                                            VDC_MEASURE0_REG,
                                            VDC_MEASURE_TIMES,
                                            ADC_LARGER_THAN_CMP_REG,
                                            0};
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */

static ADC_MonitorTypeDef UndervoltageB = { ADC_CMPCR_0,
                                            VDC_MEASURE1_REG,
                                            VDC_MEASURE_TIMES,
                                            ADC_SMALLER_THAN_CMP_REG,
                                            0};
static ADC_MonitorTypeDef OvervoltageB  = { ADC_CMPCR_1,
                                            VDC_MEASURE1_REG,
                                            VDC_MEASURE_TIMES,
                                            ADC_LARGER_THAN_CMP_REG,
                                            0};

#if defined __TMPM_370__ || defined __TMPM_376__
void INTADACPA_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADA,ADC_CMPCR_0);
  MotorErrorField[0].Error |= VE_SWUNDERVOLTAGE;
  VE_ActualStage[1].main = Stage_Emergency;
}
void INTADACPB_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADA,ADC_CMPCR_1);
  MotorErrorField[0].Error |= VE_SWOVERVOLTAGE;
  VE_ActualStage[0].main = Stage_Emergency;
}
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */

void INTADBCPA_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_0);
  MotorErrorField[1].Error |= VE_SWUNDERVOLTAGE;
  VE_ActualStage[1].main = Stage_Emergency;
}

void INTADBCPB_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_1);
  MotorErrorField[1].Error |= VE_SWOVERVOLTAGE;
  VE_ActualStage[1].main = Stage_Emergency;
}

/*! \brief  Setup ADC Compare registers
  *
  * @param  channel_number:  channel to control
  * @retval None  
*/
void ADC_OverUndervoltageDetect(uint8_t channel_number)
{
  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    if (SystemValues[channel_number].SW_Undervoltage != 0)
    {
      UndervoltageA.CmpValue = 0xfff * SystemValues[channel_number].SW_Undervoltage / VE_v_max[channel_number];
      ADC_SetMonitor(TSB_ADA, &UndervoltageA);
      NVIC_EnableIRQ(INTADACPA_IRQn);
    }
    else
      NVIC_DisableIRQ(INTADACPA_IRQn);

    if (SystemValues[channel_number].SW_Overvoltage != 0)
    {
      OvervoltageA.CmpValue  = 0xfff * SystemValues[channel_number].SW_Overvoltage  / VE_v_max[channel_number];
      ADC_SetMonitor(TSB_ADA, &OvervoltageA);
      NVIC_EnableIRQ(INTADACPB_IRQn);
    }
    else
      NVIC_DisableIRQ(INTADACPB_IRQn);

    ADC_SetConstantTrg(TSB_ADA,VDC_MEASURE0_REG,TRG_ENABLE(AIN_VDC0));
    ADC_Start(TSB_ADA,ADC_TRG_CONSTANT);
   break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    if (SystemValues[channel_number].SW_Undervoltage != 0)
    {
      UndervoltageB.CmpValue = 0xfff * SystemValues[channel_number].SW_Undervoltage * ChannelValues[channel_number].sensitivity_voltage_measure / 5000 ;
      ADC_SetMonitor(TSB_ADB, &UndervoltageB);
      NVIC_EnableIRQ(INTADBCPA_IRQn);
    }
    else
      NVIC_DisableIRQ(INTADBCPA_IRQn);

    if (SystemValues[channel_number].SW_Overvoltage != 0)
    {
      OvervoltageB.CmpValue  = 0xfff * SystemValues[channel_number].SW_Overvoltage  * ChannelValues[channel_number].sensitivity_voltage_measure / 5000;
      ADC_SetMonitor(TSB_ADB, &OvervoltageB);
      NVIC_EnableIRQ(INTADBCPB_IRQn);
    }
    else
      NVIC_DisableIRQ(INTADBCPB_IRQn);

    ADC_SetConstantTrg(TSB_ADB,VDC_MEASURE1_REG,TRG_ENABLE(AIN_VDC1));
    ADC_Start(TSB_ADB,ADC_TRG_CONSTANT);
    break;
  default:
    assert_param(0);
    break;
  }
}

/*! \brief  Get VDC voltage value
  *
  * @param  channel_number:  channel read
  * @retval None  
*/
uint16_t ADC_GetVDC(uint8_t channel_number)
{
  ADC_Result         result;
  
  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    result=ADC_GetConvertResult(TSB_ADA, VDC_MEASURE0_REG);
   break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    result=ADC_GetConvertResult(TSB_ADB, VDC_MEASURE1_REG);
    break;
  default:
    assert_param(0);
    break;
  }
  
  return result.Bit.ADResult;
}


/*! \brief  Initialize the ADC
  *
  * @param  channel_number:  channel to configure
  * @param  mesurement_type: selects between 1-shunt, 3-shunt or 2 sensor mode
  * @retval None  
*/
void ADC_Init (uint8_t channel_number, CURRENT_MEASUREMENT mesurement_type)
{
  TSB_AD_TypeDef*         pADC=NULL;

  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    pADC    = TSB_ADA;
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
  case 1:
    pADC    = TSB_ADB;
    break;
  default:
    assert_param(0);
    break;
  }

  ADC_Disable(pADC);
  
  switch (mesurement_type)
  {
  case CURRENT_SHUNT_3:
    switch (channel_number)
    {
#if defined __TMPM_370__ || defined __TMPM_376__
    case 0:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL0 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL1 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL2 ,TRG_ENABLE(PMD_PROG2));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL3 ,TRG_ENABLE(PMD_PROG3));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL4 ,TRG_ENABLE(PMD_PROG4));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL5 ,TRG_ENABLE(PMD_PROG5));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_3ShuntA);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_3PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_3PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger2_3PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger3_3PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger4_3PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger5_3PhaseA);
      break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL8 ,TRG_ENABLE(PMD_PROG2));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL9 ,TRG_ENABLE(PMD_PROG3));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL10,TRG_ENABLE(PMD_PROG4));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL11,TRG_ENABLE(PMD_PROG5));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_3ShuntB);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_3PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_3PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger2_3PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger3_3PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger4_3PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger5_3PhaseB);
      break;
    default:
      assert_param(0);
      break;
    }
        
    break;
    
  case CURRENT_SHUNT_1:
    switch (channel_number)
    {
#if defined __TMPM_370__ || defined __TMPM_376__
    case 0:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL0,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL1,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_1ShuntA);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_1PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_1PhaseA);
      break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_1ShuntB);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_1PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_1PhaseB);
      break;
    default:
      assert_param(0);
      break;
    }
    break;

#ifndef BOARD_HITEX_M370    
  case CURRENT_SENSOR_2:
    switch (channel_number)
    {
#if defined __TMPM_370__ || defined __TMPM_376__
    case 0:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL0 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL1 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL2 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL3 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL4 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL5 ,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_2SensorA);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_2PhaseA);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_2PhaseA);
      break;
      break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL8 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL9 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL10,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL11,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_2SensorB);
      ADC_SetPMDTrg(pADC, &PMDTrigger0_2PhaseB);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_2PhaseB);
      break;
    default:
      assert_param(0);
      break;
    }
    break;
#endif /* BOARD_HITEX_M370 */   
  default:
    assert_param(0);
    break;
  }
  
  ADC_Enable(pADC);
  ADC_SetClk(pADC, ADC_HOLD_FIX, ADC_FC_DIVIDE_LEVEL_2);
  
}
