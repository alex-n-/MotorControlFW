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

#ifdef __TMPM_370__
void INTADACPA_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_0);
  ADC_EnableMonitor (TSB_ADB,ADC_CMPCR_1);
  MotorErrorField[0].Error |= VE_OVERTEMPERATURE;
}
void INTADACPB_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_1);
  ADC_EnableMonitor (TSB_ADB,ADC_CMPCR_0);
  MotorErrorField[0].Error &= ~VE_OVERTEMPERATURE;
}
#endif /* __TMPM_370__ */

void INTADBCPA_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_0);
  ADC_EnableMonitor (TSB_ADB,ADC_CMPCR_1);
  MotorErrorField[1].Error |= VE_OVERTEMPERATURE;
}

void INTADBCPB_IRQHandler(void)
{
  ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_1);
  ADC_EnableMonitor (TSB_ADB,ADC_CMPCR_0);
  MotorErrorField[1].Error &= ~VE_OVERTEMPERATURE;
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
#ifdef __TMPM_370__
  case 0:
    pADC    = TSB_ADA;
    break;
#endif
  case 1:
    pADC    = TSB_ADB;
    break;
  default:
    assert_param(0);
    break;
  }

  switch (mesurement_type)
  {
  case CURRENT_SHUNT_3:
    switch (channel_number)
    {
#ifdef __TMPM_370__
    case 0:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL0 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL1 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL2 ,TRG_ENABLE(PMD_PROG2));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL3 ,TRG_ENABLE(PMD_PROG3));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL4 ,TRG_ENABLE(PMD_PROG4));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL5 ,TRG_ENABLE(PMD_PROG5));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_3ShuntA);
      break;
#endif
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL8 ,TRG_ENABLE(PMD_PROG2));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL9 ,TRG_ENABLE(PMD_PROG3));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL10,TRG_ENABLE(PMD_PROG4));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL11,TRG_ENABLE(PMD_PROG5));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_3ShuntB);
      break;
    default:
      assert_param(0);
      break;
    }
    
    ADC_SetPMDTrg(pADC, &PMDTrigger0_3Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger1_3Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger2_3Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger3_3Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger4_3Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger5_3Phase);
    
    break;
    
  case CURRENT_SHUNT_1:
    switch (channel_number)
    {
#ifdef __TMPM_370__
    case 0:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL0,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL1,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_1ShuntA);
      break;
#endif
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_1ShuntB);
      break;
    default:
      assert_param(0);
      break;
    }
    
    ADC_SetPMDTrg(pADC, &PMDTrigger0_1Phase);
    ADC_SetPMDTrg(pADC, &PMDTrigger1_1Phase);
    break;

  case CURRENT_SENSOR_2:
    switch (channel_number)
    {
#ifdef __TMPM_370__
    case 0:
      /* Not implement due to no available board */
      break;
#endif
    case 1:
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL6 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL7 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL8 ,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL9 ,TRG_ENABLE(PMD_PROG1));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL10,TRG_ENABLE(PMD_PROG0));
      ADC_SelectPMDTrgProgNum(pADC, PMD_TRG_PROG_SEL11,TRG_ENABLE(PMD_PROG1));
      ADC_SetPMDTrgProgINT(pADC, &TrgProgINT_2SensorB);

#ifndef BOARD_HITEX_M370
      ADC_SetPMDTrg(pADC, &PMDTrigger0_2Phase);
      ADC_SetPMDTrg(pADC, &PMDTrigger1_2Phase);
#endif /* BOARD_HITEX_M370 */    
      
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
  
#ifdef USE_TEMPERATURE_CONTROL  
  BOARD_ConfigureADCforTemperature(channel_number);
#endif  

  ADC_Enable(pADC);
  ADC_SetClk(pADC, ADC_HOLD_FIX, ADC_FC_DIVIDE_LEVEL_2);
  
}
