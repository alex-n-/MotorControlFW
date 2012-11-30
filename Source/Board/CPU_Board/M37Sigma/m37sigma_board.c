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

#include <stdint.h>
#include <stdlib.h>

#include "config.h"

#include TMPM_HEADER_FILE
#include TMPM_ADC_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#ifdef MOTOR_CHANNEL_0
#include BOARD_PWR_HEADER_FILE_0
#endif
#include BOARD_PWR_HEADER_FILE_1

#include "motorctrl.h"


/*! \brief  Detect Board Revision
  *
  * @retval Version Number of Board
*/

uint8_t BOARD_Detect_Revision(void)
{
  /* There is only Revision 1 available */
  return 1;
}

#ifdef USE_TEMPERATURE_CONTROL

#ifndef USE_HV_COMMUNICATION

/*! \brief  Get Temperature of FETs/IGBTs/Powermodule
  *
  * Using direct ADC Pin on MCU
  *
  * @param  channel_number:  channel to read out
  *
  * @retval Temperature in degree Celsius
*/
int8_t BOARD_GetTemperature(uint8_t channel_number)
{
  ADC_Result  result;
  uint8_t     i;

  result=ADC_GetConvertResult(TSB_ADB, ADC_REG3);
  
  for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
    if (temperature[i].adc<result.Bit.ADResult)
      break;
  
  if (i<sizeof(temperature)/sizeof(temp_table)-1)
  {
    int diff_adc_table,diff_measure;
    diff_adc_table=temperature[i].adc-temperature[i+1].adc;
    diff_measure  =temperature[i].adc-result.Bit.ADResult;
    
    return temperature[i].temperature + diff_measure*(temperature[i+1].temperature-temperature[i].temperature)/diff_adc_table;
  }
  return 0;
}

static ADC_MonitorTypeDef Overtemp= { ADC_CMPCR_0,
                                      ADC_REG3,
                                      5,
                                      ADC_SMALLER_THAN_CMP_REG,
                                      0};
static ADC_MonitorTypeDef ClearTemp={ ADC_CMPCR_1,
                                      ADC_REG3,
                                      5,
                                      ADC_LARGER_THAN_CMP_REG,
                                      0};


/*! \brief ConfigureADCforTemperature
  *
  * Set up monitoring for overtemperature detection
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
void BOARD_ConfigureADCforTemperature(uint8_t channel_number)
{
  uint16_t  adc_value;
  uint8_t   i;

  if (SystemValues[1].Overtemperature!=0)
  {
    uint16_t diff_adc_table;
    uint8_t  diff_temp;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>SystemValues[1].Overtemperature)
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-SystemValues[1].Overtemperature;

    adc_value = temperature[i-1].adc
              + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;

    Overtemp.CmpValue=adc_value;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>(SystemValues[1].Overtemperature-TEMP_SLOPE))
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-(SystemValues[1].Overtemperature-TEMP_SLOPE);

    adc_value = temperature[i-1].adc
              + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;

    ClearTemp.CmpValue=adc_value;
    
    
    ADC_SetMonitor(TSB_ADB, &Overtemp);
    ADC_SetMonitor(TSB_ADB, &ClearTemp);
    ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_1);
    
    NVIC_EnableIRQ(INTADBCPA_IRQn);
    NVIC_EnableIRQ(INTADBCPB_IRQn);
    
  }
  
  ADC_SetConstantTrg(TSB_ADB,ADC_REG3,TRG_ENABLE(ADC_REG3));
  ADC_Start(TSB_ADB,ADC_TRG_CONSTANT);
}
#else /* USE_HV_COMMUNICATION */

#include "hv_serial_communication.h"

/*! \brief  Get Temperature of FETs/IGBTs/Powermodule
  *
  * Using HV Communication protocol
  *
  * @param  channel_number:  channel to read out
  *
  * @retval Temperature in degree Celsius
*/
int8_t BOARD_GetTemperature(uint8_t channel_number)
{
  uint16_t result;
  uint8_t  i;

  result=HV_Communication_GetValue(1);
  
  for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
    if (temperature[i].adc<result)
      break;
  
  if (i<sizeof(temperature)/sizeof(temp_table)-1)
  {
    int diff_adc_table,diff_measure;
    diff_adc_table=temperature[i].adc-temperature[i+1].adc;
    diff_measure  =temperature[i].adc-result;
    
    return temperature[i].temperature + diff_measure*(temperature[i+1].temperature-temperature[i].temperature)/diff_adc_table;
  }
  return 0;
  
}

/*! \brief ConfigureADCforTemperature
  *
  * Set up monitoring for overtemperature detection
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
void BOARD_ConfigureADCforTemperature(uint8_t channel_number)
{
  uint16_t OvertempCmpValue = 0x3ff;
  uint16_t ClearTempCmpValue= 0x3ff;

  uint16_t  adc_value;
  uint8_t   i;

  if (SystemValues[1].Overtemperature!=0)
  {
    uint16_t diff_adc_table;
    uint8_t  diff_temp;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>SystemValues[1].Overtemperature)
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-SystemValues[1].Overtemperature;

    adc_value = temperature[i-1].adc
              + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;
    
    OvertempCmpValue=adc_value;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>(SystemValues[1].Overtemperature-TEMP_SLOPE))
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-(SystemValues[1].Overtemperature-TEMP_SLOPE);

    adc_value = temperature[i-1].adc
              + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;

    ClearTempCmpValue=adc_value;
  }
  
  HV_Communication_ConfigureTemperatureControl(OvertempCmpValue,ClearTempCmpValue);
  
}

#endif /* USE_HV_COMMUNICATION */

#endif /* USE_TEMPERATURE_CONTROL */
