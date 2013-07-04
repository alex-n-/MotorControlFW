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
#include <string.h>

#include "config.h"

#include TMPM_HEADER_FILE
#include TMPM_ADC_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */

#include "motorctrl.h"
#include "temperature_measure_table.h"

#ifdef USE_TEMPERATURE_CONTROL

static uint8_t actual_temperature;

#ifndef USE_HV_COMMUNICATION

/*! \brief  Get Temperature of FETs/IGBTs/Powermodule
  *
  * Using direct ADC Pin on MCU
  *
  * @param  channel_number:  channel to read out
  *
  * @retval Temperature in degree Celsius
*/
int8_t TEMPERATURE_GetTemperature(uint8_t channel_number)
{
  ADC_Result         result;
  uint8_t            i;

	memset(&result,0,sizeof(result));
	
  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    result=ADC_GetConvertResult(TEMPERATURE_ADC0, TEMPERATURE_REG0);
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */    
  case 1:
    result=ADC_GetConvertResult(TEMPERATURE_ADC1, TEMPERATURE_REG1);
    break;
  default:
    assert_param(0);
    break;
  }

  for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
    if (temperature[i].adc<result.Bit.ADResult)
      break;
  
  if (i<sizeof(temperature)/sizeof(temp_table)-1)
  {
    int diff_adc_table,diff_measure;
    diff_adc_table      =temperature[i].adc-temperature[i+1].adc;
    diff_measure        =temperature[i].adc-result.Bit.ADResult;
    actual_temperature    =temperature[i].temperature + diff_measure*(temperature[i+1].temperature-temperature[i].temperature)/diff_adc_table;
    return actual_temperature;
  }
  
  return 0;
}

/*! \brief ConfigureADCforTemperature
  *
  * Set up continous measure of temperature channels
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
void TEMPERATURE_ConfigureADCforTemperature(uint8_t channel_number)
{
  switch (channel_number)
  {
#if defined __TMPM_370__ || defined __TMPM_376__
  case 0:
    ADC_SetConstantTrg(TEMPERATURE_ADC0,TEMPERATURE_REG0,TRG_ENABLE(TEMPERATURE_REG0));
    ADC_Start(TEMPERATURE_ADC0,ADC_TRG_CONSTANT);
    break;
#endif /* defined __TMPM_370__ || defined __TMPM_376__ */    
  case 1:
    ADC_SetConstantTrg(TEMPERATURE_ADC1,TEMPERATURE_REG1,TRG_ENABLE(TEMPERATURE_REG1));
    ADC_Start(TEMPERATURE_ADC1,ADC_TRG_CONSTANT);
    break;

  default:
    assert_param(0);
    break;
  }
}

/*! \brief Check Overtemperature
  *
  * Check for overtemperature and release temperature
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
void TEMPERATURE_CheckOvertemp(uint8_t channel_number)
{
  static uint8_t counter=0;
  int8_t temp;
  
  if (SystemValues[channel_number].Overtemperature == 0)
    return;
  
  temp = actual_temperature;
  
  if (temp>=SystemValues[channel_number].Overtemperature)
    counter++;
  
  if (counter>5)
    MotorErrorField[channel_number].Error |= VE_OVERTEMPERATURE;
  
  if (   ((MotorErrorField[channel_number].Error & VE_OVERTEMPERATURE) != 0)
      && (temp< (SystemValues[channel_number].Overtemperature-TEMP_SLOPE) ) )
  {
    counter=0;
    MotorErrorField[channel_number].Error &= ~VE_OVERTEMPERATURE;
  }
}

/*! \brief GetActualTemperature
  *
  * Get last measured temperature without reading ADC
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
uint8_t TEMPERATURE_GetActualTemperature(uint8_t channel_number)
{
    return TEMPERATURE_GetTemperature(channel_number);
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
int8_t TEMPERATURE_GetTemperature(uint8_t channel_number)
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
    diff_adc_table     = temperature[i].adc-temperature[i+1].adc;
    diff_measure       = temperature[i].adc-result;
    actual_temperature = temperature[i].temperature + diff_measure*(temperature[i+1].temperature-temperature[i].temperature)/diff_adc_table;
    return actual_temperature;
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
void TEMPERATURE_ConfigureADCforTemperature(uint8_t channel_number)
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

/*! \brief Check Overtemperature
  *
  * Check for overtemperature and release temperature
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
void TEMPERATURE_CheckOvertemp(uint8_t channel_number)
{
  
  /* Nothing to do here as done by hv_serial_communication */
}

/*! \brief GetActualTemperature
  *
  * Get last measured temperature without reading ADC
  *
  * @param  channel_number:  channel to configure
  *
  * @retval None
*/
uint8_t TEMPERATURE_GetActualTemperature(uint8_t channel_number)
{
    return TEMPERATURE_GetTemperature(channel_number);
}


#endif /* USE_HV_COMMUNICATION */

#endif /* USE_TEMPERATURE_CONTROL */
