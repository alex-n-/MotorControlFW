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

#include "FreeRTOS.h"
#include "task.h"

#include TMPM_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE

#include "eeprom.h"
#include "spi.h"
#include "motorctrl.h"
#include "debug.h"

static void eeprom_latch_in(void)
{
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, ENABLE);           /* Deselect CS */
  vTaskDelay( 1/ portTICK_RATE_MS);                                   /* Wait for 1 ms */
  GPIO_WriteDataBit(EEPROM_CS_PORT, EEPROM_CS_BIT, DISABLE);          /* Select CS */
}

uint8_t BOARD_Detect_Revision(void)
{
  unsigned char status;
  
  SPI_SelectDevice(SPI_DEVICE_EEPROM);                                /* Select SPI Device  (block access for other drivers) */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WREN);                      /* Send write enable to eeprom */
  eeprom_latch_in();                                                  /* Latch data into eeprom */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRITE);                     /* Send write command */
  eeprom_latch_in();                                                  /* Latch data into eeprom */
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_RDSR);                      /* Read out status register */
  status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);                        /* read out data */
  status = SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  eeprom_latch_in();

  SPI_TransmitByte(BOARD_SPI_CHANNEL, CMD_WRDI);                      /* Disable write enable */

  SPI_DeselectDevice();                                               /* Give SPI access back for other Tasks */
  
  return (status==STATUS_WEL?1:2);                                    /* If there is an EEProm it's revision 2 otherwise 1 */  
}

#ifdef USE_TEMPERATURE_CONTROL  
int8_t BOARD_GetTemperature(uint8_t channel_number)
{
  ADC_Result  result;
  uint8_t     i;

  switch (channel_number)
  {
  case 0:
    result=ADC_GetConvertResult(TSB_ADB, ADC_REG9);
    break;
  case 1:
    result=ADC_GetConvertResult(TSB_ADB, ADC_REG10);
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
    diff_adc_table=temperature[i].adc-temperature[i+1].adc;
    diff_measure  =temperature[i].adc-result.Bit.ADResult;
    
    return temperature[i].temperature + diff_measure*(temperature[i+1].temperature-temperature[i].temperature)/diff_adc_table;
  }
  return 0;
}

static ADC_MonitorTypeDef Overtemp= { ADC_CMPCR_0,
                                      ADC_REG10,
                                      5,
                                      ADC_SMALLER_THAN_CMP_REG,
                                      0};
static ADC_MonitorTypeDef ClearTemp={ ADC_CMPCR_1,
                                      ADC_REG10,
                                      5,
                                      ADC_LARGER_THAN_CMP_REG,
                                      0};


void BOARD_ConfigureADCforTemperature(uint8_t channel_number)
{
  uint16_t  adc_value;
  uint8_t   i;

  if (channel_number>1)
    assert_param(0);
  
  if (SystemValues[channel_number].Overtemperature!=0)
  {
    uint16_t diff_adc_table;
    uint8_t  diff_temp;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>SystemValues[1].Overtemperature)
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-SystemValues[1].Overtemperature;

    adc_value = temperature[i-1].adc + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;

    Overtemp.CmpValue=adc_value;
    
    for (i=0;i<sizeof(temperature)/sizeof(temp_table);i++)
      if (temperature[i].temperature>(SystemValues[1].Overtemperature-TEMP_SLOPE))
        break;
    
    diff_adc_table=temperature[i-1].adc-temperature[i].adc;
    diff_temp     =temperature[i-1].temperature-(SystemValues[1].Overtemperature-TEMP_SLOPE);

    adc_value = temperature[i-1].adc + diff_temp*(temperature[i-1].temperature-temperature[i].temperature)*diff_adc_table;

    ClearTemp.CmpValue=adc_value;
    
    switch (channel_number)
    {
    case 0:
      Overtemp.ResultREGx =ADC_REG9;
      ClearTemp.ResultREGx=ADC_REG9;
    case 1:                                                                   /* Fall trough due to board design */
      ADC_SetMonitor(TSB_ADB, &Overtemp);
      ADC_SetMonitor(TSB_ADB, &ClearTemp);
      ADC_DisableMonitor(TSB_ADB,ADC_CMPCR_1);
    
      NVIC_EnableIRQ(INTADBCPA_IRQn);
      NVIC_EnableIRQ(INTADBCPB_IRQn);
      break;
    default:
      assert_param(0);
      break;
    }
  }
  
  switch (channel_number)
  {
  case 0:
    ADC_SetConstantTrg(TSB_ADB,ADC_REG9,TRG_ENABLE(ADC_REG9));
    ADC_Start(TSB_ADB,ADC_TRG_CONSTANT);
    break;
  case 1:
    ADC_SetConstantTrg(TSB_ADB,ADC_REG10,TRG_ENABLE(ADC_REG10));
    ADC_Start(TSB_ADB,ADC_TRG_CONSTANT);
    break;
  default:
    assert_param(0);
    break;
  }
}
#endif /* USE_TEMPERATURE_CONTROL */

