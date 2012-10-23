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
#include BOARD_PGA_HEADER_FILE

#include "motorctrl.h"

#define TEMP_SLOPE 5

static const int revisions[2][2] = {{3700,1},
                              {2050,2},
                             };

#ifdef USE_TEMPERATURE_CONTROL
/* Temperature Table for Vishay NTCLE100E3103JB0 */
static const temp_table temperature[]={
                                  {0xF87,-40},
                                  {0xF5B,-35},
                                  {0xF22,-30},
                                  {0xED9,-25},
                                  {0xE7E,-20},
                                  {0xE0F,-15},
                                  {0xD89,-10},
                                  {0xCEE, -5},
                                  {0xC3D,  0},
                                  {0xB78,  5},
                                  {0xAA4, 10},
                                  {0x9C6, 15},
                                  {0x8E2, 20},
                                  {0x800, 25},
                                  {0x723, 30},
                                  {0x652, 35},
                                  {0x590, 40},
                                  {0x4DE, 45},
                                  {0x43D, 50},
                                  {0x3AE, 55},
                                  {0x330, 60},
                                  {0x2C2, 65},
                                  {0x263, 70},
                                  {0x210, 75},
                                  {0x1C9, 80},
                                  {0x18C, 85},
                                  {0x157, 90},
                                  {0x12A, 95},
                                  {0x104,100},
                                  {0x0E3,105},
                                  {0x0C6,110},
                                  {0x0AE,115},
                                  {0x098,120},
                                  {0x086,125},
                                };
#endif /* USE_TEMPERATURE_CONTROL */

uint8_t BOARD_Detect_Revision(void)
{
  ADC_Result  result;
  uint8_t     i;
  
  PGA_SetChannel(SPI_DEVICE_AMP_V_I_DCBUS, CHANNEL1);
  
  ADC_Enable(TSB_ADB);  
  ADC_SetClk(TSB_ADB, ADC_HOLD_FIX, ADC_FC_DIVIDE_LEVEL_2);
  ADC_SetSWTrg(TSB_ADB, ADC_REG2, TRG_ENABLE(ADC_REG2));
  ADC_Start(TSB_ADB, ADC_TRG_SW);
  while (ADC_GetConvertState(TSB_ADB, ADC_TRG_SW) == BUSY);
  result=ADC_GetConvertResult(TSB_ADB, ADC_REG2);
  ADC_Disable(TSB_ADB);  

  PGA_SetChannel(SPI_DEVICE_AMP_V_I_DCBUS, CHANNEL0);
  
  for (i=0;i<sizeof(revisions)/sizeof(revisions[0]);i++)
    if (abs((revisions[i][0]-result.Bit.ADResult)<100))
      break;
  
  return revisions[i][1];
}

#ifdef USE_TEMPERATURE_CONTROL
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

#endif /* USE_TEMPERATURE_CONTROL */
