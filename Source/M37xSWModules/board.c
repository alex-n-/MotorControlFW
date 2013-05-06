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
#include TMPM_ADC_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#include BOARD_LED_HEADER_FILE
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

#ifndef BOARD_M37SIGMA
#include BOARD_SPI_HEADER_FILE
#endif /* BOARD_M37SIGMA */

#ifdef USE_RGB_LED
#include BOARD_RGB_LED_HEADER_FILE
#endif /* USE_RGB_LED */


#if ( (defined BOARD_M372STK) || (defined BOARD_M374STK) )
#include BOARD_PGA_HEADER_FILE
#endif /* (defined BOARD_M372STK) || (defined BOARD_M374STK) */

#ifdef BOARD_HITEX_M370
#include "hitex_m370_lcd.h"
#include "hitex_m370_keypad.h"
#include "hitex_m370_gain.h"
#endif /* BOARD_HITEX_M370 */

#include "board.h"
#include "eeprom.h"
#include "spi.h"
#include "pmd.h"
#include "motorctrl.h"
#include "i2c_master_bitbanging.h"
#include "external_speed_control.h"
#include "hv_serial_communication.h"
#include "temperature_control.h"

uint8_t   BoardRevision = 0;
uint8_t   INIT_Done     = 0;
uint32_t  T0            = 0;
FWVersion FirmwareVersion;

/*! \brief  Setup the Board Hardware
  *
  * @param  None
  * @retval None
*/
void BOARD_SetupHW(void)
{
  /* Get the peripheral I/O clock frequency */
  const uint32_t a = 2U;
  SystemCoreClockUpdate();
  T0 = SystemCoreClock / (a << ((TSB_CG->SYSCR >> 8) & 7U));

  FirmwareVersion.fw_version[0]=FW_VERSION_MAJOR;
  FirmwareVersion.fw_version[1]=FW_VERSION_MINOR;
  
#ifndef BOARD_M37SIGMA
  SPI_DeviceInit(SPI_10_MHZ);                                                   /* Init SPI Channel */
#endif /* BOARD_M37SIGMA */
  BOARD_SetupGain();                                                            /* Set up the gain for current measure */
  
  BoardRevision = BOARD_Detect_Revision();

#ifdef BOARD_PWR_HEADER_FILE_0                                                          /* Set up Board dependant values */
#include BOARD_PWR_HEADER_FILE_0
  ChannelValues[0].DeadTime                    = BOARD_DEAD_TIME;
  ChannelValues[0].gain_current_measure        = BOARD_GAIN_CURRENT_MEASURE;
  ChannelValues[0].measurement_type            = BOARD_MEASUREMENT_TYPE;
  ChannelValues[0].sensitivity_voltage_measure = BOARD_SENSITIVITY_VOLTAGE_MEASURE;
  ChannelValues[0].sensitivity_current_measure = BOARD_SENSITIVITY_CURRENT_MEASURE;

#ifdef BOARD_SENSOR_DIRECTION
  ChannelValues[0].sensor_direction            = BOARD_SENSOR_DIRECTION;
#endif /* BOARD_SENSOR_DIRECTION */  
  ChannelValues[0].poll                        = BOARD_POLL;
  ChannelValues[0].polh                        = BOARD_POLH;
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */ 

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
  ChannelValues[1].DeadTime                    = BOARD_DEAD_TIME;
  ChannelValues[1].gain_current_measure        = BOARD_GAIN_CURRENT_MEASURE;
  ChannelValues[1].measurement_type            = BOARD_MEASUREMENT_TYPE;
  ChannelValues[1].sensitivity_voltage_measure = BOARD_SENSITIVITY_VOLTAGE_MEASURE;
  ChannelValues[1].sensitivity_current_measure = BOARD_SENSITIVITY_CURRENT_MEASURE;
    
#ifdef BOARD_SENSOR_DIRECTION
  ChannelValues[1].sensor_direction            = BOARD_SENSOR_DIRECTION;
#endif /* BOARD_SENSOR_DIRECTION */
  ChannelValues[1].poll                        = BOARD_POLL;
  ChannelValues[1].polh                        = BOARD_POLH;
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */
  
#ifdef USE_LED
  LED_Init();                                                                   /* Init LED Ports */
#endif /* USE_LED */
  
#ifdef USE_KEYPAD  
  KEYPAD_Init();                                                                /* Init Keypad Port */
#endif /* USE_KEYPAD */
  
#ifdef USE_LCD  
  LCD_Init();                                                                   /* Init LCD */
#endif /* USE_LCD */   
  
#ifdef USE_HV_COMMUNICATION
  HV_SerialCommunicationInit();
#endif /* USE_HV_COMMUNICATION */  

#ifdef USE_I2C_MASTER
  I2C_Init();
#endif /* USE_I2C_MASTER */

#ifdef USE_RGB_LED
  RGB_LED_Init();
#endif /* USE_RGB_LED */
  
}

void BOARD_SetupHW2(void)
{
#if defined __TMPM_370__  || defined __TMPM_376__
  ADC_Init(0,(CURRENT_MEASUREMENT)ChannelValues[0].measurement_type);           /* enable, configure the ADC */
  PMD_Init(0);
#ifdef USE_TEMPERATURE_CONTROL
  TEMPERATURE_ConfigureADCforTemperature(0);
#endif
#ifndef BOARD_VDC_CHANNEL_0 
#ifdef USE_SW_OVER_UNDER_VOLTAGE_DETECTION
#ifdef HV_COMM
  HV_Communication_OverUndervoltageDetect(0);
#else
  ADC_OverUndervoltageDetect(0);
#endif /* USE_HV_COMMUNICATION */
#endif /* USE_SW_OVER_UNDER_VOLTAGE_DETECTION */
#endif /* !defined BOARD_VDC_CHANNEL_0 */
  
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */

  ADC_Init(1,(CURRENT_MEASUREMENT)ChannelValues[1].measurement_type);           /* enable, configure the ADC */
  PMD_Init(1);
#ifdef USE_TEMPERATURE_CONTROL
  TEMPERATURE_ConfigureADCforTemperature(1);
#endif
#ifndef BOARD_VDC_CHANNEL_1
#ifdef USE_SW_OVER_UNDER_VOLTAGE_DETECTION
#ifdef HV_COMM
  HV_Communication_OverUndervoltageDetect(1);
#else
  ADC_OverUndervoltageDetect(1);
#endif /* USE_HV_COMMUNICATION */
#endif /* USE_SW_OVER_UNDER_VOLTAGE_DETECTION */
#endif /* !defined BOARD_VDC_CHANNEL_1 */
  
  INIT_Done=1;                                                                  /* Allow other tasks to access the HW */

#ifdef USE_EXTERNAL_SPEED_CONTROL  
  EXTERNAL_SPEED_CONTROL_Init();
#endif
}

#if defined __TMPM_370__  || defined __TMPM_376__
const PMD_TrgProgINTTypeDef  TrgProgINT_3ShuntA =
{
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
};
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */

const PMD_TrgProgINTTypeDef  TrgProgINT_3ShuntB =
{
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
}; 

#if defined __TMPM_370__  || defined __TMPM_376__
const PMD_TrgProgINTTypeDef  TrgProgINT_1ShuntA =
{
  PMD_INTNONE,
  PMD_INTADPDA,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};  
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */

const PMD_TrgProgINTTypeDef  TrgProgINT_1ShuntB =
{
  PMD_INTNONE,
  PMD_INTADPDB,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};

#if defined __TMPM_370__  || defined __TMPM_376__
const PMD_TrgProgINTTypeDef TrgProgINT_2SensorA =
{
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */

const PMD_TrgProgINTTypeDef TrgProgINT_2SensorB =
{
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};

const PMD_TrgTypeDef PMDTrigger0_3PhaseA =
{      
  PMD_PROG0,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger1_3PhaseA =
{      
  PMD_PROG1,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger2_3PhaseA =
{      
  PMD_PROG2,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger3_3PhaseA =
{      
  PMD_PROG3,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger4_3PhaseA =
{      
  PMD_PROG4,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger5_3PhaseA =
{      
  PMD_PROG5,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger0_1PhaseA =
{      
  PMD_PROG0,
  VE_PHASE_NONE,
  VE_PHASE_ENABLE,
  VE_PHASE_NONE,
  VE_PHASE_NONE,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT
};

const PMD_TrgTypeDef PMDTrigger1_1PhaseA =
{      
  PMD_PROG1,
  VE_PHASE_ENABLE,
  VE_PHASE_NONE,
  VE_PHASE_NONE,
  VE_PHASE_ENABLE,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_VDC1
};

#ifndef BOARD_HITEX_M370
const PMD_TrgTypeDef PMDTrigger0_2PhaseA =
{      
  PMD_PROG0,
  VE_PHASE_W,
  VE_PHASE_V,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_2PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger1_2PhaseA =
{      
  PMD_PROG1,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_V,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_VDC1
};
#endif /* BOARD_HITEX_M370 */

const PMD_TrgTypeDef PMDTrigger0_3PhaseB =
{      
  PMD_PROG0,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger1_3PhaseB =
{      
  PMD_PROG1,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger2_3PhaseB =
{      
  PMD_PROG2,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger3_3PhaseB =
{      
  PMD_PROG3,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger4_3PhaseB =
{      
  PMD_PROG4,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger5_3PhaseB =
{      
  PMD_PROG5,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger0_1PhaseB =
{      
  PMD_PROG0,
  VE_PHASE_NONE,
  VE_PHASE_ENABLE,
  VE_PHASE_NONE,
  VE_PHASE_NONE,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT
};

const PMD_TrgTypeDef PMDTrigger1_1PhaseB =
{      
  PMD_PROG1,
  VE_PHASE_ENABLE,
  VE_PHASE_NONE,
  VE_PHASE_NONE,
  VE_PHASE_ENABLE,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_VDC1
};

#ifndef BOARD_HITEX_M370
const PMD_TrgTypeDef PMDTrigger0_2PhaseB =
{      
  PMD_PROG0,
  VE_PHASE_W,
  VE_PHASE_V,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_2PHASE_V,
  AIN_VDC1
};

const PMD_TrgTypeDef PMDTrigger1_2PhaseB =
{      
  PMD_PROG1,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_V,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_VDC1
};
#endif /* BOARD_HITEX_M370 */
