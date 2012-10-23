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
#include BOARD_SPI_HEADER_FILE
#include BOARD_LED_HEADER_FILE
#include BOARD_GAIN_HEADER_FILE
#ifndef BOARD_HITEX_M370
#include BOARD_PGA_HEADER_FILE
#endif

#ifdef BOARD_HITEX_M370
#include "hitex_m370_lcd.h"
#include "hitex_m370_keypad.h"
#endif /* BOARD_HITEX_M370 */

#include "board.h"
#include "eeprom.h"
#include "spi.h"
#include "pmd.h"
#include "motorctrl.h"
#include "i2c_master_bitbanging.h"
#include "external_speed_control.h"

uint8_t   BoardRevision = 0;
uint8_t   INIT_Done     = 0;
uint32_t  T0            = 0;

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
  
  SPI_DeviceInit(SPI_10_MHZ);                                         /* Init SPI Channel */

  BoardRevision = BOARD_Detect_Revision();
  
#ifdef MOTOR_CHANNEL_0                                                /* Set up Board dependant values */
  ChannelValues[0].a_max            = BOARD_A_MAX_CHANNEL_0;
  ChannelValues[0].gain_current     = BOARD_GAIN_CURRENT_0;
  ChannelValues[0].v_max            = BOARD_V_MAX_CHANNEL_0;
  ChannelValues[0].gain_vdc         = BOARD_GAIN_IDCBUS_0;
  ChannelValues[0].shunt_type       = BOARD_SHUNT_TYPE_CHANNEL_0;
  ChannelValues[0].sensor_direction = BOARD_SENSOR_DIRECTION_0;
  ChannelValues[0].poll             = BOARD_POLL_0;
  ChannelValues[0].polh             = BOARD_POLH_0;
#endif /* MOTOR_CHANNEL_0 */ 
#ifdef MOTOR_CHANNEL_1
  ChannelValues[1].a_max            = BOARD_A_MAX_CHANNEL_1;
  ChannelValues[1].gain_current     = BOARD_GAIN_CURRENT_1;
  ChannelValues[1].v_max            = BOARD_V_MAX_CHANNEL_1;
  ChannelValues[1].gain_vdc         = BOARD_GAIN_IDCBUS_1;
  ChannelValues[1].shunt_type       = BOARD_SHUNT_TYPE_CHANNEL_1;
  ChannelValues[1].sensor_direction = BOARD_SENSOR_DIRECTION_1;
  ChannelValues[1].poll             = BOARD_POLL_1;
  ChannelValues[1].polh             = BOARD_POLH_1;
#endif /* MOTOR_CHANNEL_1 */
  
#ifdef USE_LED
  LED_Init();                                                         /* Init LED Ports */
#endif /* USE_LED */
  
#ifdef USE_KEYPAD  
  KEYPAD_Init();                                                      /* Init Keypad Port */
#endif /* USE_KEYPAD */
  
#ifdef USE_LCD  
  LCD_Init();                                                         /* Init LCD */
#endif /* USE_LCD */ 
  
  BOARD_SetupGain();                                                  /* Set up the gain for current measure */
  
#ifdef USE_HV_COMMUNICATION
  HV_SerialCommunicationInit();
#endif /* USE_HV_COMMUNICATION */  

#ifdef USE_I2C_MASTER
  I2C_Init();
#endif  
}

void BOARD_SetupHW2(void)
{
#ifdef __TMPM_370__  
  ADC_Init(0,(CURRENT_MEASUREMENT)ChannelValues[0].shunt_type);       /* enable, configure the ADC */
  PMD_Init(0);
#endif /* __TMPM_370__ */

  ADC_Init(1,(CURRENT_MEASUREMENT)ChannelValues[1].shunt_type);       /* enable, configure the ADC */
  PMD_Init(1);
  
  INIT_Done=1;                                                        /* Allow other tasks to access the HW */

#ifdef USE_EXTERNAL_SPEED_CONTROL  
  EXTERNAL_SPEED_CONTROL_Init();
#endif
  
}

#ifdef __TMPM_370__
const PMD_TrgProgINTTypeDef  TrgProgINT_3ShuntA =
{
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTADPDA,
};
#endif /* __TMPM_370__ */

const PMD_TrgProgINTTypeDef  TrgProgINT_3ShuntB =
{
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTADPDB,
}; 

#ifdef __TMPM_370__
const PMD_TrgProgINTTypeDef  TrgProgINT_1ShuntA =
{
  PMD_INTNONE,
  PMD_INTADPDA,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};  
#endif /* __TMPM_370__ */

const PMD_TrgProgINTTypeDef  TrgProgINT_1ShuntB =
{
  PMD_INTNONE,
  PMD_INTADPDB,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};

#ifdef __TMPM_370__
const PMD_TrgProgINTTypeDef TrgProgINT_2SensorA =
{
  PMD_INTADPDA,
  PMD_INTADPDA,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};
#endif /* __TMPM_370__ */

const PMD_TrgProgINTTypeDef TrgProgINT_2SensorB =
{
  PMD_INTADPDB,
  PMD_INTADPDB,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
  PMD_INTNONE,
};

const PMD_TrgTypeDef PMDTrigger0_3Phase =
{      
  PMD_PROG0,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger1_3Phase =
{      
  PMD_PROG1,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger2_3Phase =
{      
  PMD_PROG2,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_ENABLE,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger3_3Phase =
{      
  PMD_PROG3,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger4_3Phase =
{      
  PMD_PROG4,
  VE_PHASE_U,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_ENABLE,
  AIN_3PHASE_U,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger5_3Phase =
{      
  PMD_PROG5,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_3PHASE_V,
  AIN_3PHASE_W,
  AIN_3PHASE_U,
  AIN_3PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger0_1Phase =
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

const PMD_TrgTypeDef PMDTrigger1_1Phase =
{      
  PMD_PROG1,
  VE_PHASE_ENABLE,
  VE_PHASE_NONE,
  VE_PHASE_NONE,
  VE_PHASE_ENABLE,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_CURRENT,
  AIN_1PHASE_VDC
};

#ifndef BOARD_HITEX_M370
const PMD_TrgTypeDef PMDTrigger0_2Phase =
{      
  PMD_PROG0,
  VE_PHASE_W,
  VE_PHASE_V,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_2PHASE_V,
  AIN_2PHASE_VDC
};

const PMD_TrgTypeDef PMDTrigger1_2Phase =
{      
  PMD_PROG1,
  VE_PHASE_V,
  VE_PHASE_W,
  VE_PHASE_U,
  VE_PHASE_ENABLE,
  AIN_2PHASE_V,
  AIN_2PHASE_W,
  AIN_2PHASE_V,
  AIN_2PHASE_VDC
};
#endif /* BOARD_HITEX_M370 */