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

#ifndef _BOARD_HITEX_M370_H_
#define _BOARD_HITEX_M370_H_

#define USE_TEMPERATURE_CONTROL                                       /*!< Activate Temperature Control and measurement */
#define USE_EMERGENCY_SIGNAL
#define USE_OVERVOLTAGE_SIGNAL
//#define HWPATCH_2_TIMES_LV
//#define USE_RW_BOARD_SETTINGS                                         /*!< board settings may be changed by software */

#include "tmpm370_adc.h"

#define BOARD_NAME                              "Hitex M370"                  /* Name of the Board */

/* Analog input port order for current measurement */
#define AIN_3PHASE_U                            ADC_AIN14
#define AIN_3PHASE_V                            ADC_AIN13
#define AIN_3PHASE_W                            ADC_AIN12 
#define AIN_3PHASE_VDC                          ADC_AIN8

#define AIN_1PHASE_CURRENT                      ADC_AIN16
#define AIN_1PHASE_VDC                          ADC_AIN4

/* BOARD PARAMETER */
#ifdef HWPATCH_2_TIMES_LV
#define BOARD_A_MAX_CHANNEL_0                   3000                          /* [mA]             - Input current(Iabc) at ADC =0xFFF0 */
#define BOARD_V_MAX_CHANNEL_0                   36                            /* [V]              - Input voltage(Vdc)   at ADC =0xFFF0 */
#define BOARD_SHUNT_TYPE_CHANNEL_0              CURRENT_SHUNT_3               /* NONE             - Enum Type (SHUNT_TYPE_1 = 1,SHUNT_TYPE_3 = 3) */
#define BOARD_VDC_CHANNEL_0                     24                            /* [V]              - VDC value for Channel 0 */
#define BOARD_GAIN_CURRENT_0			0
#define BOARD_GAIN_IDCBUS_0			0
#define BOARD_SENSOR_DIRECTION_0                0
#else
#define BOARD_A_MAX_CHANNEL_0                   3000                          /* [mA]             - Input current(Iabc) at ADC =0xFFF0 */
#define BOARD_V_MAX_CHANNEL_0                   400                           /* [V]              - Input voltage(Vdc)   at ADC =0xFFF0 */
#define BOARD_SHUNT_TYPE_CHANNEL_0              CURRENT_SHUNT_3               /* NONE             - Enum Type (SHUNT_TYPE_1 = 1,SHUNT_TYPE_3 = 3) */
#define BOARD_VDC_CHANNEL_0                     300                           /* [V]              - VDC value for Channel 0 */
#define BOARD_GAIN_CURRENT_0			0
#define BOARD_GAIN_IDCBUS_0			0
#define BOARD_SENSOR_DIRECTION_0                0
#endif /* HWPATCH_2_TIMES_LV */
#define BOARD_POLL_0                            1                     /* NONE             - Low  Side FETs high active */
#define BOARD_POLH_0                            1                     /* NONE             - High Side FETs high active */

#define BOARD_A_MAX_CHANNEL_1                   2000                          /* [mA]             - Input current(Iabc) at ADC =0xFFF0 */
#define BOARD_V_MAX_CHANNEL_1                   36                            /* [V]              - Input voltage(Vdc)   at ADC =0xFFF0 */
#define BOARD_SHUNT_TYPE_CHANNEL_1              CURRENT_SHUNT_1               /* NONE             - Enum Type (SHUNT_TYPE_1 = 1,SHUNT_TYPE_3 = 3) */
#define BOARD_SENSOR_DIRECTION_1                0
#define BOARD_GAIN_CURRENT_1			0
#define BOARD_GAIN_IDCBUS_1			0
#define BOARD_SENSOR_DIRECTION_1                0

#define BOARD_POLL_1                            1                     /* NONE             - Low  Side FETs high active */
#define BOARD_POLH_1                            1                     /* NONE             - High Side FETs high active */
#define BOARD_AVAILABLE_CHANNELS                VE_CHANNEL_0|VE_CHANNEL_1     /* Hitex Board has VE channel 0 & 1 available */

/* SPI Channel used on the Board */
#define BOARD_SPI_CHANNEL                       SPI0                          /* SIO channel used on board for SPI */

/* Specifiction of used EEProm */
#define EEPROM_BYTE_SIZE                        128                           /* Size of EEProm in Bytes */
#define EEPROM_CS_PORT                          GPIO_PA                       /* Port for EEProm CS Signal */
#define EEPROM_CS_BIT                           GPIO_BIT_7                    /* Bit for EEProm CS Signal */

/* Specifiction for DSO storage size */
#ifdef USE_DSO
#define BOARD_DSO_SIZE                          1536                          /* Number of uint16_t */
#else
#define BOARD_DSO_SIZE                          0                             /* Number of uint16_t */
#endif

//#define USE_BLUETOOTH /* define this if you have a bluetooth dongle attached to the uart */

/* Specification of serial port for serial protocol task */
#ifdef USE_BLUETOOTH
#define SERIAL_COMMUNICATION_CHANNEL            UART3                         /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_2               /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PF                       /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_3                    /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_4                    /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX3_IRQn                   /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX3_IRQn                   /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX3_IRQHandler             /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX3_IRQHandler             /* Name of RX IRQ Handler */
#else
#define SERIAL_COMMUNICATION_CHANNEL            UART2                         /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_1               /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PD                       /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_5                    /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_6                    /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX2_IRQn                   /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX2_IRQn                   /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX2_IRQHandler             /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX2_IRQHandler             /* Name of RX IRQ Handler */
#endif

/* Specification for highspeed DSO */
#define HSDSO_CHANNEL                           UART3                         /* SIO channel used for UART */
#define HSDSO_PORT                              GPIO_PF                       /* Port of SIO */
#define HSDSO_TX                                GPIO_BIT_3                    /* Bit for TX */
#define HSDSO_TX_IRQ_HANDLER                    INTTX3_IRQHandler             /* Name of TX IRQ handler */
#define HSDSO_TX_IRQ                            INTTX3_IRQn                   /* Name of TX IRQ number */

/* Signaling of different states */
#define LED_SIGNAL_CONFIG_READ                  LED_NO_0                      /* Led Number for signaling config succesful read from EEProm */
#define LED_SIGNAL_SERIAL_COMMUNICATION_RUNNING LED_NO_1                      /* Led Number for signaling serial communication protocol active */
#define LED_SIGNAL_VE_RUN_BASE                  LED_NO_2                      /* Led base Number for VE is in FOC */

typedef struct {
  uint16_t  adc;
  int8_t    temperature;
} temp_table;


void    BOARD_SetupHW                   (void);
uint8_t BOARD_Detect_Revision           (void);
void    BOARD_ConfigureADCforTemperature(uint8_t channel_number);
int8_t  BOARD_GetTemperature            (uint8_t channel_number);

#endif /* _BOARD_HITEX_M370_H_ */
