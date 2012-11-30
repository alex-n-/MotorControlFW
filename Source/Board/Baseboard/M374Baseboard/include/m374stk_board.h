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

#ifndef _BOARD_M374STK_H_
#define _BOARD_M374STK_H_

#include <stdint.h>

/* Gain settings for the different channels */
#define BOARD_GAIN_CURRENT_MEASURE              GAIN_1

/* Analog input port order for current measurement */
#define AIN_3PHASE_U                            ADC_AIN9
#define AIN_3PHASE_V                            ADC_AIN10
#define AIN_3PHASE_W                            ADC_AIN11 
#define AIN_3PHASE_VDC                          ADC_AIN12

#define AIN_1PHASE_CURRENT                      ADC_AIN2
#define AIN_1PHASE_VDC                          ADC_AIN12

#define AIN_2PHASE_V                            ADC_AIN10
#define AIN_2PHASE_W                            ADC_AIN11
#define AIN_2PHASE_VDC                          ADC_AIN12 

/* Specification of serial port for serial protocol task */
#define SERIAL_COMMUNICATION_CHANNEL            UART0                 /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_1       /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PE               /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_0            /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_1            /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX0_IRQn           /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX0_IRQn           /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX0_IRQHandler     /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX0_IRQHandler     /* Name of RX IRQ Handler */

/* SPI Channel used on the Board */
#define BOARD_SPI_CHANNEL                       SPI1                  /* SIO channel used on board for SPI */

/* Specifiction of used EEProm */
#define EEPROM_BYTE_SIZE                        512                   /* Size of EEProm in Bytes */
#define EEPROM_CS_PORT                          GPIO_PF               /* Port for EEProm CS Signal */
#define EEPROM_CS_BIT                           GPIO_BIT_0            /* Bit for EEProm CS Signal */

/* Specification for highspeed DSO */
#define HSDSO_CHANNEL                           UART3                         /* SIO channel used for UART */
#define HSDSO_PORT                              GPIO_PF                       /* Port of SIO */
#define HSDSO_TX                                GPIO_BIT_3                    /* Bit for TX */
#define HSDSO_TX_IRQ_HANDLER                    INTTX3_IRQHandler             /* Name of TX IRQ handler */
#define HSDSO_TX_IRQ                            INTTX3_IRQn                   /* Name of TX IRQ number */

/* Specification for external Speed Control */
#define ESC_CHANNEL                     1

#define SPEED_CONTROL_ADC_PORT          GPIO_PJ
#define SPEED_CONTROL_ADC_PIN           GPIO_BIT_1
#define SPEED_CONTROL_ADC_CHANNEL       TSB_ADB
#define SPEED_CONTROL_ADC_REG           ADC_REG4
#define SPEED_CONTROL_ADC_HANDLER       INTADBTMR_IRQHandler
#define SPEED_CONTROL_ADC_IRQ           INTADBTMR_IRQn

#define SPEED_CONTROL_PWM_PORT          GPIO_PA
#define SPEED_CONTROL_PWM_PIN           GPIO_BIT_7
#define SPEED_CONTROL_PWM_TMRB          TSB_TB4

#define SPEED_CONTROL_PWM_HANDLER       INTCAP41_IRQHandler
#define SPEED_CONTROL_PWM_IRQ           INTCAP41_IRQn
#define SPEED_CONTROL_PWM_HANDLER2      INTTB41_IRQHandler
#define SPEED_CONTROL_PWM_IRQ2          INTTB41_IRQn

#define SPEED_CONTROL_CWCCW_PORT        GPIO_PA
#define SPEED_CONTROL_CWCCW_PIN         GPIO_BIT_2

#define SPEED_CONTROL_FAULT_PORT        GPIO_PA
#define SPEED_CONTROL_FAULT_PIN         GPIO_BIT_1

#define SPEED_CONTROL_FG_PORT           GPIO_PA
#define SPEED_CONTROL_FG_PIN            GPIO_BIT_3

/* Signaling of different states */
#define LED_SIGNAL_CONFIG_READ                  LED_NO_2              /* Led Number for signaling config succesful read from EEProm */
#define LED_SIGNAL_SERIAL_COMMUNICATION_RUNNING LED_NO_3              /* Led Number for signaling serial communication protocol active */
#define LED_SIGNAL_CAN_COMMUNICATION_RUNNING    LED_NO_4              /* Led Number for signaling can communication protocol active */
#define LED_SIGNAL_VE_RUN_BASE                  LED_NO_4              /* Led base Number for VE is in FOC */

void    BOARD_SetupHW                   (void);
uint8_t BOARD_Detect_Revision           (void);
void    BOARD_ConfigureADCforTemperature(uint8_t channel_number);
int8_t  BOARD_GetTemperature            (uint8_t channel_number);

#endif /* _BOARD_M374STK_H_ */




