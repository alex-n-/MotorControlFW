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

#define BOARD_GAIN_CURRENT_MEASURE              GAIN_3_0

/* Analog input port order for current measurement */
#define AIN_3PHASE_U                            ADC_AIN14
#define AIN_3PHASE_V                            ADC_AIN13
#define AIN_3PHASE_W                            ADC_AIN12 
#define AIN_3PHASE_VDC                          ADC_AIN8

#define AIN_1PHASE_CURRENT                      ADC_AIN16
#define AIN_1PHASE_VDC                          ADC_AIN4

/* SPI Channel used on the Board */
#define BOARD_SPI_CHANNEL                       SPI0                            /* SIO channel used on board for SPI */

/* Specifiction of used EEProm */
#define EEPROM_BYTE_SIZE                        128                             /* Size of EEProm in Bytes */
#define EEPROM_CS_PORT                          GPIO_PA                         /* Port for EEProm CS Signal */
#define EEPROM_CS_BIT                           GPIO_BIT_7                      /* Bit for EEProm CS Signal */

//#define USE_BLUETOOTH /* define this if you have a bluetooth dongle attached to the uart */

/* Specification of serial port for serial protocol task */
#ifdef USE_BLUETOOTH
#define SERIAL_COMMUNICATION_CHANNEL            UART3                           /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_2                 /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PF                         /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_3                      /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_4                      /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX3_IRQn                     /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX3_IRQn                     /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX3_IRQHandler               /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX3_IRQHandler               /* Name of RX IRQ Handler */
#else
#define SERIAL_COMMUNICATION_CHANNEL            UART2                           /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_1                 /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PD                         /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_5                      /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_6                      /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX2_IRQn                     /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX2_IRQn                     /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX2_IRQHandler               /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX2_IRQHandler               /* Name of RX IRQ Handler */
#endif

/* Specification for highspeed DSO */
#define HSDSO_CHANNEL                           UART3                           /* SIO channel used for UART */
#define HSDSO_PORT                              GPIO_PF                         /* Port of SIO */
#define HSDSO_TX                                GPIO_BIT_3                      /* Bit for TX */
#define HSDSO_TX_IRQ_HANDLER                    INTTX3_IRQHandler               /* Name of TX IRQ handler */
#define HSDSO_TX_IRQ                            INTTX3_IRQn                     /* Name of TX IRQ number */

/* Signaling of different states */
#define LED_SIGNAL_CONFIG_READ                  LED_NO_0                        /* Led Number for signaling config succesful read from EEProm */
#define LED_SIGNAL_SERIAL_COMMUNICATION_RUNNING LED_NO_1                        /* Led Number for signaling serial communication protocol active */
#define LED_SIGNAL_VE_RUN_BASE                  LED_NO_2                        /* Led base Number for VE is in FOC */

void    BOARD_SetupHW                   (void);
uint8_t BOARD_Detect_Revision           (void);
void    BOARD_ConfigureADCforTemperature(uint8_t channel_number);
int8_t  BOARD_GetTemperature            (uint8_t channel_number);

#endif /* _BOARD_HITEX_M370_H_ */
