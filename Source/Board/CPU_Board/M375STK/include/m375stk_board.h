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

#ifndef _BOARD_M375STK_H_
#define _BOARD_M375STK_H_

/* Clock setup */
#define BOARD_USE_EXTERNAL_OSCILLATOR           1
#define BOARD_EXTERNAL_OSCILLATOR_FREQUENCY     10000000                        /* Oscillator Frequency in MHz */
#define BOARD_USE_PLL                           1                               /* Usage of PLL - 0:OFF 1:ON */
#define BOARD_CLOCK_GEAR_DIVIDER                CG_DIVIDE_1                     /* Clock gear divider: 1/2/4/8/16 */
#define BOARD_PERIPHERIAL_CLOCK_SOURCE          CG_PHIT0_SRC_FGEAR              /* Clock peripherial source */
#define BOARD_PERIPHERIAL_CLOCK_DIVIDER         CG_DIVIDE_1                     /* Clock peripherial divider: 1/2/4/8/16 */

/* Gain settings */
#define BOARD_GAIN_CURRENT_MEASURE              GAIN_1

/* Analog input port order for current measurement */
#define AIN_3PHASE_U                            ADC_AIN10
#define AIN_3PHASE_V                            ADC_AIN11
#define AIN_3PHASE_W                            ADC_AIN12 

#define AIN_1PHASE_CURRENT                      ADC_AIN16

#define AIN_2PHASE_V                            ADC_AIN10
#define AIN_2PHASE_W                            ADC_AIN11

#define AIN_VDC1                                ADC_AIN9 

/* Information for SW Over/Undervoltage Detection */
#define VDC_MEASURE1_REG                        ADC_REG0
#define VDC_MEASURE_TIMES                       5

/* Information for Temperature Control */
#define TEMPERATURE_ADC1                        TSB_ADB
#define TEMPERATURE_REG1                        ADC_REG3

/* Specification of serial port for serial protocol task */
#define SERIAL_COMMUNICATION_CHANNEL            UART0                           /* SIO channel used for UART */
#define SERIAL_COMMUNICATION_FUNCTION_REGISTER  GPIO_FUNC_REG_1                 /* Function register for UART function */
#define SERIAL_COMMUNICATION_PORT               GPIO_PE                         /* Port of SIO */
#define SERIAL_COMMUNICATION_TX                 GPIO_BIT_0                      /* Bit for TX */
#define SERIAL_COMMUNICATION_RX                 GPIO_BIT_1                      /* Bit for RX */
#define SERIAL_COMMUNICATION_TX_IRQ             INTTX0_IRQn                     /* Name of TX IRQ */
#define SERIAL_COMMUNICATION_RX_IRQ             INTRX0_IRQn                     /* Name of RX IRQ */
#define SERIAL_COMMUNICATION_TX_HANDLER         INTTX0_IRQHandler               /* Name of TX IRQ Handler */
#define SERIAL_COMMUNICATION_RX_HANDLER         INTRX0_IRQHandler               /* Name of RX IRQ Handler */

/* Specification for highspeed DSO */
#define HSDSO_CHANNEL                           UART1                           /* SIO channel used for UART */
#define HSDSO_PORT                              GPIO_PB                         /* Port of SIO */
#define HSDSO_TX                                GPIO_BIT_4                      /* Bit for TX */
#define HSDSO_FUNC_REG                          GPIO_FUNC_REG_5                 /* Function Register for switching to UART mode */
#define HSDSO_TX_IRQ_HANDLER                    INTTX1_IRQHandler               /* Name of TX IRQ handler */
#define HSDSO_TX_IRQ                            INTTX1_IRQn                     /* Name of TX IRQ number */

uint8_t BOARD_Detect_Revision           (void);

#endif /* _BOARD_M37SIGMA_H_ */
