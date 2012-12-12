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
 * OTHERWISE. (C)Copyright TOSHIBA CORPORATION 2012 All rights reserved
 */
#include <stdint.h> 

/*!< Activate Temperature Control and measurement */
//#define USE_TEMPERATURE_CONTROL

/*!< Select the temerature slop - hysteresis of temperature control -
     drop temperature in degree celsius before motor start is possible again 
     after overtemperature has been detected */
//#define TEMP_SLOPE x

/*!< Temperature table for the thermistor to be used for the temperatre calculation /
     overtemperature control as defined in temperature_measure_table.h */
//#define VISHAY_NTCLE100E3103JB0

/*!< use the EMERGENCY SIGNAL pin from the powerstage */
//#define USE_EMERGENCY_SIGNAL

/*!< use the OVERVOLTAGE SIGNAL pin from the powerstage */
//#define USE_OVERVOLTAGE_SIGNAL

/*!< use Atmel HV serial communication protocol for the DC Link Voltage measurement with
     galvanic isolation */
//#define USE_HV_COMMUNICATION

/*!< invert high/low signal of serial signal */
//#define USE_HV_COMMUNICATION_INVERT

/*!< Clamp the DC Link voltage to a given value - either because there is no 
     voltage measurement possibility of due to galvanic isolation without
     Atmel HV serial communication protocol */
//#define BOARD_VDC_CHANNEL_0                     xx
//#define BOARD_VDC_CHANNEL_1                     xx

/*******************/
/* BOARD PARAMETER */
/*******************/

/*!< Dead time of the MOSFets/IGBTs used on the powerstage */
//#define BOARD_DEAD_TIME                         xxxx                            /* [us]             - Dead time for IPM */

/*!< Sensitivity of the current measurement (current sensors / OpAmp stage) */
//#define BOARD_SENSITIVITY_CURRENT_MEASURE       xxx                             /* [mV/A]           - Sensivity of current measurement circuit */

/*!< Type of current measurement */
//#define BOARD_MEASUREMENT_TYPE                  CURRENT_SHUNT_1 | CURRENT_SENSOR_2 | CURRENT_SHUNT_3 

/*!< Direction of the current sensors 0 means IF- of side of motor
                                      1 means IF+ of side of motor
     (only needed to be defined if CURRENT_SENSOR_2 for BOARD_MEASUREMENT_TYPE*/
//#define BOARD_SENSOR_DIRECTION                  0 | 1

/*!< Sensitivity of the voltage measurement (resistor array) */
//#define BOARD_SENSITIVITY_VOLTAGE_MEASURE       xx                              /* [mV/V]           - Sensivity of voltage measurement circuit  */

/*!< Polarisation of FET Signal for high level of low side */ 
//#define BOARD_POLL                              x                               /* NONE             - Low  Side FETs high active */
   
/*!< Polarisation of FET Signal for high level of high side */ 
//#define BOARD_POLH                              x                               /* NONE             - High Side FETs high active */

