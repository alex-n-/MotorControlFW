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

/******************************************************************************/
/*                                                                            */
/*                           This is a Glyn Powerboad                         */
/* - please get in contact with Glyn (Power@glyn.de) for further information  */ 
/*                                                                            */
/******************************************************************************/

#include <stdint.h> 

#define USE_TEMPERATURE_CONTROL                                                 /*!< Activate Temperature Control and measurement */
#define TEMP_SLOPE 5
#define MITSUBISHI_PS219B4_AS

#define USE_EMERGENCY_SIGNAL
#define USE_OVERVOLTAGE_SIGNAL

#define USE_HV_COMMUNICATION
#define USE_HV_COMMUNICATION_INVERT

/* BOARD PARAMETER */
#define BOARD_NAME_PWR                          "M37x IPM-PWR v1.0"

#define BOARD_DEAD_TIME                         2200                            /* [ns]             - Dead time for IPM */
#define BOARD_BOOTSTRAP_DELAY                     10                            /* [ms]             - Bootstrap time */

#define BOARD_SENSITIVITY_CURRENT_MEASURE       104                             /* [mV/A]           - Sensivity of current measurement circuit */
#define BOARD_MEASUREMENT_TYPE                  CURRENT_SENSOR_2                /* NONE */

#define BOARD_SENSOR_DIRECTION                  0
#define BOARD_SENSITIVITY_VOLTAGE_MEASURE       12                              /* [mV/V]           - Sensivity of voltage measurement circuit  */
#define BOARD_POLL                              1                               /* NONE             - Low  Side FETs high active */
#define BOARD_POLH                              1                               /* NONE             - High Side FETs high active */
