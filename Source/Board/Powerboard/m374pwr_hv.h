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

#define USE_TEMPERATURE_CONTROL                                                 /*!< Activate Temperature Control and measurement */
#define TEMP_SLOPE 5

#define USE_EMERGENCY_SIGNAL
#define USE_OVERVOLTAGE_SIGNAL
//#define USE_CURRENT_SENSORS                                                   /* Make use of Current Sensors instead of Shunts */

/* BOARD PARAMETER */
#define BOARD_DEAD_TIME                         1600                            /* [us]             - Dead time for IGBTs */

#ifdef USE_CURRENT_SENSORS
#define BOARD_SENSITIVITY_CURRENT_MEASURE       xxx                             /* [mV/A]           - Sensivity of current measurement circuit */
#define BOARD_MEASUREMENT_TYPE                  CURRENT_SENSOR_2                /* NONE */
#define BOARD_SENSOR_DIRECTION                  0
#else
#define BOARD_SENSITIVITY_CURRENT_MEASURE       xxx                             /* [mV/V]           - Sensivity of voltage measurement circuit  */
#define BOARD_MEASUREMENT_TYPE                  CURRENT_SHUNT_1                 /* NONE */
#endif /* USE_CURRENT_SENSORS */

#define BOARD_SENSITIVITY_VOLTAGE_MEASURE       xxx                             /* [mV/V]           - Sensivity of voltage measurement circuit  */

#define BOARD_POLL                              1                               /* NONE             - Low  Side FETs high active */
#define BOARD_POLH                              1                               /* NONE             - High Side FETs high active */

#ifdef USE_TEMPERATURE_CONTROL
/* Temperature Table for Vishay NTCLE100E3103JB0 */

typedef struct {
  uint16_t  adc;
  int8_t    temperature;
} temp_table;

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
#endif /*USE_TEMPERATURE_CONTROL */
