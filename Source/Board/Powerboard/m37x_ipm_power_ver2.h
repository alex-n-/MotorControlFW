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

#define HV_JUPERED

#define USE_TEMPERATURE_CONTROL                                       /*!< Activate Temperature Control and measurement */
#define TEMP_SLOPE 5

#define USE_EMERGENCY_SIGNAL
#define USE_OVERVOLTAGE_SIGNAL
#define USE_HV_COMMUNICATION

/* BOARD PARAMETER */
#define BOARD_DEAD_TIME                         2200                            /* [us]             - Dead time for IPM */
#define BOARD_SENSITIVITY_CURRENT_MEASURE       104                             /* [mV/A]           - Sensivity of current measurement circuit */
#define BOARD_MEASUREMENT_TYPE                  CURRENT_SENSOR_2                /* NONE */

#define BOARD_SENSOR_DIRECTION                  0
#define BOARD_POLL                              1                               /* NONE             - Low  Side FETs high active */
#define BOARD_POLH                              1                               /* NONE             - High Side FETs high active */

#ifdef HV_JUPERED
#define BOARD_SENSITIVITY_VOLTAGE_MEASURE       12                              /* [mV/V]           - Sensivity of voltage measurement circuit  */
#else
#define BOARD_SENSITIVITY_VOLTAGE_MEASURE       84                              /* [mV/V]           - Sensivity of voltage measurement circuit  */
#endif 

#ifdef USE_TEMPERATURE_CONTROL
/* Temperature Table for Mitsubishi PS219B4-AS */

typedef struct {
  uint16_t  adc;
  int8_t    temperature;
} temp_table;

static const temp_table temperature[]={
                                    {0x00F,-15},
                                    {0x029,-10},
                                    {0x043, -5},
                                    {0x05E,  0},
                                    {0x078,  5},
                                    {0x092, 10},
                                    {0x0AC, 15},
                                    {0x0C7, 20},
                                    {0x0E1, 25},
                                    {0x0FB, 30},
                                    {0x115, 35},
                                    {0x130, 40},
                                    {0x14A, 45},
                                    {0x164, 50},
                                    {0x17E, 55},
                                    {0x199, 60},
                                    {0x1B3, 65},
                                    {0x1CD, 70},
                                    {0x1E7, 75},
                                    {0x202, 80},
                                    {0x21C, 85},
                                    {0x236, 90},
                                    {0x250, 95},
                                    {0x26B,100},
                                    {0x285,105},
                                    {0x29F,110},
                                    {0x2BA,115},
                                    {0x2D4,120},
                                  };
#endif /*USE_TEMPERATURE_CONTROL */
