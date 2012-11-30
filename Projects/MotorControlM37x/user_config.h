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

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/* FW configuration */
#define USE_INTERNAL_MOTOR_PARAMS                                               /*!< Use compiled in motor parameters */
#define USE_LED                                                                 /*!< Activate LED signaling */
#define USE_SERIAL_COMMUNICATION                                                /*!< Activate serial communication protocol */
#define USE_CONFIG_STORAGE                                                      /*!< Read and Store Motor Parameter ... */
#define USE_CONFIG_STORAGE_EEPROM                                               /*!< ... in EEPROM */
//#define USE_CONFIG_STORAGE_FLASH                                                /*!< ... in flash memory */

#define USE_DSO                                                                 /*!< Activate DSO functionality */
#define USE_HSDSO                                                               /*!< Activate Highspeed-DSO functionality */
#define USE_LOAD_STATISTICS                                                     /*!< Activate internal cpu load statistics */
#define USE_ENCODER                                                             /*!< Use encoder for RPM determination */
#define USE_STALL_DETECT                                                        /*!< Activate automatic stall detection */
//#define USE_MOTOR_DISCONNECT_DETECTION                                          /*!< Activate automatic motor disconnected */
//#define USE_CAN                                                                 /*!< Enable CAN communication */
//#define USE_TURN_CONTROL                                                        /*!< Activate Turn control for position reaching */
//#define USE_USER_CALLBACKS                                                      /*!< Make use of VE Clallback Hooks */

/*************/
/* Channel 0 */
/*************/

/* Motor defines */
//#define MOTOR_CHANNEL_0 "motor_define_Gefeg_ECs-7140_HV.h"                      /*!< Motor to be used */
//#define MOTOR_CHANNEL_0 "motor_define_Shinano_PMBA-200FK_HV.h"                  /*!< Motor to be used */
#define MOTOR_CHANNEL_0 "motor_define_Nanotec_DB42S03@HitexChannel0.h"          /*!< Motor to be used */

/* Default Power Board override */
//#define BOARD_PWR_HEADER_FILE_0       "xxxxxxxxxxxxxxxxx.h"                     /*! Powerboard to be used */

/*************/
/* Channel 1 */
/*************/

/* Motor defines */
//#define MOTOR_CHANNEL_1 "motor_define_Nanotec_DB42S03.h"                        /*!< Motor to be used */
//#define MOTOR_CHANNEL_1 "motor_define_Scorpion_3014-830KV.h"                    /*!< Motor to be used */
//#define MOTOR_CHANNEL_1 "motor_define_Nanotec_DB42S01@24V.h"                    /*!< Motor to be used */
#define MOTOR_CHANNEL_1 "motor_define_Shinano_DR38312.h"                        /*!< Motor to be used */
//#define MOTOR_CHANNEL_1 "motor_define_Maxxon_EC.h"                              /*!< Motor to be used */

/* Default Power Board override */
//#define BOARD_PWR_HEADER_FILE_1       "m374pwr_lv.h"                            /*! Powerboard to be used */
//#define BOARD_PWR_HEADER_FILE_1       "m374pwr_hv.h"                            /*! Powerboard to be used */
//#define BOARD_PWR_HEADER_FILE_1       "m37x_ipm_power_ver1.h"                   /*! Powerboard to be used */
//#define BOARD_PWR_HEADER_FILE_1       "m37x_ipm_power_ver2.h"                   /*! Powerboard to be used */

/* Which channel to store in EEProm */
#define MOTOR_CHANNEL_FOR_STORAGE                1                              /*!< Motor channel to be stored if not enough memory for all */

#endif /* _USER_CONFIG_H_ */
