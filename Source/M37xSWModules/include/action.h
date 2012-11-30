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

#ifndef _ACTION_H_
#define _ACTION_H_

#include <stdint.h>

#include "motorctrl.h"
#include "dso.h"

#define DECLARE(x, ...) \
__packed struct x { \
    __VA_ARGS__ \
} __attribute__((packed))

/*! \brief Start Motor Question
 *
 *  Load data of protocol communication for start of motor (question from host)
 */
DECLARE(StartMotor_q, 
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Start Motor Answer
 *
 *  Load data of protocol communication for start of motor (answer to host)
 */
DECLARE(StartMotor_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Stop Motor Question
 *
 *  Load data of protocol communication for stop of motor (question from host)
 */
DECLARE(StopMotor_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Stop Motor Answer
 *
 *  Load data of protocol communication for stop of motor (answer to host)
 */
DECLARE(StopMotor_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Set Motor Parameter Question
 *
 *  Load data of protocol communication for setting the motor parameters (question from host)
 */
DECLARE(SetMotorParameter_q,
  uint8_t         motor_nr;                                                     /*!< Motor number */
  MotorParameters Parameters;                                                   /*!< Motor parameters */      
);

/*! \brief Set Motor Parameter Answer
 *
 *  Load data of protocol communication for setting the motor parameters (answer to host)
 */
DECLARE(SetMotorParameter_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get Motor Parameter Question
 *
 *  Load data of protocol communication for getting the motor parameters (question from host)
 */
DECLARE(GetMotorParameter_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Get Motor Parameter Answer
 *
 *  Load data of protocol communication for getting the motor parameters (answer to host)
 */
DECLARE(GetMotorParameter_a,
  MotorParameters Parameters;                                                   /*!< Motor parameters */
);

/*! \brief Set PI-Control Parameter Question
 *
 *  Load data of protocol communication for setting the PI parameters (question from host)
 */
DECLARE(SetPIControl_q,
  uint8_t           motor_nr;                                                   /*!< Motor number */
  PIControlSettings PISettings;                                                 /*!< PI Values */
);

/*! \brief Set PI-Control Parameter Answer
 *
 *  Load data of protocol communication for setting the PI parameters (answer to host)
 */
DECLARE(SetPIControl_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get PI-Control Parameter Question
 *
 *  Load data of protocol communication for getting the PI parameters (question from host)
 */
DECLARE(GetPIControl_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Get PI-Control Parameter Answer
 *
 *  Load data of protocol communication for getting the PI parameters (answer to host)
 */
DECLARE(GetPIControl_a,
  PIControlSettings PISettings;                                                 /*!< PI Values */
);

/*! \brief Get Channel Dependand Values Question
 *
 *  Load data of protocol communication for getting the Channel Dependand Values (question from host)
 */
DECLARE(GetChannelDependand_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Get Channel Dependand Values Answer
 *
 *  Load data of protocol communication for getting the Channel Dependand Values (answer to host)
 */
DECLARE(GetChannelDependand_a,
  ChannelDependandValues ChannelSettings;                                       /*!< Channel Dependand Values */
);

/*! \brief Set Channel Dependand Values Question
 *
 *  Load data of protocol communication for setting the Channel Dependand Values (question from host)
 */
DECLARE(SetChannelDependand_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
  ChannelDependandValues ChannelSettings;                                       /*!< Channel Dependand Values */
);

/*! \brief Set Channel Dependand Values Answer
 *
 *  Load data of protocol communication for setting the Channel Dependand Values (answer to host)
 */
DECLARE(SetChannelDependand_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Set System Dependand Parameter Question
 *
 *  Load data of protocol communication for setting the System Dependand Parameter (question from host)
 */
DECLARE(SetSystemDependand_q,
  uint8_t motor_nr;
  SystemDependandValues SystemSettings;
);

/*! \brief Set System Dependand Parameter Answer
 *
 *  Load data of protocol communication for setting the System Dependand Parameters (answer to host)
 */
DECLARE(SetSystemDependand_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get System Dependand Parameter Question
 *
 *  Load data of protocol communication for getting the System Dependand Parameter (question from host)
 */
DECLARE(GetSystemDependand_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Get System Dependand Parameter Answer
 *
 *  Load data of protocol communication for getting the System Dependand Parameter (answer to host)
 */
DECLARE(GetSystemDependand_a,
  SystemDependandValues SystemSettings;
);

/*! \brief Set the Motor Settings Question
 *
 *  Load data of protocol communication for setting the Motor Settings (question from host)
 */
DECLARE(SetMotorSet_q,
  uint8_t          motor_nr;                                                    /*!< Motor number */
  MotorSetSettings Set;                                                         /*!< Set values   */
);

/*! \brief Set the Motor Settings Answer
 *
 *  Load data of protocol communication for setting the Motor Settings (answer to host)
 */
DECLARE(SetMotorSet_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get actual Motor State Question
 *
 *  Load data of protocol communication for getting the actual Motor State (question from host)
 */
DECLARE(GetMotorState_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Get actual Motor State Answer
 *
 *  Load data of protocol communication for getting the actual Motor State (answer to host)
 */
DECLARE(GetMotorState_a,
  MotorStateSettings State;                                                     /*!< actual state of Motor */
);

/*! \brief Store Parameters (Motor/PI/System) to EEProm Question
 *
 *  Load data of protocol communication for storing the uploaded Paramerer Sets 
 *  to the EEProm (question from host)
 */
DECLARE(StoreParameters_q,
  uint8_t motor_nr;                                                             /*!< Motor number */
);

/*! \brief Store Parameters (Motor/PI/System) to EEProm Answer
 *
 *  Load data of protocol communication for storing the uploaded Paramerer Sets 
 *  to the EEProm (answer to host)
 */
DECLARE(StoreParameters_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Clears EEProm content Question
 *
 *  Load data of protocol communication for clearing EEProm content (question from host)
 */
DECLARE(ClearParameters_q,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Clears EEProm content Answer
 *
 *  Load data of protocol communication for clearing EEProm content (answer to host)
 */
DECLARE(ClearParameters_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Set new UART speed for communication Question
 *
 *  Load data of protocol communication for setting new UART speed (question from host)
 */
DECLARE(ConfigureUARTSpeed_q,
  uint32_t speed;                                                               /*!< new speed setting in bps */
);

/*! \brief Set new UART speed for communication Answer
 *
 *  Load data of protocol communication for setting new UART speed (answer to host)
 */
DECLARE(ConfigureUARTSpeed_a,
  uint8_t dummy;                                                                /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get a snapshot of different Vector Engine Registers Question
 *
 *  Load data of protocol communication for getting a snaphot of different
 *  internal Vector Engine registers (question from host)
 */
DECLARE(GetDSOSnaphot_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
  uint32_t selected_values;
);

/*! \brief Get a snapshot of different Vector Engine Registers Answer
 *
 *  Load data of protocol communication for getting a snaphot of different
 *  internal Vector Engine registers (answer to host)
 */
DECLARE(GetDSOSnaphot_a,
  DSOValues values;
  uint32_t  Timestamp;
);

/*! \brief Configure DSO Logging facility Question
 *
 *  Load data of protocol communication for configuring a logging of internal
 *  Vector Engine Registers (question from host)
 */
DECLARE(ConfigDSOLog_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
  uint32_t selected_values;                                                     /*!< Bitfield with DSO_Selection enumerated values for selecting the signals to be logged */
  uint32_t trigger_on_value;                                                    /*!< Bitfield with DSO_Selection enumerated values for selecting the signals to trigger on */
  uint8_t  trigger_mode;                                                        /*!< Bitfield with TriggerMode enumerated values to select the trigger mode of the DSO */
  uint16_t trigger_level;                                                       /*!< Trigger level for the DSO */
  uint8_t  spread_factor;                                                       /*!< Factor of which the logging is spread */
);

/*! \brief Configure DSO Logging facility Answer
 *
 *  Load data of protocol communication for configuring a logging of internal
 *  Vector Engine Registers (answer to host)
 */
DECLARE(ConfigDSOLog_a,
  uint16_t logsteps;                                                            /*!< Number of data-sets that will be logged */
);

/*! \brief Request the DSO Log Data Question
 *
 *  Load data of protocol communication for getting the data that has been logged
 *  from the DOS after the DSO has triggered (question from host)
 */
DECLARE(GetDSOLogData_q,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Request the DSO Log Data Answer
 *
 *  Load data of protocol communication for getting the data that has been logged
 *  from the DOS after the DSO has triggered (answer to host)
 */
DECLARE(GetDSOLogData_a,
  DSOValues values;                                                             /*!< Dataset of logged DSO data */
);

/*! \brief Request actual system load Question
 *
 *  Load data of protocol communication for getting getting the load of the
 *  running system by percent
 */
DECLARE(GetSystemLoad_q,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Request actual system load
 *
 *  Load data of protocol communication for getting getting the load of the
 *  running system by percent
 */
DECLARE(GetSystemLoad_a,
  uint8_t SystemLoadPercentage;                                                 /*!< System Load in Percent */
);

/*! \brief Request the Board Info
 *
 *  Load data of protocol communication for getting the the board info
 */

DECLARE(GetBoardInfo_q,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Request the DSO Log Data Answer
 *
 *  Load data of protocol communication for getting the the board info
 */
DECLARE(GetBoardInfo_a,
  BoardInfoValues values;                                                       /*!< Board Info Values */
);

/*! \brief Configure highspeed DSO - question
 *
 *  Configure highspeed DSO
 */
DECLARE(ConfigureHsDSO_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
  uint8_t  enable;                                                              /*!< enable the HSDSO  */
  uint32_t selected_values;                                                     /*!< Bitfield with DSO_Selection enumerated values for selecting the signals to be logged */
  uint8_t  spread_factor;                                                       /*!< Factor of which the logging is spread */
);

/*! \brief Configure highspeed DSO - answer
 *
 *  Configure highspeed DSO
 */
DECLARE(ConfigureHsDSO_a,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get Firmware Version - question
 *
 *  Get FW Version
 */
DECLARE(GetFWVersion_q,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get Firmware Version - answer
 *
 *  Get FW Version
 */
DECLARE(GetFWVersion_a,
  FWVersion  values;                                                            /*!< FW Version  */
);

/*! \brief Do Turn - question
 *
 *  Do Turn
 */
DECLARE(DoTurn_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
  int16_t  turns;                                                               /*!< Number of turns to perform */
  uint16_t max_speed;                                                           /*!< Maximum rpm while turning */
);

/*! \brief Do Turn - answer
 *
 *  Do Turn
 */
DECLARE(DoTurn_a,
  uint8_t  dummy;                                                               /*!< Dummy value - due to protocol specification  */
);

/*! \brief Get Turn Number - question
 *
 */
DECLARE(GetTurnNumber_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
);

/*! \brief Get Turn Number - answer
 *
 */
DECLARE(GetTurnNumber_a,
  int32_t turn_number;                                                          /*!< Number of full turns */
);

/*! \brief Get Temperature - question
 *
 */
DECLARE(GetTemperature_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
);

/*! \brief Get Temperature - answer
 *
 */
DECLARE(GetTemperature_a,
  int8_t temperature;                                                           /*!< Temperature in Celsius */
);

/*! \brief Get Error State - question
 *
 */
DECLARE(GetErrorState_q,
  uint8_t  motor_nr;                                                            /*!< Motor number */
);

/*! \brief Get Error State - answer
 *
 */
DECLARE(GetErrorState_a,
  int8_t Error;                                                                 /*!< Error status of motor */
);

int StartMotor          (struct StartMotor_q* q,          struct StartMotor_a* a);
int StopMotor           (struct StopMotor_q* q,           struct StopMotor_a* a);
int SetMotorParameter   (struct SetMotorParameter_q* q,   struct SetMotorParameter_a* a);
int GetMotorParameter   (struct GetMotorParameter_q* q,   struct GetMotorParameter_a* a);
int SetPIControl        (struct SetPIControl_q* q,        struct SetPIControl_a* a);
int GetPIControl        (struct GetPIControl_q* q,        struct GetPIControl_a* a);
int GetChannelDependand (struct GetChannelDependand_q* q, struct GetChannelDependand_a* a);
int SetChannelDependand (struct SetChannelDependand_q* q, struct SetChannelDependand_a* a);
int SetSystemDependand  (struct SetSystemDependand_q* q,  struct SetSystemDependand_a* a);
int GetSystemDependand  (struct GetSystemDependand_q* q,  struct GetSystemDependand_a* a);
int SetMotorSet         (struct SetMotorSet_q* q,         struct SetMotorSet_a* a);
int GetMotorState       (struct GetMotorState_q* q,       struct GetMotorState_a* a);
int StoreParameters     (struct StoreParameters_q* q,     struct StoreParameters_a* a);
int ClearParameters     (struct ClearParameters_q* q,     struct ClearParameters_a* a);
int ConfigureUARTSpeed  (struct ConfigureUARTSpeed_q* q,  struct ConfigureUARTSpeed_a* a);
int GetDSOSnaphot       (struct GetDSOSnaphot_q* q,       struct GetDSOSnaphot_a* a);
int ConfigDSOLog        (struct ConfigDSOLog_q* q,        struct ConfigDSOLog_a* a);
int GetDSOLogData       (struct GetDSOLogData_q* q,       struct GetDSOLogData_a* a);
int GetSystemLoad       (struct GetSystemLoad_q* q,       struct GetSystemLoad_a* a);
int GetBoardInfo        (struct GetBoardInfo_q* q,        struct GetBoardInfo_a* a);
int ConfigureHsDSO      (struct ConfigureHsDSO_q* q,      struct ConfigureHsDSO_a* a);
int GetFWVersion        (struct GetFWVersion_q* q,        struct GetFWVersion_a* a);
int DoTurn              (struct DoTurn_q* q,              struct DoTurn_a* a);
int GetTurnNumber       (struct GetTurnNumber_q* q,       struct GetTurnNumber_a* a);
int GetTemperature      (struct GetTemperature_q* q,      struct GetTemperature_a* a);
int GetErrorState       (struct GetErrorState_q* q,       struct GetErrorState_a* a);

/* this hack is used to calculate the maximum size of any of the structures.
if you add a new function with new structures, don't forget to change this 
macro accordingly. */

#define FIND_MAX_STRUCT(x) \
__packed union max_ ## x \
{ \
  struct StartMotor_ ## x                 i1; \
  struct StopMotor_ ## x                  i2; \
  struct SetMotorParameter_ ## x          i3; \
  struct GetMotorParameter_ ## x          i4; \
  struct SetPIControl_ ## x               i5; \
  struct GetPIControl_ ## x               i6; \
  struct GetChannelDependand_ ## x        i7; \
  struct SetSystemDependand_ ## x         i8; \
  struct GetSystemDependand_ ## x         i9; \
  struct SetMotorSet_ ## x                i10; \
  struct GetMotorState_ ## x              i11; \
  struct StoreParameters_ ## x            i12; \
  struct ClearParameters_ ## x            i13; \
  struct ConfigureUARTSpeed_ ## x         i14; \
  struct GetDSOSnaphot_ ## x              i15; \
  struct ConfigDSOLog_ ## x               i16; \
  struct GetDSOLogData_ ## x              i17; \
  struct GetSystemLoad_ ## x              i18; \
  struct GetBoardInfo_ ## x               i19; \
  struct ConfigureHsDSO_ ## x             i20; \
  struct GetFWVersion_ ## x               i21; \
  struct DoTurn_ ## x                     i22; \
  struct GetTurnNumber_ ## x              i23; \
  struct GetTemperature_ ## x             i24; \
  struct GetErrorState_ ## x              i25; \
  struct SetChannelDependand_ ## x        i26; \
} __attribute__((packed))

#define MAX_SIZE_Q  sizeof(FIND_MAX_STRUCT(q))
#define MAX_SIZE_A  sizeof(FIND_MAX_STRUCT(a))

enum function_error { FUNCTION_ERROR_NONE, FUNCTION_ERROR_TIMEOUT, FUNCTION_ERROR_CRC, FUNCTION_ERROR_EINVAL, };

#endif /* _ACTION_H_ */
