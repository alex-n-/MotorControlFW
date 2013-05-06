/*!< THE SOURCE CODE AND ITS
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

#ifndef _MOTORCTRL_H_
#define _MOTORCTRL_H_

#include <stdint.h>
#include "config.h"
#include "adc.h"
#include "dso.h"

#define MAX_LENGTH_MOTOR_ID   20                                                /*!< Number of characters for Motor ID */
#define MAX_LENGTH_BOARD_NAME 20                                                /*!< Number of characters for Board Name */

/*! \brief Motor rotation direction.
 *
 *  This enumeration displays the different rotation directions of the motor
 */
typedef enum {
  MOTOR_OFF     = (1<<0),                                                       /*!< Motor switched off */
  MOTOR_CW      = (1<<1),                                                       /*!< Motor turning clock-wise */
  MOTOR_CCW     = (1<<2),                                                       /*!< Motor turning counter-clock-wise */
  MOTOR_OFFLINE = (1<<3),
} MOTOR_STATE;

/*! \brief Motor encoder usage type.
 *
 *  This enumeration selects the encoder usage for the Motor
 */
typedef enum {
  MOTOR_NO_ENCODER,                                                             /*!< Motor has NO Encoder - use SW emulation for event counting */
  MOTOR_ENCODER_SPEED,                                                          /*!< Motor is having en Encoder (Hall/Incremental) used for Speed Control */
  MOTOR_ENCODER_EVENT,                                                          /*!< Motor is having an Encoder (Hall/Incremental) used for  Counting Events */
} MOTOR_ENCODER; 

/*! \brief Motor turning possibilities.
 *
 *  This enumeration displays the different rotation possibilities the motor is capeable of.
 */
typedef enum {
  MOTOR_NONE,                                                                   /*!< No Motor define */
  MOTOR_CW_ONLY,                                                                /*!< Motor can turn clock-wise only */
  MOTOR_CCW_ONLY,                                                               /*!< Motor can turn counter-clock-wise only */
  MOTOR_CW_CCW,                                                                 /*!< Motor can turn clock-wise and counter-clock wise */
} MOTOR_DIRECTION;

/*! \brief Avaliable vector engine channels.
 *
 *  This enumeration is for selecting the available ve channels for the board.
 */
typedef enum {
  VE_CHANNEL_0  = (1<<0),                                                       /*!< Vector Engine Channel 0 is available */
  VE_CHANNEL_1  = (1<<1),                                                       /*!< Vector Engine Channel 1 is available */
  VE_CHANNEL_2  = (1<<2),                                                       /*!< Vector Engine Channel 2 is available */
} VE_CHANNELS;

/*! \brief Motor Error Flags.
 *
 *  This enumeration is for the different error flags in motor state
 */
typedef enum {
  VE_ERROR_NONE       = 0,                                                      /*!< No error */  
  VE_OVERTEMPERATURE  = (1<<0),                                                 /*!< Over temperature detected */  
  VE_EMERGENCY        = (1<<1),                                                 /*!< Emergency Signal Detected */
  VE_OVERVOLTAGE      = (1<<2),                                                 /*!< Overvoltage Signal Detected */
  VE_NOMOTOR          = (1<<3),
  VE_SWUNDERVOLTAGE   = (1<<4),
  VE_SWOVERVOLTAGE    = (1<<5),
  VE_SPEEDREDUCTION   = (1<<7),
} VE_ERROR;

/*! \brief Parameter Changed Flags.
 *
 *  This enumeration is for the different uploaded parameter sets
 */
typedef enum {
  VE_CHANGE_MOTOR_PARAMS      = (1<<0),                                         /*!< New motor parameter set */  
  VE_CHANGE_PI_PARAMS         = (1<<1),                                         /*!< New PI control parameter set */
  VE_CHANGE_SYSTEM_PARAMS_VE  = (1<<2),                                         /*!< New system values parameter set */
  VE_CHANGE_SYSTEM_PARAMS_PMD = (1<<2),                                         /*!< New system values parameter set */
  VE_CHANGE_BOARD_PARAMS_VE   = (1<<3),                                         /*!< New system values parameter set */
  VE_CHANGE_BOARD_PARAMS_PMD  = (1<<3),                                         /*!< New system values parameter set */
} VE_CHANGED_VALUE;

/*! \brief External Speed control flags.
 *
 *  This enumeration is for the different external speed control modes
 */
typedef enum {
  ESC_NONE  = 0,                                                                /*!< No External Speed Control */
  ESC_ADC   = 1,                                                                /*!< External Speed Control via ADC */
  ESC_PWM   = 2,                                                                /*!< External Speed Control via PWM */
} VE_ESC_VALUE;


/*! \brief Motor Parameters.
 *
 *  All parameters needed to describe a motor fully
 */
typedef __packed struct
{
  uint8_t   PolePairs;                                                          /*!< NONE             - Half of Poles */
  uint8_t   Direction;                                                          /*!< NONE             - Enum Type (MOTOR_CW_ONLY / MOTOR_CCW_ONLY / MOTOR_CW_CCW) */
  uint8_t   Encoder;                                                            /*!< NONE             - Enum Type (MOTOR_NO_SENSOR / MOTOR_HALL_SENSOR / MOTOR_ENCODER) */
  uint16_t  EncRes;                                                             /*!< NONE             - Number (Resolution) */
  uint8_t   EncMult;                                                            /*!< NONE             - Number (Multiplier) */
  uint16_t  MaxAngAcc;                                                          /*!< [rad/sec^2]      - max angular velocity */
  uint16_t  TorqueFactor;                                                       /*!< [mNm/A]          - max torgue value * 10 */
  uint32_t  Resistance;                                                         /*!< [mOhm]           - Winding resistance */
  uint32_t  Inductance;                                                         /*!< [uH]             - Winding inductance */
  uint16_t  HzLimit;                                                            /*!< [Hz]             - Limitation speed of motor */
  uint16_t  HzChange;                                                           /*!< [Hz]             - Max Forced speed of motor (no FOC) */
  uint16_t  PositionDelay;                                                      /*!< [ms]             - Time of Positioning Stage */
  uint32_t  IqStart;                                                            /*!< [mA]             - Start current (Iq) */
  uint32_t  IdStart;                                                            /*!< [mA]             - Start current (Id) */
  uint32_t  IqLim;                                                              /*!< [mA]             - q-axis limitation current */
  uint32_t  IdLim;                                                              /*!< [mA]             - d-axis limitation current */
  uint8_t   MotorId[MAX_LENGTH_MOTOR_ID];                                       /*!< NONE             - name of the Motor device */
} __attribute__((packed)) MotorParameters;


/*! \brief PI Control settings.
 *
 *  Settings for the PI Control Loop (also at the different stages)
 */
typedef __packed struct
{
  uint32_t Id_Ki;                                                               /*!< [V/As]           - d-axis current control Integral gain */
  uint32_t Id_Kp;                                                               /*!< [V/A]            - d-axis current control Proportional gain */
  uint32_t Iq_Ki;                                                               /*!< [V/As]           - q-axis current control Integral gain */
  uint32_t Iq_Kp;                                                               /*!< [V/A]            - q-axis current control Proportional gain */
  uint32_t Position_Ki;                                                         /*!< [Hz/Vs] * 1000   - Position estimation Integral gain (Ki) * 1000 */
  uint32_t Position_Kp;                                                         /*!< [Hz/V] * 1000    - Position estimation Proportional gain (Kp) * 1000 */
  uint32_t Speed_Ki;                                                            /*!< [mA/Hz*s]        - Speed Control Integral gain */
  uint32_t Speed_Kp;                                                            /*!< [mA/Hz]          - Speed Control Proportional gain */
} __attribute__((packed)) PIControlSettings;

/*! \brief channel dependend values.
 *
 *  Values that are purely channel dependend
 */
typedef __packed struct
{
  uint16_t  DeadTime;                                                           /*!< [ns]             - Dead time of output switching polarity unit (steps of 100 ns) */
  uint16_t  BootstrapDelay;                                                      /*!< [ms]             - Bootstrap time */
  uint16_t  sensitivity_current_measure;                                        /*!< [mV/A]           - Input current(Iabc) at ADC =0xFFF0 */
  uint16_t  sensitivity_voltage_measure;                                        /*!< [mV/V]           - Input voltage(Vdc)   at ADC =0xFFF0 */
  uint8_t   gain_current_measure;                                               /*!< NONE             - Gain setting for current measurement with 3 input lines (3Shunt ro 2Sensor) */
  uint8_t   measurement_type;                                                   /*!< NONE             - Enum Type (SHUNT_TYPE_1 = 1,SHUNT_TYPE_3 = 3) */
  uint8_t   sensor_direction;                                                   /*!< NONE             - Direction of the current sensors */  
  uint8_t   poll;                                                               /*!< NONE             - Active state for lower side FETs */
  uint8_t   polh;                                                               /*!< NONE             - Active state for lower side FETs */
} __attribute__((packed)) ChannelDependandValues;

/*! \brief System settings.
 *
 *  Values that are dependend of the combination of Board and Motor
 */
typedef __packed struct
{
  uint16_t  PWMFrequency;                                                       /*!< [Hz]             - PWM Frequency */
  uint8_t   ShutdownMode;                                                       /*!< NONE             - Shutdown Mode for the VE */
  uint8_t   RestartMode;                                                        /*!< NONE             - Restart behaviour */
  uint16_t  StallDetectValue;                                                   /*!< NONE             - vqi value for stall detection */
  int8_t    Overtemperature;                                                    /*!< [Degree Celsius] - Shutdown if temperature exceeds this temperature setting */
  uint8_t   can_id;                                                             /*!< NONE             - CAN ID of Board */
  uint8_t   ExternalSpeedCtrl;                                                  /*!< NONE             - Enum Type (ESC_NONE = 0, ESC_ADC = 1, ESC_PWM = 2) */
  uint16_t  SW_Overvoltage;                                                     /*!< [V]              - Voltage for SW Overvolage detection */
  uint16_t  SW_Undervoltage;                                                    /*!< [V]              - Voltage for SW Undervoltage detection */
} __attribute__((packed)) SystemDependandValues;


/*! \brief Board info values.
 *
 *  Values that give inforamtion about the board
 */
typedef __packed struct
{
  uint8_t   channels;                                                           /*!< NONE             - Bitfield for determine the availabble motor channels */
  uint16_t  DSOSize;                                                            /*!< NONE             - Number of Data Samples for DSO functionality */
  uint8_t   BoardName[MAX_LENGTH_BOARD_NAME];                                   /*!< NONE             - name of the board */
  uint8_t   BoardNamePWR0[MAX_LENGTH_BOARD_NAME];                               /*!< NONE             - name of the power board channel 0*/
  uint8_t   BoardNamePWR1[MAX_LENGTH_BOARD_NAME];                               /*!< NONE             - name of the power board channel 1*/
  uint8_t   BoardRevision;                                                      /*!< NONE             - revision of the board */
  uint16_t  FirmwareFeatures;                                                   /*!< NONE             - features compiled into firmware */
} __attribute__((packed)) BoardInfoValues;

/*! \brief Firmware Version values.
 *
 *  Values that give inforamtion about the board
 */
typedef __packed struct
{
  uint8_t   fw_version[2];                                                      /*!< NONE             - Version information [Major][Minor] */
} __attribute__((packed)) FWVersion;


/*! \brief Motor settings.
 *
 *  Target speed Speed and target direction for the Motor.
 */
typedef __packed struct
{
  int32_t TargetSpeed;                                                          /*!< [rpm]            - Target Rotation speed of Motor - negative is CCW*/
} __attribute__((packed)) MotorSetSettings;

/*! \brief Motor state.
 *
 *  Actual speed, direction, used current and used torque at the time of request
 */
typedef __packed struct
{
  int32_t   ActualSpeed;                                                        /*!< [rpm]              - Actual Rotation speed of Motor - negative is CCW*/
  int32_t   TargetSpeed;                                                        /*!< [rpm]              - Target Rotation speed of Motor - negative is CCW*/
  uint32_t  Current;                                                            /*!< [mA]               - Current of motor */
  uint32_t  Torque;                                                             /*!< [Nm/mA] *10        - Torque of Motor */
  uint16_t  Reserved;
  uint32_t  Timestamp;                                                          /*!< [systicks]         - systick counter of system */
} __attribute__((packed)) MotorStateSettings;

/*! \brief Motor Error.
 *
 *  Signaling different errors that could have appeared
 */
typedef __packed struct
{
  uint8_t  Error;
} __attribute__((packed)) MotorError;

/*! \brief DSO Logging data set.
 *
 *  One data set of sampled DSO data during logging.
 */
typedef __packed struct
{
  int16_t  value[MAX_DSO_VALUE];
} __attribute__((packed)) DSOValues;


/* GLOBAL Variables */
extern MotorSetSettings        MotorSetValues[MAX_CHANNEL];
extern MotorStateSettings      MotorStateValues[MAX_CHANNEL];
extern MotorParameters         MotorParameterValues[MAX_CHANNEL];
extern MotorError              MotorErrorField[MAX_CHANNEL];
extern PIControlSettings       PIControl[MAX_CHANNEL];
extern ChannelDependandValues  ChannelValues[MAX_CHANNEL];
extern SystemDependandValues   SystemValues[MAX_CHANNEL];
extern uint8_t                 ParameterChange[MAX_CHANNEL];

#endif /*!< _MOTORCTRL_H_ */

