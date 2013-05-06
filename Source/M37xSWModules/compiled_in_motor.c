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
#include <string.h>

#include "config.h"
#ifdef USE_INTERNAL_MOTOR_PARAMS

#include "motorctrl.h"

/*! \brief  Setup the internal Motor Parameters
  *
  * If the motor parameters are compiled into the Firmware
  * this is setting up the values to Ram
  *
  * @param  None
  * @retval None
*/
void SetupInternalMotorParams(void)
{
  /* Setup compiled in values for Motor 0 */ 
#ifdef MOTOR_CHANNEL_0
#include MOTOR_CHANNEL_0
  MotorParameterValues[0].PolePairs      = MOTOR_POLE_PAIRS;
  MotorParameterValues[0].Direction      = MOTOR_DIRECTION;
  MotorParameterValues[0].Encoder        = MOTOR_ENCODER_TYPE;
  MotorParameterValues[0].EncRes         = MOTOR_ENCODER_RESOLUTION;
  MotorParameterValues[0].EncMult        = MOTOR_RESOLUTION_MULT;
  MotorParameterValues[0].MaxAngAcc      = MOTOR_ANGULAR_ACC_MAX;
  MotorParameterValues[0].TorqueFactor   = MOTOR_TORQUE_MAX;
  MotorParameterValues[0].Resistance     = MOTOR_RESISTANCE;
  MotorParameterValues[0].Inductance     = MOTOR_INDUCTANCE;
  MotorParameterValues[0].HzLimit        = MOTOR_HZ_LIMIT;
  MotorParameterValues[0].HzChange       = MOTOR_HZ_CHANGE;
  MotorParameterValues[0].PositionDelay  = MOTOR_POSITION_DELAY;
  MotorParameterValues[0].IqStart        = MOTOR_IQ_START;
  MotorParameterValues[0].IdStart        = MOTOR_ID_START;
  MotorParameterValues[0].IqLim          = MOTOR_IQ_LIM;
  MotorParameterValues[0].IdLim          = MOTOR_ID_LIM;
  memcpy (&MotorParameterValues[0].MotorId,MOTORID,sizeof(MOTORID));
  
  PIControl[0].Id_Ki                     = CONTROL_ID_KI;
  PIControl[0].Id_Kp                     = CONTROL_ID_KP;
  PIControl[0].Iq_Ki                     = CONTROL_IQ_KI;
  PIControl[0].Iq_Kp                     = CONTROL_IQ_KP;
  PIControl[0].Position_Ki               = CONTROL_POSITION_KI;
  PIControl[0].Position_Kp               = CONTROL_POSITION_KP;
  PIControl[0].Speed_Ki                  = CONTROL_SPEED_KI;
  PIControl[0].Speed_Kp                  = CONTROL_SPEED_KP;

  SystemValues[0].PWMFrequency           = SYSTEM_PWM_FREQUENCY;
  SystemValues[0].ShutdownMode           = SYSTEM_SHUTDOWN_MODE;
  SystemValues[0].RestartMode            = SYSTEM_RESTART_MODE;
  SystemValues[0].StallDetectValue       = SYSTEM_STALL_VALUE;
  SystemValues[0].Overtemperature        = SYSTEM_OVERTEMP_VALUE;
  SystemValues[0].ExternalSpeedCtrl      = SYSTEM_SPEED_CONTROL_MODE;
  SystemValues[0].SW_Overvoltage         = SYSTEM_SW_OVERVOLTAGE;
  SystemValues[0].SW_Undervoltage        = SYSTEM_SW_UNDERVOLTAGE;

  /* A little bit tricky - undef all defines to be able to load the same
    define again, or ause an other define, but having the same naming */
#include "motor_undefine.h"
#endif

/* Setup compiled in values for Motor 1 */ 
#ifdef MOTOR_CHANNEL_1
#include MOTOR_CHANNEL_1

  MotorParameterValues[1].PolePairs      = MOTOR_POLE_PAIRS;
  MotorParameterValues[1].Direction      = MOTOR_DIRECTION;
  MotorParameterValues[1].Encoder        = MOTOR_ENCODER_TYPE;
  MotorParameterValues[1].EncRes         = MOTOR_ENCODER_RESOLUTION;
  MotorParameterValues[1].EncMult        = MOTOR_RESOLUTION_MULT;
  MotorParameterValues[1].MaxAngAcc      = MOTOR_ANGULAR_ACC_MAX;
  MotorParameterValues[1].TorqueFactor   = MOTOR_TORQUE_MAX;
  MotorParameterValues[1].Resistance     = MOTOR_RESISTANCE;
  MotorParameterValues[1].Inductance     = MOTOR_INDUCTANCE;
  MotorParameterValues[1].HzLimit        = MOTOR_HZ_LIMIT;
  MotorParameterValues[1].HzChange       = MOTOR_HZ_CHANGE;
  MotorParameterValues[1].PositionDelay  = MOTOR_POSITION_DELAY;
  MotorParameterValues[1].IqStart        = MOTOR_IQ_START;
  MotorParameterValues[1].IdStart        = MOTOR_ID_START;
  MotorParameterValues[1].IqLim          = MOTOR_IQ_LIM;
  MotorParameterValues[1].IdLim          = MOTOR_ID_LIM;
  memcpy (&MotorParameterValues[1].MotorId,MOTORID,sizeof(MOTORID));
  
  PIControl[1].Id_Ki                     = CONTROL_ID_KI;
  PIControl[1].Id_Kp                     = CONTROL_ID_KP;
  PIControl[1].Iq_Ki                     = CONTROL_IQ_KI;
  PIControl[1].Iq_Kp                     = CONTROL_IQ_KP;
  PIControl[1].Position_Ki               = CONTROL_POSITION_KI;
  PIControl[1].Position_Kp               = CONTROL_POSITION_KP;
  PIControl[1].Speed_Ki                  = CONTROL_SPEED_KI;
  PIControl[1].Speed_Kp                  = CONTROL_SPEED_KP;

  SystemValues[1].PWMFrequency           = SYSTEM_PWM_FREQUENCY;
  SystemValues[1].ShutdownMode           = SYSTEM_SHUTDOWN_MODE;
  SystemValues[1].RestartMode            = SYSTEM_RESTART_MODE;
  SystemValues[1].StallDetectValue       = SYSTEM_STALL_VALUE;
  SystemValues[1].Overtemperature        = SYSTEM_OVERTEMP_VALUE;
  SystemValues[1].ExternalSpeedCtrl      = SYSTEM_SPEED_CONTROL_MODE;
  SystemValues[1].SW_Overvoltage         = SYSTEM_SW_OVERVOLTAGE;
  SystemValues[1].SW_Undervoltage        = SYSTEM_SW_UNDERVOLTAGE;

#include "motor_undefine.h"
#endif
}  
#endif
