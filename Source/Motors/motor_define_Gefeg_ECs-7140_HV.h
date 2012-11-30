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
#ifndef _MOTOR_DEFINE_H_
#define _MOTOR_DEFINE_H_

#include "motorctrl.h"
/*===================================================================*
Parameter Definition
*===================================================================*/

/* Motor Parameter */
#define	MOTOR_POLE_PAIRS          2                 /* NONE             - Half of Poles */
#define	MOTOR_DIRECTION           MOTOR_CW_CCW      /* NONE             - Enum Type (MOTOR_CW_ONLY / MOTOR_CCW_ONLY / MOTOR_CW_CCW) */
#define	MOTOR_ENCODER_TYPE        MOTOR_NO_ENCODER  /* NONE             - Enum Type (MOTOR_NO_SENSOR / MOTOR_HALL_SENSOR / MOTOR_ENCODER) */
#define	MOTOR_ENCODER_RESOLUTION  0                 /* NONE             - Number (Resolution) */
#define	MOTOR_RESOLUTION_MULT     0                 /* NONE             - Number (Multiplier) */
#define	MOTOR_ANGULAR_ACC_MAX     50                /* [rev/sec^2]      - max angular velocity */
#define	MOTOR_TORQUE_MAX          330               /* [mNm/A]          - max torgue value */
#define MOTOR_RESISTANCE          40000             /* est [mOhm]       - Winding resistance */
#define MOTOR_INDUCTANCE          34000             /* est [uH]         - Winding inductance */
#define MOTOR_HZ_LIMIT            140               /* [Hz]             - Limitation speed of motor */
#define MOTOR_HZ_CHANGE           20                /* [Hz]             - Max Forced speed of motor (no FOC) */
#define MOTOR_INIT_DELAY          300               /* [ms]             - Time of Positioning Stage */
#define MOTOR_IQ_START            100               /* [mA]             - Start current (Iq) */
#define MOTOR_ID_START            150               /* [mA]             - Start current (Id) */
#define MOTOR_IQ_LIM              500               /* [mA]             - q-axis limitation current */
#define	MOTOR_ID_LIM              500               /* [mA]             - d-axis limitation current */
#define MOTORID                   "Gefeg  ECs-7140" /* max 20 characters */

/* Control Loop Parameter */
#define CONTROL_ID_KI             20                 /* [V/As]           - d-axis current control Integral gain */
#define CONTROL_ID_KP             40                 /* [V/A]            - d-axis current control Proportional gain */
#define CONTROL_IQ_KI             20                 /* [V/As]           - q-axis current control Integral gain */
#define CONTROL_IQ_KP             40                 /* [V/A]            - q-axis current control Proportional gain */
#define CONTROL_POSITION_KI       2                  /* [Hz/Vs]          - Position estimation Integral gain (Ki) * 1000 */
#define CONTROL_POSITION_KP       10                 /* [Hz/V]           - Position estimation Proportional gain (Kp) * 1000 */
#define CONTROL_SPEED_KI          18                 /* [mA/Hz*s]        - Speed Control Integral gain */
#define CONTROL_SPEED_KP          30                 /* [mA/Hz]          - Speed Control Proportional gain */

/* System Parameter */
#define SYSTEM_PWM_FREQUENCY      12500              /* [Hz]             - PWM Frequency */
#define SYSTEM_SHUTDOWN_MODE      SHUTDOWN_GENTLE    /* NONE             - Shutdown Mode for the VE */
#define SYSTEM_RESTART_MODE       SWITCH_OFF_MOTOR   /* NONE             - Restart Motor to Target Speed when Motor stalled detected */
#define SYSTEM_STALL_VALUE        1200               /* NONE             - Stall detect value as vqi in stall_detect.c */
#define SYSTEM_OVERTEMP_VALUE     60                 /* [Degrees Celsius]- Motor switch off when temerature exceeded (0 means switch off) */
#define SYSTEM_SPEED_CONTROL_MODE 0                  /* NONE             - 0 - No speed control / 1 - ADC / 2 - PWM */

#endif
