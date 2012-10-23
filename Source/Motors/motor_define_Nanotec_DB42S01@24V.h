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

/* Motor Parameter */
#define MOTOR_POLE_PAIRS          4
#define MOTOR_DIRECTION           3
#define MOTOR_ENCODER_TYPE        MOTOR_NO_ENCODER
#define MOTOR_ENCODER_RESOLUTION  0
#define MOTOR_RESOLUTION_MULT     0
#define MOTOR_ANGULAR_ACC_MAX     500                   /* rad/sec^2 */
#define MOTOR_TORQUE_MAX          57                    /* mNm/A */
#define MOTOR_RESISTANCE          3500                  /* mOhm */
#define MOTOR_INDUCTANCE          5800                  /* uH */
#define MOTOR_HZ_LIMIT            267                   /* Hz */
#define MOTOR_HZ_CHANGE           25                    /* Hz */
#define MOTOR_INIT_DELAY          100                   /* ms */
#define MOTOR_IQ_START            500                   /* mA */
#define MOTOR_ID_START            500                   /* mA */
#define MOTOR_IQ_LIM              800                   /* mA */
#define MOTOR_ID_LIM              800                   /* mA */
#define MOTORID "Nanotec DB42S01"

/* Control Loop Parameter */
#define CONTROL_ID_KI             100                   /* V/As */
#define CONTROL_ID_KP             30                    /* V/A */
#define CONTROL_IQ_KI             100                   /* V/As */
#define CONTROL_IQ_KP             30                    /* V/A */
#define CONTROL_POSITION_KI       2                     /* Hz/Vs */
#define CONTROL_POSITION_KP       10                    /* Hz/V */
#define CONTROL_SPEED_KI          20                    /* mA/Hz*s */
#define CONTROL_SPEED_KP          20                    /* mA/Hz */

/* System Parameter */
#define SYSTEM_DEAD_TIME          2500                  /* ns */
#define SYSTEM_PWM_FREQUENCY      12500                 /* HZ */
#define SYSTEM_SHUTDOWN_MODE      2
#define SYSTEM_RESTART_MODE       SWITCH_OFF_MOTOR      /* NONE             - Restart Motor to Target Speed when Motor stalled detected */
#define SYSTEM_STALL_VALUE        0                     /* NONE             - Stall detect value as vqi in stall_detect.c */
#define SYSTEM_OVERTEMP_VALUE     0                     /* [Degrees Celsius]- Motor switch off when temerature exceeded (0 means switch off) */
#define SYSTEM_SPEED_CONTROL_MODE 0                     /* NONE             - 0 - No speed control / 1 - ADC / 2 - PWM */

#endif
