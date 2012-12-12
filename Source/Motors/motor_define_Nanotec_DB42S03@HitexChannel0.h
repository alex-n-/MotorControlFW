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

#define  MOTOR_POLE_PAIRS                               4 /* [NONE] */               
#define  MOTOR_DIRECTION                                3 /* [NONE]          0 - None 1 - CW only 2 - CCW only 3 - CW and CCW */                                                                                                                                                                                                                                                                                                                                                             
#define  MOTOR_ENCODER_TYPE                             0 /* [NONE]          0 - No encoder 1 - Use encoder for speed control 2 - Use encoder for counting events */                                                                                                                                                                                                                                                                                                                         
#define  MOTOR_ENCODER_RESOLUTION                      24 /* [NONE]          Number of pulses per physical turn */                                                                                                                                                                                                                                                                                                                                                                           
#define  MOTOR_RESOLUTION_MULT                          1 /* [NONE]          Multiply factor if needed */                                                                                                                                                                                                                                                                                                                                                                                    
#define  MOTOR_ANGULAR_ACC_MAX                        250 /* [rad/sec^2]     Maximum angular acceleration Speed up limit for the motor */                                                                                                                                                                                                                                                                                                                                                    
#define  MOTOR_TORQUE_MAX                              34 /* [mNm/A] */              
#define  MOTOR_RESISTANCE                             750 /* [mOhm]          Resistance per phase */                                                                                                                                                                                                                                                                                                                                                                                         
#define  MOTOR_INDUCTANCE                            1050 /* [uH]            Inductance per phase */                                                                                                                                                                                                                                                                                                                                                                                         
#define  MOTOR_HZ_LIMIT                               350 /* [Hz]            Maximum rating for motor */                                                                                                                                                                                                                                                                                                                                                                                     
#define  MOTOR_HZ_CHANGE                               25 /* [Hz]            Change frequency from Forced- to Field-Oriented-Commutation */                                                                                                                                                                                                                                                                                                                                                  
#define  MOTOR_INIT_DELAY                             100 /* [ms]            Time to reach the start position */                                                                                                                                                                                                                                                                                                                                                                             
#define  MOTOR_IQ_START                              1000 /* [mA]            Torque current in Forced-Commutation stage Limited by IqLim */                                                                                                                                                                                                                                                                                                                                                  
#define  MOTOR_ID_START                              1400 /* [mA]            Flux current in Forced-Commutation stage Limited by IdLim */                                                                                                                                                                                                                                                                                                                                                    
#define  MOTOR_IQ_LIM                                1850 /* [mA]            Torque current maximum */                                                                                                                                                                                                                                                                                                                                                                                       
#define  MOTOR_ID_LIM                                1700 /* [mA]            Flux current maximum */                                                                                                                                                                                                                                                                                                                                                                                         
#define  MOTORID                   "Nanotec DB42S03@HitexChannel0"

#define  CONTROL_ID_KI                                  8 /* [V/As]          Integration part of PI Controller for flux current */                                                                                                                                                                                                                                                                                                                                                           
#define  CONTROL_ID_KP                                  3 /* [V/A]           Proportional part of PI Controller for flux current */                                                                                                                                                                                                                                                                                                                                                          
#define  CONTROL_IQ_KI                                  8 /* [V/As]          Integration part of PI Controller for torque current */                                                                                                                                                                                                                                                                                                                                                         
#define  CONTROL_IQ_KP                                  3 /* [V/A]           Proportional part of PI Controller for torque current */                                                                                                                                                                                                                                                                                                                                                        
#define  CONTROL_POSITION_KI                            0 /* [Hz/Vs]         Integration part of PI Controller of position estimator */                                                                                                                                                                                                                                                                                                                                                      
#define  CONTROL_POSITION_KP                           15 /* [Hz/V]          Proportional part of PI Controller of position estimator */                                                                                                                                                                                                                                                                                                                                                     
#define  CONTROL_SPEED_KI                               7 /* [mA/Hz*s]       Integration part of PI Controller of speed controller */                                                                                                                                                                                                                                                                                                                                                        
#define  CONTROL_SPEED_KP                              10 /* [mA/Hz]         Proportional part of PI Controller of speed controller */                                                                                                                                                                                                                                                                                                                                                       

#define  SYSTEM_PWM_FREQUENCY                       12500 /* [HZ] */                 
#define  SYSTEM_SHUTDOWN_MODE                           2 /* [NONE]          0 - No signals 1 - Short brake 2 - Gentle shutdown (with MaxAngAcc) */                                                                                                                                                                                                                                                                                                                                          
#define  SYSTEM_RESTART_MODE                            1 /* [NONE]          0 - Switch off motor after stall detection 1 - Restart motor after stall detection */                                                                                                                                                                                                                                                                                                                           
#define  SYSTEM_STALL_VALUE                           800 /* [NONE]          Stall detect value as vqi in stall_detect.c */                                                                                                                                                                                                                                                                                                                                                                  
#define  SYSTEM_OVERTEMP_VALUE                         40 /* [NONE]          Motor switch off when temerature exceeded (0 means switch off) */                                                                                                                                                                                                                                                                                                                                               
#define  SYSTEM_SPEED_CONTROL_MODE                      0 /* [NONE]          0 - No external speed control 1 - Speed control by external voltage signal 2 - Speed control by external PWM Signal */                                                                                                                                                                                                                                                                                          
#define  SYSTEM_SW_OVERVOLTAGE                         26 /* [V]             Overvoltage detection by Software */                                                                                                                                                                                                                                                                                                                                                                            
#define  SYSTEM_SW_UNDERVOLTAGE                        18 /* [V]             Undervoltage detection by Software */                                                                                                                                                                                                                                                                                                                                                                           

#endif
