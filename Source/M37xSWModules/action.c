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
#include <stdio.h>

#include "config.h"
#include BOARD_BOARD_HEADER_FILE

#ifdef USE_LED
#include BOARD_LED_HEADER_FILE
#endif /* USE_LED */

#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "load_statistics.h"
#include "action.h"
#include "ve.h"
#include "pmd.h"
#include "dso.h"
#include "hsdso.h"
#include "turn_control.h"
#include "encoder.h"
#include "config_storage.h"
#include "motorctrl.h"
#include "temperature_control.h"
#include "board.h"
#include "hv_serial_communication.h"

#ifdef USE_LOAD_DEPENDANT_SPEED_REDUCTION
extern int32_t VE_TargetSpeed_Backup[MAX_CHANNEL];
#endif /* USE_LOAD_DEPENDANT_SPEED_REDUCTION */

/*! \brief  Start Motor
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int StartMotor(struct StartMotor_q* q, struct StartMotor_a* a)
{

  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));

  return VE_Start(q->motor_nr);
}
    
/*! \brief  Start Motor
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
#ifdef USE_CONFIG_STORAGE
extern uint8_t g_config_storage[CONFIG_STORAGE_PAGE_SIZE];
#endif /* USE_CONFIG_STORAGE */

int StopMotor(struct StopMotor_q* q, struct StopMotor_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));

#if ((defined USE_CONFIG_STORAGE) && (defined DEBUG))
  memset(&g_config_storage[0], 0xcc, CONFIG_STORAGE_PAGE_SIZE);
  vTaskList((signed char*)&g_config_storage[0]);
  printf("Name          State   Prio  StackLeft  Number\n*********************************************");
#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_suppress = Pe167
#elif defined __KEIL__
#pragma diag_suppress = 167
#endif
  puts(&g_config_storage[0]);
#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_suppress = Pe167
#elif defined __KEIL__
#pragma diag_suppress = 167
#endif 
#endif /* (defined USE_CONFIG_STORAGE) && (define DEBUG) */
  
  
  return VE_Stop(q->motor_nr);
}
  
/*! \brief  Set Motor Parameters
  *
  * Write the Motor Parameter to Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int SetMotorParameter   (struct SetMotorParameter_q* q,   struct SetMotorParameter_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memcpy(&MotorParameterValues[q->motor_nr], &q->Parameters, sizeof(MotorParameterValues[q->motor_nr]));

  ParameterChange[q->motor_nr] |= VE_CHANGE_MOTOR_PARAMS;
  memset(a, 0, sizeof(*a));

  return 0;
}

/*! \brief  Get Motor Parameters
  *
  * Read out the Motor Parameter from Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetMotorParameter   (struct GetMotorParameter_q* q,   struct GetMotorParameter_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));
  memcpy(&a->Parameters, &MotorParameterValues[q->motor_nr], sizeof(a->Parameters));
  
  return 0;
}

/*! \brief  Set PI Parameters
  *
  * Write the PI Control Loop Parameter to Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int SetPIControl        (struct SetPIControl_q* q,        struct SetPIControl_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memcpy(&PIControl[q->motor_nr], &q->PISettings, sizeof(PIControl[q->motor_nr]));

  ParameterChange[q->motor_nr] |= VE_CHANGE_PI_PARAMS;
  memset(a, 0, sizeof(*a));
  
  return 0;
}

/*! \brief  Get PI Parameters
  *
  * Read the PI Control Loop Parameter from Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetPIControl        (struct GetPIControl_q* q,        struct GetPIControl_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));
  memcpy(&a->PISettings, &PIControl[q->motor_nr], sizeof(a->PISettings));

  return 0;
}

/*! \brief  Get Channel dependand values
  *
  * Read out the channel dependand values (as a_max, v_max, shunt_type)
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetChannelDependand   (struct GetChannelDependand_q* q,   struct GetChannelDependand_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));
  memcpy(&a->ChannelSettings, &ChannelValues[q->motor_nr], sizeof(a->ChannelSettings));

  return 0;
}

/*! \brief  Set Channel dependand values
  *
  * Set the channel dependand values (as a_max, v_max, shunt_type)
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int SetChannelDependand   (struct SetChannelDependand_q* q,   struct SetChannelDependand_a* a)
{
  dprintf("%s(%d) called\n", __func__, q->motor_nr);

#ifdef USE_RW_BOARD_SETTINGS  
  memset(a, 0, sizeof(*a));
  memcpy(&ChannelValues[q->motor_nr], &q->ChannelSettings, sizeof(q->ChannelSettings));
  ParameterChange[q->motor_nr] |= VE_CHANGE_BOARD_PARAMS_VE | VE_CHANGE_BOARD_PARAMS_PMD ;
  return 0;
#else
  return -1;
#endif
}

/*! \brief  Set System Parameters
  *
  * Write the System Parameters to Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int SetSystemDependand  (struct SetSystemDependand_q* q,  struct SetSystemDependand_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memcpy(&SystemValues[q->motor_nr], &q->SystemSettings, sizeof(SystemValues[q->motor_nr]));

  ParameterChange[q->motor_nr] |= VE_CHANGE_SYSTEM_PARAMS_VE | VE_CHANGE_SYSTEM_PARAMS_PMD;
  memset(a, 0, sizeof(*a));

  return 0;
}

/*! \brief  Get System Parameters
  *
  * Read out the System Parameters from Ram
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetSystemDependand  (struct GetSystemDependand_q* q,  struct GetSystemDependand_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memset(a, 0, sizeof(*a));
  memcpy(&a->SystemSettings, &SystemValues[q->motor_nr], sizeof(a->SystemSettings));

  return 0;
}

/*! \brief  Set Motor Speed
  *
  * Set the Target Speed of the Motor
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int SetMotorSet         (struct SetMotorSet_q* q,         struct SetMotorSet_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memcpy(&MotorSetValues[q->motor_nr], &q->Set, sizeof(MotorSetValues[q->motor_nr]));

#ifdef USE_LOAD_DEPENDANT_SPEED_REDUCTION
  VE_TargetSpeed_Backup[q->motor_nr]  = MotorSetValues[q->motor_nr].TargetSpeed;
  MotorErrorField[q->motor_nr].Error &= ~VE_SPEEDREDUCTION;
#endif /* USE_LOAD_DEPENDANT_SPEED_REDUCTION */
  
  memset(a, 0, sizeof(*a));

  return 0;
}

/*! \brief  Get actual Motor State
  *
  * Read out the current status of the motor (actual speed, current, torque)
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetMotorState       (struct GetMotorState_q* q,       struct GetMotorState_a* a)
{
//  dprintf("%s(%d) called\n", __func__,q->motor_nr);

  memcpy(&a->State, &MotorStateValues[q->motor_nr], sizeof(a->State));
  a->State.Timestamp = xTaskGetTickCount(); 

  return 0;
}

/*! \brief  Store all Parameters
  *
  * Store all Parameters to Non-Volatile Memory
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int StoreParameters     (struct StoreParameters_q* q,     struct StoreParameters_a* a)
{
  int ret = 0;

  dprintf("%s():\n", __func__);

  memset(a, 0, sizeof(*a));
#ifdef USE_CONFIG_STORAGE  
  ret = config_storage_save_config();
#endif
  
  return ret;
}

/*! \brief  Clear all Parameters
  *
  * Clear the Non-Volatile Memory 
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int ClearParameters     (struct ClearParameters_q* q,     struct ClearParameters_a* a)
{
  int ret = 0;

  dprintf("%s():\n", __func__);

  memset(a, 0, sizeof(*a));
#ifdef USE_CONFIG_STORAGE  
  ret = config_storage_clear_config();
#endif

  return ret;
}

/*! \brief  Change UART Speed
  *
  * Dynamically change the UART to a different speed
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int ConfigureUARTSpeed  (struct ConfigureUARTSpeed_q* q,  struct ConfigureUARTSpeed_a* a)
{
  /* not supported any more */
  return -1;
}

/*! \brief  Get Snapshot of DSO
  *
  * Read out one set of configured DSO Parameters
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetDSOSnaphot       (struct GetDSOSnaphot_q* q,       struct GetDSOSnaphot_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

#ifdef USE_DSO

  memset(a, 0, sizeof(*a));

  DSO_Configure_Single_Shot_Mode(q->motor_nr,q->selected_values);
  a->Timestamp=xTaskGetTickCount();
  
  return DSO_GetLogData((int16_t*)a->values.value);

#else
  memset(a, -1, sizeof(*a));
  return -1;
#endif // USE_DSO  

}

/*! \brief  Configure DSO Log
  *
  * Configure which values shall be captured by the DSO
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int ConfigDSOLog        (struct ConfigDSOLog_q* q,        struct ConfigDSOLog_a* a)
{
  dprintf("%s(%d) called\n", __func__,q->motor_nr);

#ifdef USE_DSO
  memset(a, 0, sizeof(*a));
  a->logsteps=DSO_Configure_Continuous_Mode(q->motor_nr,
                                            q->selected_values,
                                            q->trigger_on_value,
                                            q->trigger_mode,
                                            q->trigger_level,
                                            q->spread_factor);
  return 0;
#else
  memset(a, -1, sizeof(*a));
  return -1;
#endif // USE_DSO  
}

/*! \brief  Get DSO Log Data
  *
  * Read out one set of log data after DSO has finished
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetDSOLogData       (struct GetDSOLogData_q* q,       struct GetDSOLogData_a* a)
{
  dprintf("%s() called\n", __func__);

#ifdef USE_DSO
  memset(a,   0, sizeof(*a));
  return DSO_GetLogData((int16_t*)a->values.value);
#else
  memset(a, -1, sizeof(*a));
  return -1;
#endif // USE_DSO
}

/*! \brief  Get System Load
  *
  * Get the Load of the System (complete including IRQ)
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetSystemLoad       (struct GetSystemLoad_q* q,       struct GetSystemLoad_a* a)
{
//  dprintf("%s() called\n", __func__);

  memset(a,   0, sizeof(*a));
#ifdef USE_LOAD_STATISTICS
  a->SystemLoadPercentage=SYSTEMLoad_get();
  return 0;  
#else
  a->SystemLoadPercentage=0;
  return 0;
#endif // USE_LOAD_STATISTICS
}

/*! \brief  Get Board Info
  *
  * Read out some meaningful information of the board
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetBoardInfo        (struct GetBoardInfo_q* q,        struct GetBoardInfo_a* a)
{
  uint16_t  features=0;
  
  dprintf("%s() called\n", __func__);

#ifdef USE_DSO
  features |= (1<<0);
#endif /* USE_DSO */
#ifdef USE_LOAD_STATISTICS
  features |= (1<<1);
#endif /* USE_LOAD_STATISTICS */
#ifdef USE_TURN_CONTROL
  features |= (1<<2);
#endif /* USE_TURN_CONTROL */
#ifdef USE_TEMPERATURE_CONTROL
  features |= (1<<3);
#endif /* USE_TEMPERATURE_CONTROL */
#ifdef USE_HSDSO
  features |= (1<<4);
#endif /* USE_HSDSO */
#ifdef USE_RW_BOARD_SETTINGS
  features |= (1<<5);
#endif /* USE_RW_BOARD_SETTINGS */
#ifdef USE_EXTERNAL_SPEED_CONTROL
  features |= (1<<6);
#endif /* USE_EXTERNAL_SPEED_CONTROL */
#ifdef USE_SW_OVER_UNDER_VOLTAGE_DETECTION
  features |= (1<<7);
#endif /* USE_SW_OVER_UNDER_VOLTAGE_DETECTION */
#ifdef USE_MOTOR_DISCONNECT_DETECTION
  features |= (1<<8);
#endif /* USE_MOTOR_DISCONNECT_DETECTION */
 
  memset(a,   0, sizeof(*a));

  a->values.channels        = BOARD_AVAILABLE_CHANNELS;
  a->values.DSOSize         = BOARD_DSO_SIZE;
  a->values.BoardRevision   = BoardRevision;
  a->values.FirmwareFeatures= features;
  
  memset(a->values.BoardName,0,MAX_LENGTH_BOARD_NAME);
  memcpy(a->values.BoardName,BOARD_NAME,sizeof(BOARD_NAME));
  
#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
  memset(a->values.BoardNamePWR0,0,MAX_LENGTH_BOARD_NAME);
  memcpy(a->values.BoardNamePWR0,BOARD_NAME_PWR,sizeof(BOARD_NAME_PWR));
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
  memset(a->values.BoardNamePWR1,0,MAX_LENGTH_BOARD_NAME);
  memcpy(a->values.BoardNamePWR1,BOARD_NAME_PWR,sizeof(BOARD_NAME_PWR));
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */
  
  return 0;
}

/*! \brief  Configure High Speed DSO
  *
  * Configure which values shall be transmitted by the DSO
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int ConfigureHsDSO       (struct ConfigureHsDSO_q* q,     struct ConfigureHsDSO_a* a)
{
  dprintf("%s() called, enable %d\n", __func__, q->enable);

  memset(a, 0, sizeof(*a));

#ifdef USE_HSDSO
  if (q->enable)
    return HsDSO_Enable(q->motor_nr, q->selected_values, q->spread_factor);
  else
    return HsDSO_Disable(q->motor_nr);
#else
  return 0;
#endif
}

/*! \brief  Get firmware version
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetFWVersion        (struct GetFWVersion_q* q,        struct GetFWVersion_a* a)
{
  dprintf("%s() called\n", __func__);

  memset(a, 0, sizeof(*a));

  a->values.fw_version[0]= FW_VERSION_MAJOR;
  a->values.fw_version[1]= FW_VERSION_MINOR;
  
  return 0;
}

/*! \brief  Do turn
  *
  * Turns the motor a number of times
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int DoTurn              (struct DoTurn_q* q,              struct DoTurn_a* a)
{
  dprintf("%s() called\n", __func__);

#ifdef USE_TURN_CONTROL
  memset(a, 0, sizeof(*a));
  CalculateTurnRampAndTurn(q->motor_nr,
                           q->turns,
                           q->max_speed);
  return 0;
#else
  memset(a, -1, sizeof(*a));
  return -1;
#endif  
}

/*! \brief  Get Turn Number
  *
  * Get the actual value of full turns
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetTurnNumber       (struct GetTurnNumber_q* q,       struct GetTurnNumber_a* a)
{
  dprintf("%s() called\n", __func__);

#ifdef USE_TURN_CONTROL
  a->turn_number=EncoderData[q->motor_nr].FullTurns;
  return 0;
#else
  memset(a, -1, sizeof(*a));
  return -1;
#endif  
}

/*! \brief  Get Temperature
  *
  * Read out temperature value in degree Celsius from Sensor
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetTemperature      (struct GetTemperature_q* q,      struct GetTemperature_a* a)
{
  int ret = -1;

  memset(a, 0, sizeof(*a));

#ifdef USE_TEMPERATURE_CONTROL
  a->temperature = TEMPERATURE_GetActualTemperature(q->motor_nr);
  ret = 0;
#endif

  return ret;
}

/*! \brief  Get Error State
  *
  * Read out the actual error state of the motor
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetErrorState       (struct GetErrorState_q* q,       struct GetErrorState_a* a)
{
  memset(a, 0, sizeof(*a));

  a->Error = MotorErrorField[q->motor_nr].Error;

  return 0;
}

/*! \brief  Get DC-Link Voltage
  *
  * Read out the actual DC-Link voltage of the board
  *
  * @param  q: Question from Serial Protocol
  * @param  a: Answer   for  Serial Protocol
  * @retval Success
*/
int GetDCLinkVoltage    (struct GetDCLinkVoltage_q* q,    struct GetDCLinkVoltage_a* a)
{
  TEE_VE_TypeDef*     pVEx    = NULL;

  memset(a, 0, sizeof(*a));

  switch (q->motor_nr)
  {
#if defined __TMPM_370__  || defined __TMPM_376__
  case 0:
    pVEx    = TEE_VE0;
    break;
#endif /* defined __TMPM_370__  || defined __TMPM_376__ */
  case 1:
    pVEx    = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  a->Voltage = 10 * (pVEx->VDC) * VE_v_max[q->motor_nr] / (0xfff<<3);

#ifdef USE_HV_COMMUNICATION
  a->Voltage = 10 * HV_Communication_GetValue(0)*VE_v_max[q->motor_nr] / 0x3ff;
#elif defined USE_SW_OVER_UNDER_VOLTAGE_DETECTION
  a->Voltage = 10 * ADC_GetVDC(q->motor_nr) * 5000 / ChannelValues[q->motor_nr].sensitivity_voltage_measure / 0xfff;
#endif  
  
#ifdef BOARD_VDC_CHANNEL_0
  if (q->motor_nr == 0)
    a->Voltage = BOARD_VDC_CHANNEL_0*10;
#endif  

#ifdef BOARD_VDC_CHANNEL_1
  if (q->motor_nr == 1)
    a->Voltage = BOARD_VDC_CHANNEL_1*10;
#endif  
  
  return 0;
}
