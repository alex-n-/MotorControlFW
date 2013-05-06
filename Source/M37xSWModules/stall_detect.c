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
 * OTHERWISE. (C)Copyright TOSHIBA CORPORATION 2012 All rights reserved
 */

#include <stdio.h>
#include <string.h>
#include "tx03_common.h"

#include "config.h"
#ifdef USE_STALL_DETECT

#include "FreeRTOS.h"
#include "task.h"

#include "ve.h"
#include "motorctrl.h"

#define INIT_DELAY                    100
#define JUMP_STALL_DETECT_PERCENTAGE   70

static int16_t  vqi[MAX_CHANNEL];
static int16_t  old_vqi[MAX_CHANNEL];
static uint8_t  stall_detected[MAX_CHANNEL];

/*! \brief Check for magnetic field stall
  *
  * called during VE IRQ
  *
  * @param  channel_number: channel to check
  * @retval None
*/
void STALL_Detect (uint8_t channel_number)
{
  TEE_VE_TypeDef*   pVEx = NULL;
  
  switch (channel_number)
  {
  case 0:
    pVEx  = TEE_VE0;
    break;
  case 1:
    pVEx  = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }
  
  vqi[channel_number]=pVEx->VQIH>>16;                                           /* get new value */

}

/*! \brief  Stall Detect Worker Task
  *
  * @param  pvParameters: None
  * @retval None
*/
void StallTask(void* pvParameters)
{
  unsigned char i;
  int32_t       target_speed;
  uint16_t      init_delay;
  uint8_t       shutdown_mode;
  
  while(1)
  {
    for (i=0;i<MAX_CHANNEL;i++)
    {
      
      /* Vqi value drops/rises around 70% in the case that the macnetic field stalls
         check for a 30% drop for detecting - only when in FOC
         This works quite well when using Encoder and also for the first time when using 
         sensorless.
         In sensorless case it happens that on the second startup the fiels stalls when 
         changing to FOC - but there is no Vqi drop measureable.
         Therefor check for a Vqi that is below a border (get this border by fixing the moto and check
         the final value Vqi in DSO.
      */
      
      if (VE_ActualStage[i].main==Stage_FOC && SystemValues[i].StallDetectValue != 0)
      {
        if (VE_Omega[i].part.reg>0)
          if ((old_vqi[i]*JUMP_STALL_DETECT_PERCENTAGE/100>vqi[i])
            || (    (vqi[i]<SystemValues[i].StallDetectValue)
                 && (MotorParameterValues[i].Encoder  == MOTOR_NO_ENCODER)))
            stall_detected[i]=1;                                                /* make filed as stalled - Worker Task will take action */
        if (VE_Omega[i].part.reg<0)
          if ((old_vqi[i]*JUMP_STALL_DETECT_PERCENTAGE/100<vqi[i])
            || (    (vqi[i] > -SystemValues[i].StallDetectValue)
                 && (MotorParameterValues[i].Encoder  == MOTOR_NO_ENCODER)))
            stall_detected[i]=1;                                                /* make filed as stalled - Worker Task will take action */
      }
      old_vqi[i]=vqi[i];
      
      if (stall_detected[i]==1)                                                 /* check for stalled field */
      {
        target_speed = MotorSetValues[i].TargetSpeed;                           /* remember original target value for restart case */
        shutdown_mode = SystemValues[i].ShutdownMode;                           /* remember original shutdown mode */
        SystemValues[i].ShutdownMode = 0;                                       /* change to short break shutdown -> motro is already not spinning anymore*/   
        VE_Stop(i);                                                             /* Stop the motor */
        vTaskDelay( 100 / portTICK_RATE_MS );                                   /* wait a moment to complete shutdown */
        SystemValues[i].ShutdownMode  = shutdown_mode ;                         /* restore original shutdown mode */
        stall_detected[i]=0;                                                    /* reset stall flag */
        if (SystemValues[i].RestartMode == RESTART_MOTOR)                       /* system restart mode ? */
        {
          init_delay  = MotorParameterValues[i].PositionDelay;                  /* remember original init delay */
          MotorParameterValues[i].PositionDelay = INIT_DELAY;                   /* set init delay */
          VE_Start(i);                                                          /* restart the motor */
          MotorSetValues[i].TargetSpeed     = target_speed;                     /* restore the original target speed */
          MotorParameterValues[i].PositionDelay = init_delay;                   /* restore the original init delay */
        }
      }
    }
    vTaskDelay( 100 / portTICK_RATE_MS );                                       /* sleep a while */
  }
}
#endif // USE_STALL_DETECT
