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

#include <stdint.h>
#include <stdlib.h>

#include "config.h"
#ifdef USE_TURN_CONTROL

#include "FreeRTOS.h"
#include "task.h"

#include "ve.h"
#include "motorctrl.h"
#include "encoder.h"

#define APPROACH_RPM      10
#define LOW_SPEED_BORDER  30

uint8_t  do_turn_control[MAX_CHANNEL];
static int32_t  MaxSpeed[MAX_CHANNEL];
static int32_t  ForcedSpeed[MAX_CHANNEL];
static uint32_t WaitValueForShutdown[MAX_CHANNEL];
static int32_t  TargetValue[MAX_CHANNEL];
static uint32_t WaitValueForForced[MAX_CHANNEL];

/*! \brief  Calculate Turn Ramp
  *
  * Calculate the different values for the Turn Control Ramp
  *
  * @param  channel_number: channel number to use
  * @param  nr_turns: number of turns
  * @param  max_speed: maximum rpm to reach while turning
  * @retval None
*/
void CalculateTurnRampAndTurn(uint8_t channel_number,int16_t nr_turns,uint16_t max_speed)
{
  uint16_t SpeedUpTime;
  uint16_t TurnsWhileSpeedUp;
  uint16_t SpeedDownTime;
  uint16_t TurnsWhileSpeedDown;

  /* Calculate new absolute position */
  TargetValue[channel_number] += nr_turns;

recalculate:
  /* remember the maximum rpm */  
  MaxSpeed[channel_number]    =  max_speed;

  /* Calculate border of Forced-/Field-Oriented-Commutation */    
  ForcedSpeed[channel_number]=   60
                               * MotorParameterValues[channel_number].HzChange
                               / MotorParameterValues[channel_number].PolePairs
                               - 5;                                           /* Security margin */

  /* Calculate the time needed to reach target rpm in ms */  
  SpeedUpTime = (uint16_t) ( (long long)PAI2
                                      * max_speed
                                      * MotorParameterValues[channel_number].PolePairs
                                      / MotorParameterValues[channel_number].MaxAngAcc
                                      / SECONDS_PER_MINUTE
                                      / 100000);
  /* Calculate the number of tuns happen during speed up */
  TurnsWhileSpeedUp =   max_speed
                      / 2                                                     /* spinning the whole time with half the speed */
                      / SECONDS_PER_MINUTE
                      * SpeedUpTime
                      / 1000;
  
  /* Calculate the time for stopping from forced commutation stage */
  SpeedDownTime = (uint16_t) ( (long long)PAI2
                                        * ForcedSpeed[channel_number]
                                        * MotorParameterValues[channel_number].PolePairs
                                        / MotorParameterValues[channel_number].MaxAngAcc
                                        / SECONDS_PER_MINUTE
                                        / 100000);
                          
  /* Calculate the Turns during stopping from force commutation speed */
  TurnsWhileSpeedDown =   ForcedSpeed[channel_number]
                        / 2 
                        / SECONDS_PER_MINUTE
                        * SpeedDownTime
                        / 1000;

  /* Safety check - when speeding up / down + safety margin makes more turns than just reaching the target speed */
  if (abs(nr_turns)<3*TurnsWhileSpeedUp)
  {
    max_speed /=2;
    goto recalculate;
  }
  
  /* if less than 30 turns - calculate the speed dynamic */
  if ((abs(nr_turns)<LOW_SPEED_BORDER) && (abs(max_speed!=nr_turns*10)))
  {
    max_speed =nr_turns*10;
    goto recalculate;
  }
  
  /* When maximum speed lower than forced commutation speed - limit this */
  if (max_speed<ForcedSpeed[channel_number])
    ForcedSpeed[channel_number]=max_speed;
  
  if(nr_turns<0)
  {
    MaxSpeed[channel_number]    *= -1;
    ForcedSpeed[channel_number] *= -1;

    WaitValueForShutdown[channel_number] = TargetValue[channel_number] + TurnsWhileSpeedDown;
    WaitValueForForced[channel_number]   = TargetValue[channel_number] + TurnsWhileSpeedUp   + TurnsWhileSpeedDown ;
    if (nr_turns<-LOW_SPEED_BORDER)                                           /* Some safety margins */
    {
      WaitValueForShutdown[channel_number]+=2;
      WaitValueForForced[channel_number]  +=5;
    }      
  }
  else
  {
    WaitValueForShutdown[channel_number] = TargetValue[channel_number] - TurnsWhileSpeedDown;
    WaitValueForForced[channel_number]   = TargetValue[channel_number] - TurnsWhileSpeedUp -TurnsWhileSpeedDown;
    if (nr_turns>LOW_SPEED_BORDER)                                            /* Some safety margins */
    {
      WaitValueForShutdown[channel_number]-=2;
      WaitValueForForced[channel_number]  -=5;
    }      

  }    

  do_turn_control[channel_number]=1;                                          /* start turn control */
}

/*! \brief  Turn Task
  *
  * Control Task for reaching an absolute or relative turn position 
  *
  * @param  pvParameters: None
  * @retval None
*/
void TurnTask(void* pvParameters)
{
  long channel = (long) pvParameters;
  while(1)
  {
    if (do_turn_control[channel]==1)
    {
      MotorSetValues[channel].TargetSpeed=MaxSpeed[channel];
      VE_Start(channel);
      
      /* Wait to reach shutdown to forced commutation value */
      while (   (EncoderData[channel].FullTurns!=WaitValueForForced[channel])
             && (do_turn_control[channel]==1))
        vTaskDelay( 1 / portTICK_RATE_MS );                                   /* sleep a while */
        
      MotorSetValues[channel].TargetSpeed=ForcedSpeed[channel];
      
      /* Wait to reach shutdown to shutdown */
      while (   (EncoderData[channel].FullTurns!=WaitValueForShutdown[channel])
             && (do_turn_control[channel]==1))
        vTaskDelay( 1 / portTICK_RATE_MS );                                   /* sleep a while */
        
      /* Limit speed to approach level */
      if (EncoderData[channel].CW==1)
        MotorSetValues[channel].TargetSpeed= APPROACH_RPM;
      else
        MotorSetValues[channel].TargetSpeed=-APPROACH_RPM;
      
      /* Wait for reaching destination */
      while (   (EncoderData[channel].FullTurns!=TargetValue[channel])
             && (do_turn_control[channel]==1))
        vTaskDelay( 1 / portTICK_RATE_MS );                                   /* sleep a while */
      
      VE_Stop(channel);                                                       /* stop motor */
      do_turn_control[channel]            =0;                                 /* stop turn control */
      MotorSetValues[channel].TargetSpeed =0;                                 /* reset target value to 0 - if someone has interrupted the turn control by stopping the motor */
      TargetValue[channel]                = EncoderData[channel].FullTurns;   /* save actual value - needed if starting again a stopped turn position */
    }
    
    vTaskDelay( 100 / portTICK_RATE_MS );                                     /* sleep a while */
  }
}
#endif // USE_TURN_CONTROL
