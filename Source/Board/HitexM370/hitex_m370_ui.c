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

#include "config.h"
#if (defined BOARD_HITEX_M370 && defined USE_UI)

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

#include BOARD_LCD_HEADER_FILE
#include BOARD_KEYPAD_HEADER_FILE

#include "ve.h"
#include "motorctrl.h"
#include "action.h"
#include "board.h"

#define UI_NAME                     "TMPM370 UI - FW:"

#define UI_NAME_LINE                0
#define MOTOR_NAME_LINE             1
#define SPEED_LINE                  2
#define ADDITIONAL_INFORMATION_LINE 3

#define UI_NAME_START               1
#define MOTOR_NAME_START            2

#define FW_VERSION_START            16
#define NR_DIGITS_FW_MAJOR          1
#define NR_DIGITS_FW_MINOR          2

#define DIRECTION_ACTUAL            2
#define POSITION_ACTUAL             (DIRECTION_ACTUAL+1)
#define DIRECTION_SET               12
#define POSITION_SET                (DIRECTION_SET+1)
#define SPEED_SEPERATOR_POS         9

#define NR_DIGITS_SPEED             4
#define NR_DIGITS_CURRENT           4
#define NR_DIGITS_TORQUE            3

#define CURSOR_POS_1                (DIRECTION_SET+NR_DIGITS_SPEED)
#define POSITION_CURRENT            1
#define POSITION_TORQUE             (POSITION_CURRENT+NR_DIGITS_CURRENT+1)
#define POSITION_STATE              12

static  uint8_t   motor_nr=1;
static  uint8_t   cursor_pos=CURSOR_POS_1;
static  uint32_t  oldstate  =0xff,
                  oldset    =0xff,
                  oldstatus =0xff,
                  oldtorque =0xff,
                  oldcurrent=0xff,
                  count_difference;
static  uint8_t   oldmotorid[20];
static  uint32_t  new_rot;

void UITask(void* pvParameters)
{
  (void) pvParameters;

  while (INIT_Done==0)
    vTaskDelay( 100 / portTICK_RATE_MS );
  
  /* Wait until init task has finished */
  vTaskDelay( 100 / portTICK_RATE_MS );
 
  /* Switch off Cursor to prevent anoying blink in the middle of display */
  LCD_DisableCursorBlink();
  
  /* Print UI Name */
  LCD_DisplayText(UI_NAME_LINE,
                  UI_NAME_START,
                  UI_NAME);
  
  /* Print FW-Version */
  LCD_DisplayValue(UI_NAME_LINE,
                   FW_VERSION_START,
                   LCD_VALUE_DECIMAL,
                   NR_DIGITS_FW_MAJOR,
                   FW_VERSION_MAJOR);

  LCD_DisplayText(UI_NAME_LINE,
                  FW_VERSION_START+1,
                  ".");
  
  LCD_DisplayValue(UI_NAME_LINE,
                   FW_VERSION_START+2,
                   LCD_VALUE_DECIMAL,
                   NR_DIGITS_FW_MINOR,
                   FW_VERSION_MINOR);
  
  /* Print speed seperator */ 
  LCD_DisplayText(SPEED_LINE,
                  SPEED_SEPERATOR_POS,
                  "/");
  for(;;)
  {
    /* 100 ms delay between testing keypads */
    vTaskDelay( 100 / portTICK_RATE_MS );
    
    /* Check if Motor Name has changed - if so, display new */
    if (strcmp((char const*)MotorParameterValues[motor_nr].MotorId,(char const*)oldmotorid) != 0)
    {
      LCD_DisplayText(MOTOR_NAME_LINE,
                      0,
                      "                    ");  /* Delete old content */
     
      LCD_DisplayText(MOTOR_NAME_LINE,
                      MOTOR_NAME_START,
                      (char *)MotorParameterValues[motor_nr].MotorId);
      
      memcpy(oldmotorid,MotorParameterValues[motor_nr].MotorId,sizeof(MotorParameterValues[motor_nr].MotorId));
    }

    /* Check if Rotation speed has changed - if so, display new */
    if (MotorStateValues[motor_nr].ActualSpeed != oldstate)
    {
      LCD_DisplayValue(SPEED_LINE,
                       POSITION_ACTUAL ,
                       LCD_VALUE_DECIMAL,
                       NR_DIGITS_SPEED,
                       abs(MotorStateValues[motor_nr].ActualSpeed));
      
      if (MotorStateValues[motor_nr].ActualSpeed<0)
        LCD_DisplayText(SPEED_LINE,
                        DIRECTION_ACTUAL,
                        "-");
      else
        LCD_DisplayText(SPEED_LINE,
                        DIRECTION_ACTUAL,
                        "+");

      oldstate=MotorStateValues[motor_nr].ActualSpeed;
    }

    /* Check if Rotation Speed Set value has changed - if so, display new */
    if (MotorSetValues[motor_nr].TargetSpeed != oldset)
    {
      LCD_DisplayValue(SPEED_LINE,
                       POSITION_SET,
                       LCD_VALUE_DECIMAL,
                       NR_DIGITS_SPEED,
                       abs(MotorSetValues[motor_nr].TargetSpeed));
      
      if (MotorSetValues[motor_nr].TargetSpeed<0)
        LCD_DisplayText(SPEED_LINE,
                        DIRECTION_SET,
                        "-");
      else
        LCD_DisplayText(SPEED_LINE,
                        DIRECTION_SET,
                        "+");

      oldset=MotorSetValues[motor_nr].TargetSpeed;
    }

    /* Check if Current has changed - if so, display new */
    if (MotorStateValues[motor_nr].Current != oldcurrent)
    {
      LCD_DisplayValue(ADDITIONAL_INFORMATION_LINE,
                       POSITION_CURRENT,
                       LCD_VALUE_DECIMAL,
                       NR_DIGITS_CURRENT,
                       MotorStateValues[motor_nr].Current);
      
      oldcurrent=MotorStateValues[motor_nr].Current;
    }

    /* Check if Torque has changed - if so, display new */
    if (MotorStateValues[motor_nr].Torque != oldtorque)
    {
      LCD_DisplayValue(ADDITIONAL_INFORMATION_LINE,
                       POSITION_TORQUE,
                       LCD_VALUE_DECIMAL,
                       NR_DIGITS_TORQUE,
                       MotorStateValues[motor_nr].Torque);
      
      oldtorque=MotorStateValues[motor_nr].Torque;
    }

    /* Check if Motor State has changed - if so, display new */
    if (MotorStateValues[motor_nr].TargetSpeed != oldstatus)
    {
      if (MotorStateValues[motor_nr].TargetSpeed != 0)
        LCD_DisplayText(ADDITIONAL_INFORMATION_LINE,
                        POSITION_STATE,
                        "Start");
      else
        LCD_DisplayText(ADDITIONAL_INFORMATION_LINE,
                        POSITION_STATE,
                        "Stop ");
        
      oldstatus = MotorStateValues[motor_nr].TargetSpeed;
    }
        
    
    switch (cursor_pos)
    {
    case CURSOR_POS_1:
      count_difference=1;
      break;
    case CURSOR_POS_1-1:
      count_difference=10;
      break;
    case CURSOR_POS_1-2:
      count_difference=100;
      break;
    case CURSOR_POS_1-3:
      count_difference=1000;
      break;
    default:
      break;
    }
    
    switch(KEYPAD_GetState())
    {
    case KEYPAD_NO_0:                                                 /* Move cursor one left */
      cursor_pos--;
      break;
    case KEYPAD_NO_3:                                                 /* Move cursor one right */
      cursor_pos++;
      break;
    case KEYPAD_NO_1:                                                 /* Increase rotation speed set value */
      new_rot=MotorSetValues[motor_nr].TargetSpeed;
      new_rot +=count_difference;
      /* Limit set value to motor limits */
      if(new_rot > (  MotorParameterValues[motor_nr].HzLimit
                    * SECONDS_PER_MINUTE
                    / MotorParameterValues[motor_nr].PolePairs ))
        new_rot =  (  MotorParameterValues[motor_nr].HzLimit
                    * SECONDS_PER_MINUTE 
                    / MotorParameterValues[motor_nr].PolePairs );
      
      MotorSetValues[motor_nr].TargetSpeed=new_rot;
      break;
    case KEYPAD_NO_4:                                                 /* Decrease rotation speed set value */  
      new_rot=MotorSetValues[motor_nr].TargetSpeed;
      new_rot -=count_difference;
      /* Limit set value to motor limits */
      if(new_rot < -( MotorParameterValues[motor_nr].HzLimit
                    * SECONDS_PER_MINUTE
                    / MotorParameterValues[motor_nr].PolePairs ))
        new_rot =  -( MotorParameterValues[motor_nr].HzLimit
                    * SECONDS_PER_MINUTE
                    / MotorParameterValues[motor_nr].PolePairs );
      
      MotorSetValues[motor_nr].TargetSpeed=new_rot;
      break;
    case KEYPAD_NO_2:                                                 /* Change motor number */
      motor_nr++;
      if (motor_nr>MAX_CHANNEL-1)
        motor_nr=0;
      break;
    case KEYPAD_NO_5:                                                 /* Toggle motor state */
      /* Send start/stop command internal to action.c */
      if (MotorStateValues[motor_nr].ActualSpeed == 0)
        VE_Start(motor_nr);
      else
        VE_Stop(motor_nr);
      break;
    default:
      break;
    }

    /* Limit set cursor position */
    if (cursor_pos>CURSOR_POS_1)
      cursor_pos=CURSOR_POS_1;

    if (cursor_pos<CURSOR_POS_1-NR_DIGITS_SPEED+1)
      cursor_pos=CURSOR_POS_1-NR_DIGITS_SPEED+1;
    
    LCD_SetCursorPosition(SPEED_LINE,cursor_pos);
    
    /* Enable cursor blink to have an idea which digit to edit */
    LCD_EnableCursorBlink();
  }
}

#endif
