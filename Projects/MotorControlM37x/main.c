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

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware library includes. */
#include "spi.h"
#include "compiled_in_motor.h"
#include "motorctrl.h"
#include "board.h"
#include "load_statistics.h"
#include "protocol.h"
#include "stall_detect.h"
#include "debug.h"
#include "dso.h"
#include "turn_control.h"
#include "config_storage.h"

#include "config.h"
#include BOARD_BOARD_HEADER_FILE
#include BOARD_LED_HEADER_FILE
#include TMPM_WDT_HEADER_FILE

#ifdef USE_RGB_LED
#include BOARD_RGB_LED_HEADER_FILE
#endif /* USE_RGB_LED */    

#ifdef BOARD_HITEX_M370
#include "hitex_m370_ui.h"
#endif /* BOARD_HITEX_M370 */

#ifdef BOARD_CAN_HEADER_FILE
#include BOARD_CAN_HEADER_FILE
#endif /* (defined BOARD_M372STK) || (defined BOARD_M374STK) */

static xTaskHandle xSystemInit;                                                 /*!< Handle of the init task - for suicide */

/*! \brief  Application Stack Overflow
  *
  * Generic Function needed by FreeRTOS
  * Will be called in case a Task has exceeded its Stack Size
  *
  * If you come here - please check your stack sizes!!
  *
  * @param  pxTask: Handle of Task
  * @param  pxTask: Name  of Task
  * @retval None
*/
void vApplicationStackOverflowHook(xTaskHandle* pxTask, signed portCHAR* pcTaskName)
{

  /* This function will get called if a task overflows its stack. */
  dprintf("pxTask : %p\n",pxTask);
  dprintf("pcTaskName:%s\n",pcTaskName);

  for(;;);
}

void vApplicationMallocFailedHook(void)
{
  /* We should never go to this function - check your memory settings */
  for(;;);
}

/*! \brief  System Init
  *
  * @param  pvParameters: None
  * @retval None
*/
void system_init( void *pvParameters )
{
  int ret;
  
  SCB->SHCSR = SCB_SHCSR_MEMFAULTENA_Msk                                        /* enable Fault Interrupts */
              |SCB_SHCSR_BUSFAULTENA_Msk
              |SCB_SHCSR_USGFAULTENA_Msk;
    
  BOARD_SetupHW();                                                              /* Setup HW components on the Board (internal/external) */

  /* Clear all settings */
  memset(&MotorParameterValues[0], 0,sizeof(MotorParameterValues)); 
  memset(&PIControl[0],            0,sizeof(PIControl)); 
  memset(&SystemValues[0],         0,sizeof(SystemValues)); 

  /* Make some usefull basic setup - otherwise system will crash */
  SystemValues[0].PWMFrequency     = 12500;
  SystemValues[1].PWMFrequency     = 12500;
  
  /* Read back parameters stored in EEProm */
#ifdef USE_CONFIG_STORAGE
  ret = config_storage_load_config();
  if (ret)
  {
#ifdef USE_INTERNAL_MOTOR_PARAMS
    SetupInternalMotorParams();                                                 /* Use compiled in Parameter setting on error */
#endif /* USE_INTERNAL_MOTOR_PARAMS */
#ifdef USE_RGB_LED
    RGB_LED_SetValue(0);
    RGB_LED_StoreValue();  
#endif /* USE_RGB_LED */ 
  }
  else
  {
#ifdef USE_LED
    LED_SetState(LED_SIGNAL_CONFIG_READ,LED_ON);                                /* Signal successful read-back */
#endif /* USE_LED */
#ifdef USE_RGB_LED
    RGB_LED_SetValue(LED_RGB_BLUE);
    RGB_LED_StoreValue();  
#endif /* USE_RGB_LED */ 
  }
  
#else
  SetupInternalMotorParams();                                                   /* When no EEProm - use compiled in parameters */
#endif
  BOARD_SetupHW2();                                                             /* Setup rest of the HW components on the Board */
  
  /* Kill myself to free stack memory */
  vTaskDelete(xSystemInit);
}

int main(void)
{
  IRQn_Type nr;

  /* INTF_IRQn is the "last" interrupt for all
  supported platforms at the moment. */
  for (nr = (IRQn_Type)0; nr < INTF_IRQn; nr++)
    NVIC_SetPriority(nr, ~0);

  if ( xTaskCreate(system_init,                                                 /* Create an init task - needed as some of the inits use a task delay */
                   (signed char*) "SystemInit",
                   120,
                   NULL,
                   INIT_TASK_PRIORITY,
                   &xSystemInit) != pdPASS )
    dprintf("Can't create System Init Task\n");

#ifdef USE_UI  
  if ( xTaskCreate(UITask,                                                      /* Create User Interface */
                   (signed char*) "UI",
                   UI_TASK_STACK_SIZE,
                   NULL,
                   UI_TASK_PRIORITY,
                   NULL) != pdPASS )
    dprintf("Can't create UI Task\n");
#endif /* USE_UI */
  
#ifdef USE_SERIAL_COMMUNICATION  
  if ( xTaskCreate(ProtocolTask,                                                /* Create serial communication protocol task */
                   (signed char*) "Comm",
                   PROTOCOL_TASK_STACK_SIZE,
                   NULL,
                   PROTOCOL_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create Protocol Task\n");
#endif /* USE_SERIAL_COMMUNICATION */

#ifdef USE_LOAD_STATISTICS
  if ( xTaskCreate(SystemLoadTaks,                                              /* Create System Load Task */
                   (signed char*) "SysLoad",
                   SYSTEM_LOAD_TASK_STACK_SIZE,
                   NULL,
                   SYSTEM_LOAD_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create System Load Task\n");
#endif /* USE_LOAD_STATISTICS */

#ifdef USE_CAN
  if ( xTaskCreate(CanTask,                                                     /* Create CAN Communication Protocol Task */
                   (signed char*) "Can",
                   CAN_TASK_STACK_SIZE,
                   NULL,
                   CAN_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create CAN Task\n");
#endif /* USE_CAN */

#ifdef USE_STALL_DETECT
  if ( xTaskCreate(StallTask,                                                   /* Create Stall Detection Task */
                   (signed char*) "Stall",
                   STALL_TASK_STACK_SIZE,
                   NULL,
                   STALL_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create Stall Task\n");
#endif /* USE_STALL_DETECT */

#ifdef USE_TURN_CONTROL
#ifdef __TMPM_370__
  if ( xTaskCreate(TurnTask,                                                    /* Create Turn Task Channel 0*/
                   (signed char*) "Turn",
                   TURN_TASK_STACK_SIZE,
                   (void*)0,
                   TURN_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create Turn Task\n");
#endif /* USE_TURN_CONTROL */
  if ( xTaskCreate(TurnTask,                                                    /* Create Turn Task Channel 1*/
                   (signed char*) "Turn",
                   TURN_TASK_STACK_SIZE,
                   (void*)1,
                   TURN_TASK_PRIORITY,
                   NULL) != pdPASS)
    dprintf("Can't create Turn Task\n");

#endif /* USE_TURN_CONTROL */
  

  vTaskStartScheduler();                                                        /* Start the scheduler. */

  /*
   * Will only get here if there was insufficient memory to create the idle task.
   * The idle task is created within vTaskStartScheduler().
   */
  for(;;);                                                                      /* Error */
}
