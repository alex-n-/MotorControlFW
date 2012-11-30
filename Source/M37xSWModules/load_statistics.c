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
#ifdef USE_LOAD_STATISTICS

#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

static unsigned char oldload;
static long idle_counter=0;
static unsigned char system_load;

/* Idle counter for the different compilers */

#if defined ( __CC_ARM   )
	#define MAX_IDLE_COUNTER_NO_LOAD 1470000
#elif (defined (__ICCARM__))
	#define MAX_IDLE_COUNTER_NO_LOAD 2850000
#elif (defined (__GNUC__))
	#define MAX_IDLE_COUNTER_NO_LOAD 2830000
#else
	#error "Please define MAX_IDLE_COUNTER_NO_LOAD."
#endif

/*! \brief  Idle handler
  *
  * Overloading function for FreeRTOS idle handler
  *
  * @param  None
  * @retval None
*/
void vApplicationIdleHook( void )
{
  idle_counter++;                                                               /* increase the idle counter by 1 */
}

/*! \brief  Calculate the system load
  *
  * calculating the system load and taking time inside of the IRQ functions
  * also into account.
  * The norlam system load implementation of FreeRTOS only measures the time
  * spend inside of the different tasks, but not taking into account the time
  * spend inside the IRQ.
  * For a normal system this is good enougth - but here we do most of the
  * calculations inside the IRQ handlers.
  * Therefor we have to use this trick.
  *
  * @param  None
  * @retval system load as percentage
*/
static unsigned char calculate_system_load(void)
{
  unsigned char load = (MAX_IDLE_COUNTER_NO_LOAD-idle_counter)                  /* normal percentage calculation */
                        * 100
                        / MAX_IDLE_COUNTER_NO_LOAD;
  if (idle_counter > MAX_IDLE_COUNTER_NO_LOAD)                                  /* check for error due to preemtion */
    load=oldload;
  oldload=load;                                                                 /* remember last valid value */
  idle_counter=0;
  return load;
}

/*! \brief  Read out the system load
  *
  *
  * @param  None
  * @retval system load as percentage
*/
unsigned char SYSTEMLoad_get(void)
{
  return system_load;
}

/*! \brief  System Load Worker Task
  *
  * @param  pvParameters: None
  * @retval None
*/
void SystemLoadTaks(void* pvParameters)
{
  portTickType xLastWakeTime;
  const portTickType xPeriod = ( 1000 / portTICK_RATE_MS );
  
  xLastWakeTime = xTaskGetTickCount();
  
  while (INIT_Done==0)                                                          /* wailt until HW setup has finished */
    vTaskDelay( 100 / portTICK_RATE_MS );
  
  for(;;)
  {
    system_load = calculate_system_load();                                      /* calculate the system load and remember it for the time it's needed */
    vTaskDelayUntil( &xLastWakeTime, xPeriod );                                 /* wait until 1 second till last calling has completed */
  }
}    

#endif
