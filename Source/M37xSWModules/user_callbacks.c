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

#ifdef USE_USER_CALLBACKS

#include "ve.h"
#include "user_callbacks.h"


/*! \brief  User callback hook AfterInputProcessing
  *
  * This function will be called after Input Processing has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterInputProcessing(void)
{
}

/*! \brief  User callback hook AfterInputPhaseConversion
  *
  * This function will be called after Input Phase Conversion has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterInputPhaseConversion(void)
{
}


/*! \brief  User callback hook AfterInputCoordinateAxisConversion
  *
  * This function will be called after Input Coordinate Axis Conversion has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterInputCoordinateAxisConversion(void)
{
}

/*! \brief  User callback hook AfterCurrentControl
  *
  * This function will be called after Current Control has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterCurrentControl(void)
{
}

/*! \brief  User callback hook AfterSinCosComputation
  *
  * This function will be called after Sin/Cos Computation has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterSinCosComputation(void)
{
}

/*! \brief  User callback hook AfterOutputCoordinateAxisConversion
  *
  * This function will be called after Output Coordinate Axis Conversion has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterOutputCoordinateAxisConversion(void)
{
}

/*! \brief  User callback hook AfterOutputPhaseConversion
  *
  * This function will be called after Output Phase Conversion has finished
  * when in Schedule 0
  *
  * @param  None
  * @retval None
*/
void AfterOutputPhaseConversion(void)
{
}

/*! \brief  User callback hook None
  *
  * This function will not be called called at all - it's just a placeholder
  *
  * @param  None
  * @retval None
*/
void None(void)
{
  /* Do not place any code here */
};


/*! \brief  User callback Table
  *
  * With this table for accessing the different Callback Hooks while in Schedule 0
  * of the Vector Engine
  * First element is the function that will be called.
  * Second Element is the next Schedule to perform (maybe switch to schedule 1 to jump over the rest of the functions)
  * Third element is the next Task of the Vector Engine to perform.
  *
  * By changing the Schedule from 0 to 1 for a hook it is possible to let the Vector Engine run in Schdule 1 till finishing
  * By changing the Next Task Value it's possible to jump over a Vector Engine Task (mabe because all
  * needed calcuulation have been done in software already)
  * 
  * Do not change the first 2 lines and not the last one!!
  * At the moment it's not possible to perform any user actions after:
  * - Output Phase Conversion
  * - Output contro
  * - Trigger generation
  *
  * But normally this should also not be needed
  *
  *
  * @param  None
  * @retval None
*/
const UserCallback CallbackTable[9] =
{  // FUNCTION                         SCHEDULE                     NEXT_TASK
  {None                               , 0                           , 0},
  {None                               , 0                           , 0},
  {AfterInputProcessing               , VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_INPUT_PHASE_CONVERSION},
  {AfterInputPhaseConversion          , VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_INPUT_COORDINATE_AXIS_CONVERSION},
  {AfterInputCoordinateAxisConversion , VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_CURRENT_CONTROL},
  {AfterCurrentControl                , VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_SIN_COS_COMPUTATION},
  {AfterSinCosComputation             , VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_OUTPUT_COORDINATE_AXIS_CONVERSION},
  {AfterOutputCoordinateAxisConversion, VE_ACTSCH_SCHEDULE_0        , VE_TASKAPP_OUTPUT_PHASE_CONVERSION},
  {AfterOutputPhaseConversion         , VE_ACTSCH_SCHEDULE_9        , VE_TASKAPP_OUTPUT_CONTROL},
};

#endif /* USE_USER_CALLBACKS */
