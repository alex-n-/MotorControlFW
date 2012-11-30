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

#ifndef _DSO_H_
#define _DSO_H_

#include <stdint.h>

#define MAX_DSO_VALUE 16                                                        /*!< Maximum number of data from DSO trasmitted at once */

/*! \brief DSO Selection signals.
 *
 *  This enumaration is for selecting the signals that should be logged with the
 *  DSO functionality of the system. Not more than MAX_DSO_VALUE bits should be
 *  set at once.
 */
typedef enum
{
  Va              = (1<< 0),                                                    /*!< a-phase voltage */
  Vb              = (1<< 1),                                                    /*!< b-phase voltage */
  Vc              = (1<< 2),                                                    /*!< c-phase voltage */
  Valpha          = (1<< 3),                                                    /*!< alpha-axis voltage */
  Vbeta           = (1<< 4),                                                    /*!< beta-axis voltage */
  Id              = (1<< 5),                                                    /*!< d-axis current */
  Id_Ref          = (1<< 6),                                                    /*!< d-axis reference current */
  Id_kp           = (1<< 7),                                                    /*!< Proportional coefficient */
  Id_ki           = (1<< 8),                                                    /*!< Proportional coefficient */
  Iq              = (1<< 9),                                                    /*!< q-axis current */
  Iq_Ref          = (1<<10),                                                    /*!< q-axis reference current */
  Iq_kp           = (1<<11),                                                    /*!< Proportional coefficient */
  Iq_ki           = (1<<12),                                                    /*!< Proportional coefficient */
  Vd              = (1<<13),                                                    /*!< d-axis voltage */
  Vq              = (1<<14),                                                    /*!< q-axis voltage */
  Vdi             = (1<<15),                                                    /*!< d-axis integral term */
  Vqi             = (1<<16),                                                    /*!< q-axis integral term */
  Theta           = (1<<17),                                                    /*!< Phase 0 */
  Omega           = (1<<18),                                                    /*!< Rotation speed */
  SIN_Theta       = (1<<19),                                                    /*!< Sine Value at 0 */
  COS_Theta       = (1<<20),                                                    /*!< Cosine Value at 0 */
  Sector          = (1<<21),                                                    /*!< Sector information */
  VDC             = (1<<22),                                                    /*!< Motor supply voltage */
  Ia              = (1<<23),                                                    /*!< a-phase current */
  Ib              = (1<<24),                                                    /*!< b-phase current */  
  Ic              = (1<<25),                                                    /*!< c-phase current */
  Ialpha          = (1<<26),                                                    /*!< alpha-axis current */
  Ibeta           = (1<<27),                                                    /*!< beat-axis current */
  SW_Id_Ref       = (1<<28),                                                    /*!< d-axis reference current (inside variable) */
  SW_Iq_Ref       = (1<<29),                                                    /*!< q-axis reference current (inside variable) */
  SW_Id           = (1<<30),                                                    /*!< d-axis current (inside variable) */
#if (defined(__KEIL__))
#pragma diag_suppress 61
#endif
  SW_Iq           = (1<<31),                                                    /*!< q-axis current (inside variable) */
#if (defined(__KEIL__))
#pragma diag_default 61
#endif
} DSO_Selection;

/*! \brief DSO Trigger selection.
 *
 *  This enumaration is for selecting the type of trigger that can be used for
 *  DSO logging possibility.
 *  The following options are available: Rising- / Falling-Edge
 *                                       Center- / Left-Trigger
 */
typedef enum
{
  TRIGGER_RISING  = (1<<0),                                                     /*!< Trigger on rising edge - otherwise if bit is not set: Trigger on Falling edge */
  TRIGGER_CENTER  = (1<<1),                                                     /*!< Do center trigger - otherwise if bit is not set: Do left trigger */
} TriggerMode;

typedef enum
{
  PMD_CALLING,                                                                  /*!< Log called from PMD Interrupt Routine */
  VE_CALLING,                                                                   /*!< Log called from VE Interrupt Routine */
} Expected_Log_Caller;

/* check if firmware is compiled */
#ifdef FW_VERSION_MAJOR
#include "ve.h"

int16_t* DSO_Fill_up_data(int16_t* pos,unsigned long selected_values,unsigned char channel_number);
int      DSO_GetLogData(int16_t* pos);
uint16_t DSO_Configure_Continuous_Mode(uint8_t  channel_number,
                                       uint32_t selected_signals,
                                       uint32_t trigger_on_value,
                                       uint8_t  trigger_mode,
                                       uint16_t level,
                                       uint8_t  spread_selection);
uint16_t DSO_Configure_Single_Shot_Mode(uint8_t  channel_number,
                                        uint32_t selected_signals);
void     DSO_Log (Expected_Log_Caller caller , TEE_VE_TypeDef* pVEx );

#endif

#endif
