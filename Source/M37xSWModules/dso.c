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

#include <stdio.h>
#include <string.h>

#include "config.h"
#ifdef USE_DSO

#include BOARD_BOARD_HEADER_FILE

#include "debug.h"
#include "dso.h"
#include "motorctrl.h"
#include "ve.h"

/*********************************************************************
************************ Local Data **********************************
*********************************************************************/	
static int16_t              DSODataArray[BOARD_DSO_SIZE];
static Expected_Log_Caller  expectedCaller        = PMD_CALLING;      /* from whom the next irq should come */
static unsigned char        trigger_fired         = 0;                /* flag for trigger has fired */
static unsigned char        channel               = 1;                /* channel to use for dso logging */
static unsigned long        selected_values       = 0;                /* selected signals for dso logging */
static unsigned int         storage_size          = 0;                /* size of dso logging storage area */
static unsigned char        do_logging            = 0;                /* flag for logging is active */
static unsigned int         values_after_trigger  = 0;                /* number of values to log after trigger has happened */
static signed   char        trigger_offset        = 0;                /* ofset inside logged data for the value to trigger on */
static unsigned int         nr_log_data           = 0;                /* Number of Data beeing captured */
static signed char          trigger_edge          = 0;                /* flag for egde trigger - otherwise center trigger */
static unsigned char        transmits             = 0;                /* number of transmit elements to host */
static int16_t              trigger_level         = 0;                /* level of trigger */
static int16_t*             actual_pos            = NULL;             /* actual pointer inside of logged data */
static int16_t*             storage_position      = NULL;             /* start of log data area */
static int16_t*             wrap_position         = NULL;             /* end of log data area (might not be at the real end - depending logged data fit to the total end */
static unsigned long        log_times             = 0;                /* numer of times the logging has been done (can not compare first log with nothing) */
static int16_t              spread                = 0;                /* actual spread state */
static unsigned char        spread_factor         = 0;                /* factor of which the logging is spread */

/*********************************************************************
************************** Macros ************************************
*********************************************************************/	
#define CHECK_WRAP(x)             \
        if ( x >= wrap_position ) \
        x = storage_position;

#define CHECK_REWRAP(x)             \
        if ( x <= storage_position )\
        x = wrap_position;

#define LOG_IF_SELECTED(reg,check,shift)        \
        if ((selected_values & (unsigned long)check)) \
        {                                       \
          *pos = reg>>shift;                    \
          pos++;                                \
        }
  
/*! \brief  Get Number of Bits in a long
  *
  * Count the number of set bits in a long type
  *
  * @param  selected_values:  value to test
  * @retval Number of set bits in long
*/
static unsigned char get_nr_of_bits_set(unsigned long selected_values)
{
  unsigned int number; 

  for (number = 0; selected_values; selected_values >>= 1)
    number += selected_values & 1;
 
  return number;
}  

/*! \brief  Get trigger offset
  *
  * Get the number of bits set below the one set for the trigger value
  *
  * @param  selected_values:  filename
  * @retval Number of set bits in long
*/
static signed char get_trigger_offset(unsigned long trigger_value)
{
  unsigned char offset_in_field=0;
  unsigned char trigger_offset = 0;
  unsigned long check_value=selected_values;
  unsigned char i;

  for (; trigger_value; trigger_value >>= 1)
    trigger_offset++;

  for (i=0; i<trigger_offset-1; i++)
  {
    offset_in_field += check_value & 1;
    check_value     >>= 1;
  }

  return offset_in_field;
}  

/*! \brief  Fill up Data for PMD
  *
  * This values can be logged during the PMD IRQ
  *
  * @param  pos: pointer to position in array
  * @retval new pointer to position in array
*/
static int16_t* DSO_Fill_up_dataPMD(int16_t* pos)
{

  LOG_IF_SELECTED(TEE_VEC->TMPREG0,             Va,         17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG1,             Vb,         17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG2,             Vc,         17);
  LOG_IF_SELECTED(TEE_VEC->TMPREG3,             Valpha,     16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG4,             Vbeta,      16);
 
  return pos;
}

/*! \brief  Fill up Data for VE
  *
  * This values can be logged during the VE IRQ
  *
  * @param  pos: pointer to position in array
  * @retval new pointer to position in array
*/
static int16_t* DSO_Fill_up_dataVE(int16_t* pos, TEE_VE_TypeDef* pVEx)
{
  LOG_IF_SELECTED(pVEx->ID,                     Id,         16);
  LOG_IF_SELECTED(pVEx->IDREF,                  Id_Ref,      0);
  LOG_IF_SELECTED(pVEx->CIDKP,                  Id_kp,       0);
  LOG_IF_SELECTED(pVEx->CIDKI,                  Id_ki,       0);
  LOG_IF_SELECTED(pVEx->IQ,                     Iq,         16);
  LOG_IF_SELECTED(pVEx->IQREF,                  Iq_Ref,      0);
  LOG_IF_SELECTED(pVEx->CIQKP,                  Iq_kp,       0);
  LOG_IF_SELECTED(pVEx->CIQKI,                  Iq_ki,       0);
  LOG_IF_SELECTED(pVEx->VD,                     Vd,         16);
  LOG_IF_SELECTED(pVEx->VQ,                     Vq,         16);
  LOG_IF_SELECTED(pVEx->VDIH,                   Vdi,        16);
  LOG_IF_SELECTED(pVEx->VQIH,                   Vqi,        16);
  LOG_IF_SELECTED(pVEx->THETA,                  Theta,       1);
  LOG_IF_SELECTED(VE_Omega[channel].part.reg,   Omega,       0);
  LOG_IF_SELECTED(pVEx->SIN,                    SIN_Theta,   0);
  LOG_IF_SELECTED(pVEx->COS,                    COS_Theta,   0);
  LOG_IF_SELECTED(pVEx->SECTOR,                 Sector,      0);
  LOG_IF_SELECTED(pVEx->VDC,                    VDC,         0);
  LOG_IF_SELECTED(TEE_VEC->TMPREG0,             Ia,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG1,             Ib,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG2,             Ic,         16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG3,             Ialpha,     16);
  LOG_IF_SELECTED(TEE_VEC->TMPREG4,             Ibeta,      16);
  LOG_IF_SELECTED(VE_Id_reference[channel],     SW_Id_Ref,   0);
  LOG_IF_SELECTED(VE_Iq_reference[channel],     SW_Iq_Ref,   0);
  LOG_IF_SELECTED(VE_Id[channel],               SW_Id,       0);
  LOG_IF_SELECTED(VE_Iq[channel],               SW_Iq,       0);

  return pos;
}

/*! \brief  Configure 'normal' logging mode
  *
  * Store selected signals to storage area configuration
  *
  * @param  channel_number:   channel to be logged
  * @param  selected_signals: bitmask of signals to be logged
  * @param  trigger_on_value: bitmask of trigger signal (only 1 bit set)
  * @param  trigger_mode:     center/left trigger - rising/falling edge
  * @param  level:            trigger value
  * @param  spread_selection: spread factor (0 log on every IRQ)
  * @retval number of transfer elements after logging has finished
*/
uint16_t DSO_Configure_Continuous_Mode(uint8_t  channel_number,
                                       uint32_t selected_signals,
                                       uint32_t trigger_on_value,
                                       uint8_t  trigger_mode,
                                       uint16_t level,
                                       uint8_t  spread_selection)
{
  nr_log_data = get_nr_of_bits_set(selected_signals);                 /* get the number of different signals to be logged */
  
  if (nr_log_data>MAX_DSO_VALUE)                                      /* check for overflow */
    return 0;

  do_logging    = 0;                                                  /* prevent logging during configuration */
  
  memset(&DSODataArray,0,sizeof(DSODataArray));                       /* clean up buffer */

  log_times       = 0;
  expectedCaller  = PMD_CALLING;                                      /* PMD IRQ comes first */
  
  storage_size    = sizeof(DSODataArray)/sizeof(DSODataArray[0]);     /* number of elements that could be logged */
  storage_position= &DSODataArray[0];                                 /* start position of storage area */
  actual_pos      = storage_position;                                 /* actual position is beginning of area */
  channel         = channel_number;                                   /* remember the channel */
  selected_values = selected_signals;                                 /* remember the signals */
  wrap_position   = (int16_t*)(( unsigned long)storage_position       /* position where to wrap (if data doesn't fit to the last byte */
                     + nr_log_data * (storage_size/nr_log_data)
                     * sizeof(uint16_t));

  spread_factor   = spread_selection;                                 /* remember spread factor */
  spread          = 0;
  trigger_level   = level;                                            /* remember trigger level */
  transmits       = ((unsigned long)wrap_position                     /* calculate number of transmit packets for GUI */
                   - (unsigned long)storage_position
                   + MAX_DSO_VALUE*sizeof(uint16_t) - 1 )
                   / MAX_DSO_VALUE / sizeof(uint16_t) -1;
  
  
  if ((trigger_mode & TRIGGER_CENTER) == TRIGGER_CENTER)              /* calculate values to log after trigger has fired */
    values_after_trigger = storage_size/nr_log_data/2;
  else
    values_after_trigger = storage_size/nr_log_data;

  if ((trigger_mode & TRIGGER_RISING) == TRIGGER_RISING)              /* remember trigger edge */
    trigger_edge = TRIGGER_RISING;
  else
    trigger_edge = ~TRIGGER_RISING;
  
  if (!trigger_on_value)
  {
      trigger_fired = 1;                                              /* when no trigger selected just start directly */
      values_after_trigger = storage_size/nr_log_data;                /* fill full buffer with data */
  }
  else
  {
      trigger_fired = 0;                                              /* when trigger signal selected - wait ... */
      trigger_offset  = get_trigger_offset(trigger_on_value);         /* get value (nuber of logged data) to trigger on */
  }
  
  dprintf("trigger_offset:%d\n",trigger_offset);
  dprintf("nr_log_data:%d\n",nr_log_data);
  dprintf("values_after_trigger:%d\n",values_after_trigger);
  
  do_logging = 1;                                                     /* enable logging (arm trigger) */
  
  return transmits;
}

/*! \brief  Single Shot Log
  *
  * Get one set of values for the selected signals
  *
  * @param  channel_number:   channel to be logged
  * @param  selected_signals: bitmask of signals to be logged
  * @retval number of transfer elements after logging has finished
*/
uint16_t DSO_Configure_Single_Shot_Mode(uint8_t  channel_number,
                                        uint32_t selected_signals)
{
  unsigned char nr_log_data;
  
  nr_log_data = get_nr_of_bits_set(selected_signals);                 /* get the number of different signals to be logged */
 
  if (nr_log_data>MAX_DSO_VALUE)                                      /* check for overflow */
    return 0;
  
  if (do_logging==1)                                                  /* checkif normal logging is active and prevent singleshot in that case */
    return 0;

  storage_size        = sizeof(DSODataArray)/sizeof(DSODataArray[0]); /* number of elements that could be logged */ 
  storage_position    = &DSODataArray[0];                             /* start position of storage area */
  actual_pos          = storage_position;                             /* actual position is beginning of area */

  channel             = channel_number;                               /* remember channel */
  selected_values     = selected_signals;                             /* remember selected signals */
  expectedCaller      = PMD_CALLING;                                  /* PMD IRQ comes first */
  log_times           = 0;                                            /* first set of data */
  values_after_trigger= 1;                                            /* one set of values */
  trigger_fired       = 1;                                            /* set trigger fired */
  transmits           = 1;                                            /* one set of data to transmit */
  do_logging          = 1;                                            /* start logging directly */
  
  return transmits;
}

/*! \brief  Get log data
  *
  * Prepare log data for transfer to GUI
  *
  * @param  pos: position inside log data
  * @retval Status for transmission
*/
int DSO_GetLogData(int16_t* pos)
{
  int i;
  
  if (do_logging==1)                                                  /* logging is active ? */
    return -2;

  if (transmits==0)                                                   /* nothing to transmit anymore */
    return -1;
  
  for (i=0;i<MAX_DSO_VALUE;i++)                                       /* prepare one set of data */
  {
    *pos++=*actual_pos++;
    CHECK_WRAP(actual_pos);
  }
  
  transmits--;                                                        /* one set less to tranfer */
  return 0;                                                           /* data is valid */
}

/*! \brief  Logging functionality
  *
  * Do the real logging - called via PMD IRQ or VE IRQ 
  *
  * @param  caller: expected caller
  * @param  pVEx:   pointer to VE registers
  * @retval None
*/
void DSO_Log (Expected_Log_Caller caller , TEE_VE_TypeDef* pVEx )
{
  int16_t*          oldval_pos = NULL;
  int16_t*          newval_pos = NULL;
  TEE_VE_TypeDef*   check_pVEx = NULL;

  if (do_logging == 0)
    return;
  
  switch (channel)
  {
  case 0:
    check_pVEx  = TEE_VE0;
    break;
  case 1:
    check_pVEx  = TEE_VE1;
    break;
  default:
    assert_param(0);
    break;
  }
    
  if (   (caller        == expectedCaller)                            /* Interrupr from IRQ that's next to log? */
      && (pVEx          == check_pVEx)                                /* correct channel? */
      && (trigger_fired == 0) )                                       /* trigger has fired */
  {
    if (caller==PMD_CALLING)
    {
      if (spread == 0)                                                /* when spread factor fefined - only log every spread-number value */
        actual_pos=DSO_Fill_up_dataPMD(actual_pos);                     /* put values available during PMD IRQ to the log data */
      expectedCaller=VE_CALLING;                                      /* next one is Vector Engine */
    }
    else
    {        
      if (spread == 0)                                                /* when spread factor fefined - only log every spread-number value */
        actual_pos=DSO_Fill_up_dataVE(actual_pos,pVEx);                 /* put values available during VE IRQ to the log data */
      expectedCaller=PMD_CALLING;                                     /* next one is PMD again */

      CHECK_WRAP(actual_pos);                                         /* check for passing wrap around position */
      log_times++;

      /* Determine position of trigger value (older) - keep wrap around in mind */
      newval_pos = actual_pos - nr_log_data;
      CHECK_REWRAP(newval_pos);

      oldval_pos = newval_pos - nr_log_data;
      CHECK_REWRAP(oldval_pos);
      
      if (log_times>2)                                                /* Can only compare if more than 2 values are available */
      {        
        if ( (   (oldval_pos[trigger_offset] > newval_pos[trigger_offset])
              && (newval_pos[trigger_offset] <= trigger_level)
              && (oldval_pos[trigger_offset] >= trigger_level)
              && (trigger_edge == ~TRIGGER_RISING) )                  /* Falling edge and value smaller or equal trigger level */
        || (   (oldval_pos[trigger_offset] < newval_pos[trigger_offset])
              && (newval_pos[trigger_offset] >= trigger_level)
              && (oldval_pos[trigger_offset] <= trigger_level)
              && (trigger_edge ==  TRIGGER_RISING) ) )                /* Rising  edge and value greater or equal trigger level */
        {
          trigger_fired=1;
//          dprintf("old 0x%04x, new 0x%04x, level 0x%04x, trigger_edge %d\n", oldval_pos[trigger_offset], newval_pos[trigger_offset], trigger_level, trigger_edge);
          return;
        }
      }
    }
  }
  
  if (    (caller       == expectedCaller)                            /* Interrupr from IRQ that's next to log? */
       && (pVEx         ==check_pVEx)                                 /* correct channel? */
       && (trigger_fired== 1)                                         /* trigger has fired */
       && (spread       == 0) )                                       /* when spread factor fefined - only log every spread-number value */
  {
    if (values_after_trigger > 0)                                     /* still something to log? */
    {
      if (caller==PMD_CALLING)
      {
        actual_pos=DSO_Fill_up_dataPMD(actual_pos);                   /* put values available during PMD IRQ to the log data */
        expectedCaller=VE_CALLING;                                    /* next one is Vector Engine */
      }
      else
      {        
        actual_pos=DSO_Fill_up_dataVE(actual_pos,pVEx);               /* put values available during VE IRQ to the log data */
        CHECK_WRAP(actual_pos);                                       /* check wrap */

        expectedCaller=PMD_CALLING;                                   /* PMD is next again */
        values_after_trigger--;                                       /* one set of values less to log */
      }
    }
    else
    {
      do_logging=0;                                                   /* Logging has finished */
      trigger_fired=0;                                                /* clear trigger fired */
      dprintf("Logging done\n");
    }
  }

  if (    (caller       == VE_CALLING)
       && (pVEx         == check_pVEx))
  {
     spread--;                                                        
     if (spread<0)
       spread=spread_factor;                                          /* cause not logging every set of values */
  }
  
  }    
#endif
