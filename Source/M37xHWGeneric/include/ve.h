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

#ifndef _VE_DEFINE_H_
#define _VE_DEFINE_H_

#include <stdint.h>
#include "adc.h"

#include "config.h"
#include TMPM_HEADER_FILE
#include TMPM_TIMER_HEADER_FILE

#define VE_CHANNEL0_TMRB                 TSB_TB3                                /* Timer for VE Loop */
#define VE_CHANNEL0_TMBR_IRQ             INTTB30_IRQn
#define VE_CHANNEL0_TMBR_IRQ_HANDLER     INTTB30_IRQHandler

#ifndef __TMPM_375__
#define VE_CHANNEL1_TMRB                 TSB_TB1                                /* Timer for VE Loop */
#define VE_CHANNEL1_TMBR_IRQ             INTTB10_IRQn
#define VE_CHANNEL1_TMBR_IRQ_HANDLER     INTTB10_IRQHandler
#else
#define VE_CHANNEL1_TMRB                 TSB_TB0                                /* Timer for VE Loop */
#define VE_CHANNEL1_TMBR_IRQ             INTTB00_IRQn
#define VE_CHANNEL1_TMBR_IRQ_HANDLER     INTTB00_IRQHandler
#endif /* __TMPM_375__ */

#define PAI   31416                                                             /* Pi     * 10000 */
#define PAI2  2 * PAI                                                           /* Pi * 2 * 10000 */

#define SECONDS_PER_MINUTE                60                                    /* One minute has 60 seconds */                                    

#define FIXPOINT_12 0x1000                                                      /* fix point Q12 4096 */
#define FIXPOINT_15 0x8000                                                      /* fix point Q15 32768 */
#define FIXPOINT_16 0x10000                                                     /* fix point Q16 65536 */
#define FIXPOINT_31 0x80000000                                                  /* fix point Q31 */
#define ROUND_BIT15 0x00008000                                                  /* Rounding off */

/* fix point12 sine values */
#define cSine1                            (int16_t) (FIXPOINT_12 * 3.140625)
#define cSine2                            (int16_t) (FIXPOINT_12 * 0.02026367)
#define cSine3                            (int16_t) (FIXPOINT_12 * -5.325196)
#define cSine4                            (int16_t) (FIXPOINT_12 * 0.5446778)
#define cSine5                            (int16_t) (FIXPOINT_12 * 1.800293)
#define VE_DIV_Q15_SQRT3                  18919                                 /* 1 / ROOT3 * 32768 */

#define ELE_DEG(x)                        (FIXPOINT_16 * (x) / 360)

#define VE_ENABLE                         (1 << 0)                              /* Enable VE */
#define VE_DISABLE                        (0 << 0)                              /* Enable VE */
#define VE_IDLE_ACTIVE                    (1 << 1)                              /* Controls whether or not the clock is supplied to the Vector Engine in IDLE mode */

#define VE_MODE_OFF                       0x00                                  /* Flag VE channel off */
#define VE_OUTPUTENABLE	                  0x04                                  /* Flag VE output enable */
#define VE_EMGRETURN                      0x0C                                  /* Flag VE return from EMG */
#define VE_ZEROCURRENTEN                  0x02                                  /* Flag VE Zero Current detection enable */
#define VE_PHASEINTERPOL                  0x01                                  /* Flag VE phase interpolation enable */


#define VE_TRIGGER_MODE_CHANNEL_0_SHIFT   0
#define VE_TRIGGER_MODE_CHANNEL_1_SHIFT   2
#define VE_TRIGGER_MODE_IGN_BOTH          0x00                                  /* Ignore both INTA0 (unit A) and INTA1 (unit B) */
#define VE_TRIGGER_MODE_START_x0          0x01                                  /* Start by INTx0 */
#define VE_TRIGGER_MODE_START_x1          0x02                                  /* Start by INTx1 */
#define VE_TRIGGER_MODE_START_BOTH        0x03                                  /* Start when both INTA0 (unit A) and INTA1 (unit B) occur */

#define VE_CPURUNTRG_CHANNEL_0            (0x1 << 0)                            /* Starts channel 0 by programming */
#define VE_CPURUNTRG_CHANNEL_1            (0x1 << 1)                            /* Starts channel 1 by programming */

/* FMODE */
#define VE_FMODE_3PHASE_MODULATION        (0 << 0)
#define VE_FMODE_2PHASE_MODULATION        (1 << 0)
#define VE_FMODE_PWM_SHIFT_DISABLE        (0 << 1)
#define VE_FMODE_PWM_SHIFT_ENABLE         (1 << 1)
#define VE_FMODE_CURR_DETECT_3SHUNT       (0x0 << 2)
#define VE_FMODE_CURR_DETECT_2SENSOR      (0x1 << 2)
#define VE_FMODE_CURR_DETECT_1SHUNT_UP    (0x2 << 2)
#define VE_FMODE_CURR_DETECT_1SHUNT_DOWN  (0x3 << 2)
#define VE_FMODE_PMD_CHANNEL_0            (0 << 4)
#define VE_FMODE_PMD_CHANNEL_1            (1 << 4)
#define VE_FMODE_ADCSEL_UNIT_A            (0x0 << 6)
#define VE_FMODE_ADCSEL_UNIT_B            (0x1 << 6)
#define VE_FMODE_ADCSEL_UNIT_AB           (0x2 << 6)
#define VE_FMODE_MREGDIS                  (1 << 9)

/* MODE */
#define VE_MODE_PHASE_INTERPOLATION_EN    (1 << 0)
#define VE_MODE_ZERO_CURRENT_DETECT       (1 << 1)
#define VE_MODE_OUTPUT_OFF                (0x0 << 2)
#define VE_MODE_OUTPUT_ENABLE             (0x1 << 2)
#define VE_MODE_OUTPUT_OFF_EMG_RET        (0x3 << 2)

/* REPTIME */
#define VE_REPTIME_CHANNEL_0_SHIFT        0
#define VE_REPTIME_CHANNEL_1_SHIFT        4
#define VE_REPTIME_NO_SCHEDULE            0
#define VE_REPTIME_1                      0x01
#define VE_REPTIME_2                      0x02
#define VE_REPTIME_3                      0x03
#define VE_REPTIME_4                      0x04
#define VE_REPTIME_5                      0x05
#define VE_REPTIME_6                      0x06
#define VE_REPTIME_7                      0x07
#define VE_REPTIME_8                      0x08
#define VE_REPTIME_9                      0x09
#define VE_REPTIME_10                     0x0a
#define VE_REPTIME_11                     0x0b
#define VE_REPTIME_12                     0x0c
#define VE_REPTIME_13                     0x0d
#define VE_REPTIME_14                     0x0e
#define VE_REPTIME_15                     0x0f

/* TASKAPP */
#define VE_TASKAPP_CLEAN_MASK                         0x0f
#define VE_TASKAPP_OUTPUT_CONTROL                     0x00
#define VE_TASKAPP_TRIGGER_GENERATION                 0x01
#define VE_TASKAPP_INPUT_PROCESSING                   0x02
#define VE_TASKAPP_INPUT_PHASE_CONVERSION             0x03
#define VE_TASKAPP_INPUT_COORDINATE_AXIS_CONVERSION   0x04
#define VE_TASKAPP_CURRENT_CONTROL                    0x05
#define VE_TASKAPP_SIN_COS_COMPUTATION                0x06
#define VE_TASKAPP_OUTPUT_COORDINATE_AXIS_CONVERSION  0x07
#define VE_TASKAPP_OUTPUT_PHASE_CONVERSION            0x08

/* ACTSCH */
#define VE_ACTSCH_CLEAN_MASK                0x0f
#define VE_ACTSCH_INDIVIDUAL_TASK_EXECUTION 0x00
#define VE_ACTSCH_SCHEDULE_0                0x00
#define VE_ACTSCH_SCHEDULE_1                0x01
#define VE_ACTSCH_SCHEDULE_4                0x04
#define VE_ACTSCH_SCHEDULE_9                0x09

typedef struct
{
  __IO uint32_t EN;                                                             /*!< VE enable/disable                            */
  __O  uint32_t CPURUNTRG;                                                      /*!< CPU start trigger selection                  */
  __IO uint32_t TASKAPP;                                                        /*!< Task selection                               */
  __IO uint32_t ACTSCH;                                                         /*!< Operation schedule selection                 */
  __IO uint32_t REPTIME;                                                        /*!< Schedule repeat count                        */
  __IO uint32_t TRGMODE;                                                        /*!< Start trigger mode                           */
  __IO uint32_t ERRINTEN;                                                       /*!< Error interrupt enable/disable               */
  __O  uint32_t COMPEND;                                                        /*!< VE forced termination                        */
  __I  uint32_t ERRDET;                                                         /*!< Error detection                              */
  __I  uint32_t SCHTASKRUN;                                                     /*!< Schedule executing flag/executing task       */
       uint32_t RESERVED0;
  __IO uint32_t TMPREG0;                                                        /*!< Temporary register                           */
  __IO uint32_t TMPREG1;                                                        /*!< Temporary register                           */
  __IO uint32_t TMPREG2;                                                        /*!< Temporary register                           */
  __IO uint32_t TMPREG3;                                                        /*!< Temporary register                           */
  __IO uint32_t TMPREG4;                                                        /*!< Temporary register                           */
  __IO uint32_t TMPREG5;                                                        /*!< Temporary register                           */
       uint32_t RESERVED00;
       uint32_t RESERVED01;
       uint32_t RESERVED02;
       uint32_t RESERVED03;
       uint32_t RESERVED04;
       uint32_t RESERVED05;
       uint32_t RESERVED06;
       uint32_t RESERVED07;
       uint32_t RESERVED08;
       uint32_t RESERVED09;
       uint32_t RESERVED10;
       uint32_t RESERVED11;
       uint32_t RESERVED12;
       uint32_t RESERVED13;
       uint32_t RESERVED14;
       uint32_t RESERVED15;
       uint32_t RESERVED16;
       uint32_t RESERVED17;
       uint32_t RESERVED18;
       uint32_t RESERVED19;
       uint32_t RESERVED20;
       uint32_t RESERVED21;
       uint32_t RESERVED22;
       uint32_t RESERVED23;
       uint32_t RESERVED24;
       uint32_t RESERVED25;
       uint32_t RESERVED26;
       uint32_t RESERVED27;
       uint32_t RESERVED28;
       uint32_t RESERVED29;
       uint32_t RESERVED30;
       uint32_t RESERVED31;
       uint32_t RESERVED32;
       uint32_t RESERVED33;
       uint32_t RESERVED34;
       uint32_t RESERVED35;
       uint32_t RESERVED36;
       uint32_t RESERVED37;
       uint32_t RESERVED38;
       uint32_t RESERVED39;
       uint32_t RESERVED40;
       uint32_t RESERVED41;
       uint32_t RESERVED42;
       uint32_t RESERVED43;
       uint32_t RESERVED44;
       uint32_t RESERVED45;
       uint32_t RESERVED46;
       uint32_t RESERVED47;
       uint32_t RESERVED48;
       uint32_t RESERVED49;
       uint32_t RESERVED50;
       uint32_t RESERVED51;
       uint32_t RESERVED52;
       uint32_t RESERVED53;
       uint32_t RESERVED54;
       uint32_t RESERVED55;
       uint32_t RESERVED56;
       uint32_t RESERVED57;
       uint32_t RESERVED58;
       uint32_t RESERVED59;
       uint32_t RESERVED60;
       uint32_t RESERVED61;
       uint32_t RESERVED62;
       uint32_t RESERVED63;
       uint32_t RESERVED64;
       uint32_t RESERVED65;
       uint32_t RESERVED66;
       uint32_t RESERVED67;
       uint32_t RESERVED68;
       uint32_t RESERVED69;
       uint32_t RESERVED70;
       uint32_t RESERVED71;
       uint32_t RESERVED72;
       uint32_t RESERVED73;
       uint32_t RESERVED74;
       uint32_t RESERVED75;
       uint32_t RESERVED76;
  __IO uint32_t TADC;                                                           /*!< Common ADC conversion time (based on PWM clock) */
} TEE_VEC_TypeDef;

typedef struct
{
  __IO uint32_t MCTLF;                                                          /*!< Status flags                                 */
  __IO uint32_t MODE;                                                           /*!< Task control mode                            */
  __IO uint32_t FMODE;                                                          /*!< Flow control                                 */
  __IO uint32_t TPWM;                                                           /*!< PWM period rate (PWM period [s] ¡Á maximum speed*1 ¡Á 2^16) */
  __IO uint32_t OMEGA;                                                          /*!< Rotation speed (speed [Hz]¡Â maximum speed *1¡ù1 ¡Á 2^15) */
  __IO uint32_t THETA;                                                          /*!< Motor phase (motor phase [deg]/360 ¡Á 2^16) */
  __IO uint32_t IDREF;                                                          /*!< d-axis reference value (current [A] ¡Â maximum current*2 ¡Á 2^15) */
  __IO uint32_t IQREF;                                                          /*!< q-axis reference value (current [A] ¡Â maximum current *2 ¡Á 2^15) */
  __IO uint32_t VD;                                                             /*!< d-axis voltage (voltage [V] ¡Â maximum voltage *3 ¡Á 2^31)*/
  __IO uint32_t VQ;                                                             /*!< q-axis voltage (voltage [V] ¡Â maximum voltage *3 ¡Á 2^31)*/
  __IO uint32_t CIDKI;                                                          /*!< Integral coefficient for PI control of d-axis */
  __IO uint32_t CIDKP;                                                          /*!< Proportional coefficient for PI control of d-axis */
  __IO uint32_t CIQKI;                                                          /*!< Integral coefficient for PI control of q-axis */
  __IO uint32_t CIQKP;                                                          /*!< Proportional coefficient for PI control of q-axis */
  __IO uint32_t VDIH;                                                           /*!< Upper 32 bits of integral term (VDI ) of d-axis voltage */
  __IO uint32_t VDILH;                                                          /*!< Lower 32 bits of integral term (VDI) of d-axis voltage */
  __IO uint32_t VQIH;                                                           /*!< Upper 32 bits of integral term (VQI) of q-axis voltage */
  __IO uint32_t VQILH;                                                          /*!< Lower 32 bits of integral term (VQI) of q-axis voltage */
  __IO uint32_t FPWMCHG;                                                        /*!< Switching speed (for 2-phase modulation and shift PWM) */
  __IO uint32_t MDPRD;                                                          /*!< PWM period (to be set identically with PMD¡¯s PWM period) */
  __IO uint32_t MINPLS;                                                         /*!< Minimum pulse width                          */
  __IO uint32_t TRGCRC;                                                         /*!< Synchronizing trigger correction value       */
       uint32_t RESERVED;
  __IO uint32_t COS;                                                            /*!< Cosine value at THETA for output conversion (Q15 data) */
  __IO uint32_t SIN;                                                            /*!< Sine value at THETA for output conversion (Q15 data) */
  __IO uint32_t COSM;                                                           /*!< Previous cosine value for input processing (Q15 data) */
  __IO uint32_t SINM;                                                           /*!< Previous sine value for input processing (Q15 data) */
  __IO uint32_t SECTOR;                                                         /*!< Sector information (0-11)                    */
  __IO uint32_t SECTORM;                                                        /*!< Previous sector information for input processing (0-11) */
  __IO uint32_t IA0;                                                            /*!< AD conversion result of a-phase zero-current *4 */
  __IO uint32_t IB0;                                                            /*!< AD conversion result of b-phase zero-current *4 */
  __IO uint32_t IC0;                                                            /*!< AD conversion result of c-phase zero-current *4 */
  __IO uint32_t IAADC;                                                          /*!< AD conversion result of a-phase current *4    */
  __IO uint32_t IBADC;                                                          /*!< AD conversion result of b-phase current *4    */
  __IO uint32_t ICADC;                                                          /*!< AD conversion result of c-phase current *4    */
  __IO uint32_t VDC;                                                            /*!< DC supply voltage (voltage [V] ¡Â maximum voltage *3 ¡Á 2^15) */
  __IO uint32_t ID;                                                             /*!< d-axis current (current [A] ¡Â maximum current *2 ¡Á 2^31) */
  __IO uint32_t IQ;                                                             /*!< q-axis current (current [A] ¡Â maximum current *2 ¡Á 2^31) */
} TEE_VE_TypeDef;

typedef struct
{
  __IO uint32_t VCMPU;                                                          /*!< PMD control: CMPU setting                    */
  __IO uint32_t VCMPV;                                                          /*!< PMD control: CMPV setting                    */
  __IO uint32_t VCMPW;                                                          /*!< PMD control: CMPW setting                    */
  __IO uint32_t OUTCR;                                                          /*!< PMD control: Output control (MDOUT)          */
  __IO uint32_t VTRGCMP0;                                                       /*!< PMD control: TRGCMP0 setting                 */
  __IO uint32_t VTRGCMP1;                                                       /*!< PMD control: TRGCMP1 setting                 */
  __IO uint32_t VTRGSEL;                                                        /*!< PMD control: Trigger selection               */
  __O  uint32_t EMGRS;                                                          /*!< PMD control: EMG return (EMGCR[EMGRS])       */
}TEE_VEPMD_TypeDef;

#define TEE_VEC    (( TEE_VEC_TypeDef *)   TSB_VE_BASE)
#define TEE_VE0    (( TEE_VE_TypeDef  *)   (TSB_VE_BASE+0x044))
#define TEE_VE1    (( TEE_VE_TypeDef  *)   (TSB_VE_BASE+0x0dc))
#define TEE_VEPMD0 (( TEE_VEPMD_TypeDef *) (TSB_VE_BASE+0x17c))
#define TEE_VEPMD1 (( TEE_VEPMD_TypeDef *) (TSB_VE_BASE+0x19c))


/*! \brief Fractional representation of values
 *
 *  Some calcalations of register values are done with fractional parts (due to small numbers)
 *  The upper 16bit are representing the register value, while the lower 16bit are representing
 *  the fractional part
 */
typedef union
{
  struct
  {
    int16_t frac;
    int16_t reg;
  } part;
  int32_t value;
} FRACTIONAL;

/*! \brief Vector Engine Main stages
 *
 *  The different stages the Vector Engine passes till reaching target speed
 */
enum _mainstage
{
  Stage_Stop,                                                                   /* control loop is off, no actions pending */
  Stage_Bootstrap,                                                              /* bootstrap IGBTs */
  Stage_Brake,                                                                  /* shortbrake Motor */
  Stage_ZeroCurrentMeasure,                                                     /* measure zero current adc value */
  Stage_Initposition,                                                           /* control loop is off, first signals out and in */
  Stage_Force,                                                                  /* forced commutation */
  Stage_FOC,                                                                    /* Field Oriented Commutation */
  Stage_Emergency                                                               /* emergency state */
};

/*! \brief Substages
 *
 *  The different substages inside the mainstages
 */
enum _substage {
  Substage_Step0,
  Substage_Step1,
  Substage_Step2,
  Substage_StepEnd };

/*! \brief Vector Engine Actual State
 *
 *  Major and minor actual stage
 */
typedef struct
{
  enum _mainstage main;
  enum _substage  sub;
} VEStage;

/*! \brief Precalculated Values
 *
 *  Struct holding some precalculated values due to run time optimisation
 */
typedef struct
{
  uint16_t WaitTime_Position;
  uint16_t WaitTime_Bootstrap;
  uint32_t R_mult_amax_div_vmax;                                                /* fixpoint15 calculation R*I/U */
  uint32_t InductanceDropoutFactor;                                             /* fixpoint15 calculation 2*PI*I*Speed*1000/U */
  uint32_t PositionKiPreCalculation;                                            /* fixpoint15 calculation Ki*Tt*U/f */
  uint32_t PositionKpPreCalculation;                                            /* fixpoint15 calculation Kp*U/f */
  uint16_t ChangeFrqUp_normed_to_HzMax;                                         /* fixpoint15 calculation HzChange normalized to HzMax */
  uint16_t ChangeFrqDown_normed_to_HzMax;                                       /* fixpoint15 calculation HzChange normalized to HzMax */
  uint16_t IqLim_normed_to_amax;                                                /* fixpoint15 calculation IqLim normalized to a_max */
  uint16_t IdLim_normed_to_amax;                                                /* fixpoint31 calculation IdLim normalized to a_max */
  uint32_t HzMax_normed_to_PWMFrequency;                                        /* fixpoint   calcualtion HzMax normalized to PWMFrequency */
  uint32_t SpeedUpLimit;
  uint16_t IdCurrentForInitposition;
  uint16_t IqCurrentForInitposition;  
} PreCalc;                                                                      /* set of pre-calculations for control loop */

/*! \brief Shutdown mode
 *
 *  Selects how the motor should behave in case of stop command
 */
typedef enum {
  SHUTDOWN_DISABLE_OUTPUT,
  SHUTDOWN_SHORT_BRAKE,
  SHUTDOWN_GENTLE,
} VE_Shutdown;

/*! \brief Restart mode
 *
 *  Selects how the motor should behave in case of stalled electromagnetic field
 */
typedef enum {
  SWITCH_OFF_MOTOR,
  RESTART_MOTOR,
} VE_Restart;

/*! \brief PMD output state
 *
 *  Switch off or on the PMD
 */
typedef enum {
  PMD_OFF,
  PMD_ON,
} VE_PMD_State;

/*! \brief VE interrupt state
 *
 *  Switch off or on the IRQs
 */
typedef enum {
  VE_IRQ_OFF,
  VE_IRQ_ON,
} VE_IRQ_State;


int8_t VE_Start(uint8_t channel_number);
int8_t VE_Stop (uint8_t channel_number);

/* global data */
extern int16_t              VE_Id[MAX_CHANNEL];                                 /* [A/maxA] d-axis current */
extern int16_t              VE_Iq[MAX_CHANNEL];                                 /* [A/maxA] q-axis current */
extern int16_t              VE_Id_reference[MAX_CHANNEL];                       /* [A/maxA] Reference of d-axis c */
extern int16_t              VE_Iq_reference[MAX_CHANNEL];                       /* [A/maxA] Reference of q-axis c */
extern FRACTIONAL           VE_Omega[MAX_CHANNEL];                              /* [Hz/maxHz] Omega(speed): Elect */
extern FRACTIONAL           VE_Omega[MAX_CHANNEL];
extern FRACTIONAL           VE_Theta[MAX_CHANNEL];
extern FRACTIONAL           VE_Omega_command[MAX_CHANNEL];
extern PreCalc              VE_PreCalc[MAX_CHANNEL];
extern VEStage              VE_ActualStage[MAX_CHANNEL];
extern uint16_t             VE_HzMax[MAX_CHANNEL];
extern uint32_t             VE_v_max[MAX_CHANNEL];
extern uint32_t             VE_a_max[MAX_CHANNEL];
extern int16_t              VE_OmegaCalc[MAX_CHANNEL];
#endif /* _VE_DEFINE_H_ */
