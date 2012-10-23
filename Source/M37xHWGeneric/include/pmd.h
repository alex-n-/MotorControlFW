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

#ifndef _PMD_H
#define _PMD_H

#include "adc.h"

#include "config.h"
#include TMPM_HEADER_FILE

#define PMD_TRG_1SHUNT1           (0x0022u)                           /*!< 1shunt (TRG0:Down,TRG1:UP,TRG3:Peek */
#define PMD_TRG_1SHUNT            (0x0011u)                           /*!< 1shunt (TRG0:Down,TRG1:UP,TRG3:Peek */
#define PMD_TRG_3SHUNT            (0x0004u)                           /*!< 3shunt (TRG0:Peek) */
#define PMD_TRG_2SENSOR           (0x0004u)                           /*!< 2sensor (TRG0:Peek) */

#define PMD_ENABLE                (1 << 0)                            /*!< Mask PMD enable */

/* PORTMD */
#define PMD_OUT_ALL_HIGH_Z        0x00                                /*!< Mask PMD all outputs high Z */
#define PMD_OUT_LOW_PMD           (1 << 0)                            /*!< Mask PMD all outputs upper phase high Z, lower phase out */
#define PMD_OUT_HIGH_PMD          (1 << 1)                            /*!< Mask PMD all outputs upper phase out, lower phase high Z */
#define PMD_OUT_ALL_PMD           ((1 << 0) | (1 << 1))               /*!< BITRANGE(1,0) Mask PMD all outputs upper phase out, lower phase out */

/* MDCR */
#define PMD_CR_CARRIER_SAW        0                                   /*!< Mask PMD saw tooth modulation */
#define PMD_CR_CARRIER_TRIANG     (1 << 0)                            /*!< Mask PMD triangle modulation */
#define PMD_CR_IRQ_PERIODE_05     0                                   /*!< Mask PMD IRQ every 0.5 cycle */
#define PMD_CR_IRQ_PERIODE_1      (1 << 1)                            /*!< Mask PMD IRQ every 1 cycle */
#define PMD_CR_IRQ_PERIODE_2      (1 << 2)                            /*!< Mask PMD IRQ every 2 cycle */
#define PMD_CR_IRQ_PERIODE_4      ((1 << 1) | (1 << 2))               /*!< BITRANGE(2,1) Mask PMD IRQ every 4 cycle */
#define PMD_CR_IRQ_COUNT_1        0                                   /*!< Mask PMD IRQ timing counter = 1 */
#define PMD_CR_IRQ_COUNT_MDPRD    (1 << 3)                            /*!< Mask PMD IRQ timing counter = period */
#define PMD_CR_DUTY_COMMON        0                                   /*!< Mask PMD 3 phase common mode */
#define PMD_CR_DUTY_INDEPEND      (1 << 4)                            /*!< Mask PMD 3 phase independent mode */
#define PMD_CR_SYNTMD_LOW         0                                   /*!< Mask PMD port output mode */
#define PMD_CR_SYNTMD_HIGH        (1 << 5)                            /*!< Mask PMD port output mode */
#define PMD_CR_PWM_EXPER_0        0                                   /*!< Mask PMD period extension mode normal */
#define PMD_CR_PWM_EXPER_4        (1 << 6)                            /*!< Mask PMD period extension mode 4x */

/* CNTSTA */
#define PMD_PWM_COUNT_STAT_MASK   (1 << 0)                            /*!< Flag PMD counter flag Down */

/* MDCNT */
#define PMD_MD_COUNTER_MASK       _BITMASK(16)                        /*!< Mask PMD counter */

/* MDPRD */
#define PMD_MD_PERIOD_MASK        _BITMASK(16)                        /*!< Mask PMD period */

/*
 * CMPU, ;
 * CMPV, ;
 * CMP
 */
#define PMD_MD_COMPARE_MASK       _BITMASK(16)                        /*!< Mask PMD compare */

/* MODSEL */
#define PMD_MODE_BUS              0                                   /*!< Flag PMD Bus Mode */
#define PMD_MODE_VE               (1 << 0)                            /*!< Flag PMD VE Mode */

/* MDOUT */
#define PMD_MDOUT_PWM_SEL         (_BITRANGE(5, 0) | _BITRANGE(10, 8))/*!< Mask PMD MDOut register */
#define PMD_MDOUT_PWM_MASK        (_BITRANGE(5, 0) | _BITRANGE(10, 8))/*!< Flag PMD all MDOut register */
#define PMD_MDOUT_OC_0            0x00                                /*!< Flag PMD MDOut 0 */
#define PMD_MDOUT_OC_1            0x01                                /*!< Flag PMD MDOut 1 */
#define PMD_MDOUT_OC_2            0x02                                /*!< Flag PMD MDOut 2 */
#define PMD_MDOUT_OC_3            0x03                                /*!< Flag PMD MDOut 3 */
#define PMD_MDOUT_OC_U0           (PMD_MDOUT_OC_0 << 0u)              /*!< Flag PMD MDOut U0 */
#define PMD_MDOUT_OC_U1           (PMD_MDOUT_OC_1 << 0u)              /*!< Flag PMD MDOut U1 */
#define PMD_MDOUT_OC_U2           (PMD_MDOUT_OC_2 << 0u)              /*!< Flag PMD MDOut U2 */
#define PMD_MDOUT_OC_U3           (PMD_MDOUT_OC_3 << 0u)              /*!< Flag PMD MDOut U3 */
#define PMD_MDOUT_OC_V0           (PMD_MDOUT_OC_0 << 2u)              /*!< Flag PMD MDOut V0 */
#define PMD_MDOUT_OC_V1           (PMD_MDOUT_OC_1 << 2u)              /*!< Flag PMD MDOut V1 */
#define PMD_MDOUT_OC_V2           (PMD_MDOUT_OC_2 << 2u)              /*!< Flag PMD MDOut V2 */
#define PMD_MDOUT_OC_V3           (PMD_MDOUT_OC_3 << 2u)              /*!< Flag PMD MDOut V3 */
#define PMD_MDOUT_OC_W0           (PMD_MDOUT_OC_0 << 4u)              /*!< Flag PMD MDOut W0 */
#define PMD_MDOUT_OC_W1           (PMD_MDOUT_OC_1 << 4u)              /*!< Flag PMD MDOut W1 */
#define PMD_MDOUT_OC_W2           (PMD_MDOUT_OC_2 << 4u)              /*!< Flag PMD MDOut W2 */
#define PMD_MDOUT_OC_W3           (PMD_MDOUT_OC_3 << 4u)              /*!< Flag PMD MDOut W3 */

#define PMD_MDOUT_PWM_U           (1 << 8)                            /*!< Flag PMD MDOut U control */
#define PMD_MDOUT_PWM_V           (1 << 9)                            /*!< Flag PMD MDOut V control */
#define PMD_MDOUT_PWM_W           (1 << 10)                           /*!< Flag PMD MDOut W control */

/* MDPOT */
#define PMD_PSYNC_ASYNC           0x00                                /*!< Flag PMD MDOut transfer async */
#define PMD_PSYNC_CNT_1           (1 << 0)                            /*!< Flag PMD MDOut transfer if counter = 1 */
#define PMD_PSYNC_CNT_PER         (1 << 1)                            /*!< Flag PMD MDOut transfer if counter = period */
#define PMD_PSYNC_CNT_1_OR_PER    _BITRANGE(1, 0)                     /*!< Flag PMD MDOut transfer if counter = period or 1 */
#define PMD_POLL_ACT_LOW          0x00                                /*!< Flag PMD MDOut low phase polarity active low */
#define PMD_POLL_ACT_HIGH         (1 << 2)                            /*!< Flag PMD MDOut low phase polarity active high */
#define PMD_POLH_ACT_LOW          0x00                                /*!< Flag PMD MDOut high phase polarity active low */
#define PMD_POLH_ACT_HIGH         (1 << 3)                            /*!< Flag PMD MDOut high phase polarity active high */

// EMGREL
#define PMD_EMG_KEY               0x5AA5                              /*!<Mask PMD emergency password */
#define PMD_MD_EMGREL_MASK        _BITMASK(8)                         /*!<Flag PMD emergency release */
// EMGCR
#define PMD_EMGCR_DISABLE         0                                   /*!<Flag PMD emergency disable */
#define PMD_EMGCR_ENABLE          (1<<0)                              /*!<Flag PMD emergency enable */
#define PMD_EMGCR_PROT_REL        (1<<1)                              /*!<Flag PMD protection mode release */
#define PMD_EMGCR_INPUT_SEL       (1<<2)                              /*!<Flag PMD EMG input select comparator */
#define PMD_EMGCR_PROT_MODE_MASK  _BITRANGE(4,3)                      /*!<Mask PMD EMG mode */
#define PMD_EMGCR_PROT_MODE_DIS   0                                   /*!<Flag PMD EMG mode output all high Z */
#define PMD_EMGCR_PROT_MODE_UPPER (1<<3)                              /*!<Flag PMD EMG mode output lower phases high Z */
#define PMD_EMGCR_PROT_MODE_LOWER (1<<4)                              /*!<Flag PMD EMG mode output higher phases high Z */
#define PMD_EMGCR_PROT_MODE_OFF   ((1<<4|(1<<3))                      /*!<Flag PMD EMG mode output all phases high Z */
#define PMD_EMGCR_TOOLBRK_ENA     (1<<5)                              /*!<Flag PMD Tool break enable */
#define PMD_EMGCR_COUNT_MASK      _BITRANGE(11,8)                     /*!<Flag PMD EMG input detection time */
// EMGSTA
#define PMD_EMG_STATUS_PROT       (1<<0)                              /*!<Flag PMD EMG status protected */
#define PMD_EMG_STATUS_INPUT      (1<<1)                              /*!<Flag PMD EMG status input */
// OVVCR
#define PMD_OVVCR_DISABLE         0                                   /*!<Flag PMD OVV control disabled */
#define PMD_OVVCR_ENABLE          (1<<0)                              /*!<Flag PMD OVV control enabled */
#define PMD_OVVCR_PROT_REL        (1<<1)                              /*!<Flag PMD OVV protection release */
#define PMD_OVVCR_INPUT_SEL       (1<<2)                              /*!<Flag PMD OVV input select ADC monitor */
#define PMD_OVVCR_PROT_MODE_MASK  _BITRANGE(4,3)                      /*!<Mask PMD OVV protection mode */
#define PMD_OVVCR_PROT_MODE_DIS   0                                   /*!<Flag PMD OVV protection mode disabled */
#define PMD_OVVCR_PROT_MODE_UPPER (1<<3)                              /*!<Flag PMD OVV protection mode lower phases off */
#define PMD_OVVCR_PROT_MODE_LOWER (1<<4)                              /*!<Flag PMD OVV protection mode upper phases off */
#define PMD_OVVCR_PROT_MODE_OFF   ((1<<4|(1<<3))                      /*!<Flag PMD OVV protection mode all phases off */
#define PMD_OVVCR_ADC0INT_ENA     (1<<5)                              /*!<Flag PMD OVV protection ADCA input enable */
#define PMD_OVVCR_ADC1INT_ENA     (1<<6)                              /*!<Flag PMD OVV protection ADCB input enable */
#define PMD_OVVCR_COUNT_MASK      _BITRANGE(11,8)                     /*!<Mask PMD OVV input detection time */
#define PMD_OVVCR_KEY             {0x5A,0xA5}                         /*!<PMD OVV protection release key */
// OVVSTA
#define PMD_OVV_STATUS_PROT       (1<<0)                              /*!<Flag PMD OVV protection status enable */
#define PMD_OVV_STATUS_INPUT      (1<<1)                              /*!<Flag PMD OVV protection status input */
// DTR
#define PMD_DTR_MASK              _BITMASK(8)                         /*!<Mask PMD dead time register */
/*
 * TRGCMP0, ;
 * TRGCMP1 ;
 * TRGCMP2, ;
 * TRGCMP3
 */
#define PMD_TRGCMP_MASK           _BITMASK(16)                        /*!< Mask PMD trigger compare */

/* TRGCR */
#define PMD_TRG0_SartBit          0                                   /*!< number of field trigger 0 */
#define PMD_TRG0_MODE_MASK        _BITRANGE(2, 0)                     /*!< Mask PMD trigger0 control */
#define PMD_TRG0_MODE_DISABLED0   (0x00 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 disable */
#define PMD_TRG0_MODE_DOWN_CNT    (0x01 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 count down */
#define PMD_TRG0_MODE_UP_CNT      (0x02 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 count up */
#define PMD_TRG0_MODE_UPDOWN_CNT  (0x03 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 count up/down */
#define PMD_TRG0_MODE_PWM_PEAK    (0x04 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 peak */
#define PMD_TRG0_MODE_PWM_BOTTOM  (0x05 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 bottom */
#define PMD_TRG0_MODE_PWM_PEAKBOT (0x06 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 peak/bottom */
#define PMD_TRG0_MODE_DISABLED1   (0x07 << PMD_TRG0_SartBit)          /*!< Flag PMD trigger0 disabled */
#define PMD_TRG0_BUFFER_SYNC_UPD  (1 << 3)                            /*!< Flag PMD trigger0 async */

#define PMD_TRG1_SartBit          4                                   /*!< number of field trigger 1 */
#define PMD_TRG1_MODE_MASK        _BITRANGE(6, PMD_TRG1_SartBit)      /*!< Mask PMD trigger1 control */
#define PMD_TRG1_MODE_DISABLED0   (0x00 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 disable */
#define PMD_TRG1_MODE_DOWN_CNT    (0x01 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 count down */
#define PMD_TRG1_MODE_UP_CNT      (0x02 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 count up */
#define PMD_TRG1_MODE_UPDOWN_CNT  (0x03 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 count up/down */
#define PMD_TRG1_MODE_PWM_PEAK    (0x04 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 peak */
#define PMD_TRG1_MODE_PWM_BOTTOM  (0x05 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 bottom */
#define PMD_TRG1_MODE_PWM_PEAKBOT (0x06 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 peak/bottom */
#define PMD_TRG1_MODE_DISABLED1   (0x07 << PMD_TRG1_SartBit)          /*!< Flag PMD trigger1 disabled */
#define PMD_TRG1_BUFFER_SYNC_UPD  (1 << 7)                            /*!< Flag PMD trigger1 async */

#define PMD_TRG2_SartBit          8                                   /*!< number of field trigger 2 */
#define PMD_TRG2_MODE_MASK        _BITRANGE(10, PMD_TRG2_SartBit)     /*!< Mask PMD trigger2 control */
#define PMD_TRG2_MODE_DISABLED0   (0x00 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 disable */
#define PMD_TRG2_MODE_DOWN_CNT    (0x01 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 count down */
#define PMD_TRG2_MODE_UP_CNT      (0x02 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 count up */
#define PMD_TRG2_MODE_UPDOWN_CNT  (0x03 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 count up/down */
#define PMD_TRG2_MODE_PWM_PEAK    (0x04 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 peak */
#define PMD_TRG2_MODE_PWM_BOTTOM  (0x05 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 bottom */
#define PMD_TRG2_MODE_PWM_PEAKBOT (0x06 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 peak/bottom */
#define PMD_TRG2_MODE_DISABLED1   (0x07 << PMD_TRG2_SartBit)          /*!< Flag PMD trigger2 disabled */
#define PMD_TRG2_BUFFER_SYNC_UPD  (1 << 11)                           /*!< Flag PMD trigger2 async */

#define PMD_TRG3_SartBit          12                                  /*!< number of field trigger 3 */
#define PMD_TRG3_MODE_MASK        _BITRANGE(14, PMD_TRG3_SartBit)     /*!< Mask PMD trigger3 control */
#define PMD_TRG3_MODE_DISABLED0   (0x00 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 disable */
#define PMD_TRG3_MODE_DOWN_CNT    (0x01 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 count down */
#define PMD_TRG3_MODE_UP_CNT      (0x02 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 count up */
#define PMD_TRG3_MODE_UPDOWN_CNT  (0x03 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 count up/down */
#define PMD_TRG3_MODE_PWM_PEAK    (0x04 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 peak */
#define PMD_TRG3_MODE_PWM_BOTTOM  (0x05 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 bottom */
#define PMD_TRG3_MODE_PWM_PEAKBOT (0x06 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 peak/bottom */
#define PMD_TRG3_MODE_DISABLED1   (0x07 << PMD_TRG3_SartBit)          /*!< Flag PMD trigger3 disabled */
#define PMD_TRG3_BUFFER_SYNC_UPD  (1 << 15)                           /*!< Flag PMD trigger3 async */

/* TRGMD */
#define PMD_EMG_TRG_PROT_DISABLE  0                                   /*!< Flag PMD trigger protection disabled */
#define PMD_EMG_TRG_PROT_ENABLE   (1 << 0)                            /*!< Flag PMD trigger protection enabled */
#define PMD_TRG_MODE_FIXED        0                                   /*!< Flag PMD trigger mode fixed */
#define PMD_TRG_MODE_VARIABLE     (1 << 1)                            /*!< Flag PMD trigger mode variable */

/* TRGSEL */
#define PMD_TRGSEL_MASK           _BITRANGE(2, 0)                     /*!< Mask PMD trigger output select */
#define PMD_TRGSEL_PMDTRG0        0                                   /*!< Flag PMD trigger output from PMDTRG0 */
#define PMD_TRGSEL_PMDTRG1        1                                   /*!< Flag PMD trigger output from PMDTRG1 */
#define PMD_TRGSEL_PMDTRG2        2                                   /*!< Flag PMD trigger output from PMDTRG2 */
#define PMD_TRGSEL_PMDTRG3        3                                   /*!< Flag PMD trigger output from PMDTRG3 */
#define PMD_TRGSEL_PMDTRG4        4                                   /*!< Flag PMD trigger output from PMDTRG4 */
#define PMD_TRGSEL_PMDTRG5        5                                   /*!< Flag PMD trigger output from PMDTRG5 */
#define PMD_TRGSEL_NO_PMDTRG0     6                                   /*!< Flag PMD trigger output from PMDTRG6 */
#define PMD_TRGSEL_NO_PMDTRG1     7                                   /*!< Flag PMD trigger output from PMDTRG7 */

void PMD_ShortBrake             (uint8_t channel_number);
void PMD_NormalOutput           (uint8_t channel_number);
void PMD_SwitchOff              (uint8_t channel_number);
void PMD_HandleParameterChange  (uint8_t channel_number);
void PMD_OvervoltageReset       (uint8_t channel_number);
void PMD_EmergencyReset         (uint8_t channel_number);
void PMD_Init                   (uint8_t channel_number);
#endif
