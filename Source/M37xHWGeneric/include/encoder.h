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

#ifndef _ENCODER_H_
#define _ENCODER_H_

#define ENC_IRQ_ENABLE        (1 << 3)                                /*!< Flag Encoder IRQ enable */
#define ENC_RUN_ENABLE        (1 << 6)                                /*!< Flag Encoder run enable */

#define ENC_PULSE_DIV1        0x00u                                   /*!< Flag Encoder pulse division by 1 */
#define ENC_PULSE_DIV2        0x01u                                   /*!< Flag Encoder pulse division by 2 */
#define ENC_PULSE_DIV4        0x02u                                   /*!< Flag Encoder pulse division by 4 */
#define ENC_PULSE_DIV8        0x03u                                   /*!< Flag Encoder pulse division by 5 */
#define ENC_PULSE_DIV16       0x04u                                   /*!< Flag Encoder pulse division by 16 */
#define ENC_PULSE_DIV32       0x05u                                   /*!< Flag Encoder pulse division by 32 */
#define ENC_PULSE_DIV64       0x06u                                   /*!< Flag Encoder pulse division by 64 */
#define ENC_PULSE_DIV128      0x07u                                   /*!< Flag Encoder pulse division by 128 */
#define ENC_NOISEFILTER_OFF   0x00u                                   /*!< Flag Encoder noise filter off */
#define ENC_NOISEFILTER_31    (1 << 4)                                /*!< Flag Encoder noise filter 31 samples */
#define ENC_NOISEFILTER_63    (1 << 5)                                /*!< Flag Encoder noise filter 63 samples */
#define ENC_NOISEFILTER_127   ((1 << 4) | (1 << 5))                   /*!< Flag Encoder noise filter 127 samples */
#define ENC_ZPHASE_ENABLE     (1 << 7)                                /*!< Flag Encoder Z phase enable */

#define ENC_CMP_ENABLE        (1 << 8)                                /*!< Flag Encoder comparator enable */
#define ENC_ZTRIGGER_FALLING  (1 << 9)                                /*!< Flag Encoder Z trigger falling */

#define ENC_COUNTER_CLEAR     (1 << 10)                               /*!< Flag Encoder counter clear */

#define ENC_SW_CAPTURE        (1 << 11)                               /*!< Flag Encoder SW capture enable */
#define ENC_ZPHASE_DETECTED   (1 << 12)                               /*!< Flag Encoder Z phase detect */

#define ENC_DIRECTION         (1 << 13)                               /*!< Mask Encoder direction */
#define ENC_REVERROR          (1 << 14)                               /*!< Flag Encoder revolution error */
#define ENC_CMP               (1 << 15)                               /*!< Flag Encoder comparator match */

#define ENC_3PHASE_ENABLE     (1 << 16)                               /*!< Flag Encoder 3 phase mode enable */
#define ENC_MODE_ENCODE       0                                       /*!< Flag Encoder encoder mode enable */
#define ENC_MODE_SENSOR_EVENT (1 << 17)                               /*!< Flag Encoder sensor mode enable */
#define ENC_MODE_SENSOR_TIME  (1 << 18)                               /*!< Flag Encoder sensor timer mode enable */
#define ENC_MODE_TIMER        ((1 << 17) | (1 << 18))                 /*!< Flag Encoder timer mode enable */

/*! \brief internal storage of different encoder values
 *
 *  This stuct displays the different values that are taken during the encoder run
 *  and be used for calculations.
 */
typedef struct
{
  int32_t FullTurns;
  int32_t ticksBetweenEvents;                                       /*!< Measured clock ticks between two events */
  int16_t event_nr;                                                 /*!< Event number (from 0 to MAX) */
  uint8_t CW;                                                       /*!< Motor direction as determined by encoder block */
  int32_t Theta0;                                                   /*!< Theta of Event 0 */
} EncoderValues;

extern EncoderValues  EncoderData[MAX_CHANNEL];

void ENC_Determine_Omega(uint8_t channel_number);
void ENC_Init           (uint8_t channel_number);

#endif  /* _ENCODER_H_ */
