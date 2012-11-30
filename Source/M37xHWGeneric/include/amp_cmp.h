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

#ifndef _AMPCMP_H_
#define _AMPCMP_H_

/*! \brief Gain selection of internal aplifier/comperator. 
 *
 *  This enumaration is for selecting the different gain possibilities.
 */
typedef enum
{
  GAIN_1_5,                                                           /*!< Gain of +1.5 */
  GAIN_2_5,                                                           /*!< Gain of +2.5 */
  GAIN_3_0,                                                           /*!< Gain of +3.0 */
  GAIN_3_5,                                                           /*!< Gain of +3.5 */
  GAIN_4_0,                                                           /*!< Gain of +4.0 */
  GAIN_6_0,                                                           /*!< Gain of +6.0 */
  GAIN_8_0,                                                           /*!< Gain of +8.0 */
  GAIN_10_0,                                                          /*!< Gain of +10.0 */
} M370_GAIN;

/* Gain as number multiplied by 10 */
static const unsigned int gaintable[]={
  15,
  25,
  30,
  35,
  40,
  60,
  80,
  100,
};

/*! \brief Channel selection of internal aplifier/comperator. 
 *
 *  This enumaration is for selecting the different channel possibilities.
 */
typedef enum
{
  CHANNEL_A,                                                          /*!< Channel A */
  CHANNEL_B,                                                          /*!< Channel B */
  CHANNEL_C,                                                          /*!< Channel C */
  CHANNEL_D,                                                          /*!< Channel D */
} M370_AMPCMP_CHANNEL;

#define CMP_ENABLE    0x01                                            /*!< Enable the comperator */
#define CMP_OPAMP_OUT 0x02                                            /*!< Enable the output */
#define AMP_ENABLE    0x01                                            /*!< Enable the amplifier */

void AMPCMP_setup(M370_AMPCMP_CHANNEL channel_number, M370_GAIN gain);

#endif
