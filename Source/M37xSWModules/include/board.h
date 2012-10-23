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

#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdint.h>
#include "config.h"
#include TMPM_ADC_HEADER_FILE

extern uint8_t  BoardRevision;
extern uint8_t  INIT_Done;
extern uint32_t T0;

extern const PMD_TrgProgINTTypeDef TrgProgINT_3ShuntA;
extern const PMD_TrgProgINTTypeDef TrgProgINT_3ShuntB;
extern const PMD_TrgProgINTTypeDef TrgProgINT_1ShuntA;
extern const PMD_TrgProgINTTypeDef TrgProgINT_1ShuntB;
extern const PMD_TrgProgINTTypeDef TrgProgINT_2SensorB;
extern const PMD_TrgTypeDef PMDTrigger0_3Phase;
extern const PMD_TrgTypeDef PMDTrigger1_3Phase;
extern const PMD_TrgTypeDef PMDTrigger2_3Phase;
extern const PMD_TrgTypeDef PMDTrigger3_3Phase;
extern const PMD_TrgTypeDef PMDTrigger4_3Phase;
extern const PMD_TrgTypeDef PMDTrigger5_3Phase;
extern const PMD_TrgTypeDef PMDTrigger0_1Phase;
extern const PMD_TrgTypeDef PMDTrigger1_1Phase;
extern const PMD_TrgTypeDef PMDTrigger0_2Phase;
extern const PMD_TrgTypeDef PMDTrigger1_2Phase;

void BOARD_SetupHW(void);
void BOARD_SetupHW2(void);

#endif /* _BOARD_H_ */
