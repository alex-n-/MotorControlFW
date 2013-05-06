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

#ifdef BOARD_PWR_HEADER_FILE_0
#include BOARD_PWR_HEADER_FILE_0
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_0 */

#ifdef BOARD_PWR_HEADER_FILE_1
#include BOARD_PWR_HEADER_FILE_1
#include "pwr_undefine.h"
#endif /* BOARD_PWR_HEADER_FILE_1 */

#ifdef USE_TEMPERATURE_CONTROL
typedef struct {
  uint16_t  adc;
  int8_t    temperature;
} temp_table;

#ifdef VISHAY_NTCLE100E3103JB0
/* Temperature Table for Vishay NTCLE100E3103JB0 */
static const temp_table temperature[]={
                                  {0xF87,-40},
                                  {0xF5B,-35},
                                  {0xF22,-30},
                                  {0xED9,-25},
                                  {0xE7E,-20},
                                  {0xE0F,-15},
                                  {0xD89,-10},
                                  {0xCEE, -5},
                                  {0xC3D,  0},
                                  {0xB78,  5},
                                  {0xAA4, 10},
                                  {0x9C6, 15},
                                  {0x8E2, 20},
                                  {0x800, 25},
                                  {0x723, 30},
                                  {0x652, 35},
                                  {0x590, 40},
                                  {0x4DE, 45},
                                  {0x43D, 50},
                                  {0x3AE, 55},
                                  {0x330, 60},
                                  {0x2C2, 65},
                                  {0x263, 70},
                                  {0x210, 75},
                                  {0x1C9, 80},
                                  {0x18C, 85},
                                  {0x157, 90},
                                  {0x12A, 95},
                                  {0x104,100},
                                  {0x0E3,105},
                                  {0x0C6,110},
                                  {0x0AE,115},
                                  {0x098,120},
                                  {0x086,125},
                                };
#endif /* VISHAY_NTCLE100E3103JB0 */

#ifdef MITSUBISHI_PS219B4_AS
/* Temperature Table for Mitsubishi PS219B4-AS */
static const temp_table temperature[]={
                                    {0x00F,-15},
                                    {0x029,-10},
                                    {0x043, -5},
                                    {0x05E,  0},
                                    {0x078,  5},
                                    {0x092, 10},
                                    {0x0AC, 15},
                                    {0x0C7, 20},
                                    {0x0E1, 25},
                                    {0x0FB, 30},
                                    {0x115, 35},
                                    {0x130, 40},
                                    {0x14A, 45},
                                    {0x164, 50},
                                    {0x17E, 55},
                                    {0x199, 60},
                                    {0x1B3, 65},
                                    {0x1CD, 70},
                                    {0x1E7, 75},
                                    {0x202, 80},
                                    {0x21C, 85},
                                    {0x236, 90},
                                    {0x250, 95},
                                    {0x26B,100},
                                    {0x285,105},
                                    {0x29F,110},
                                    {0x2BA,115},
                                    {0x2D4,120},
                                  };
#endif /* MITSUBISHI_PS219B4_AS */
																	
#endif /*USE_TEMPERATURE_CONTROL */
