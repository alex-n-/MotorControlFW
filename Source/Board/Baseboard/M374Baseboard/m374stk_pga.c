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

#include "FreeRTOS.h"
#include "task.h"

#include BOARD_BOARD_HEADER_FILE
#include BOARD_PGA_HEADER_FILE
#include BOARD_SPI_HEADER_FILE

#include "spi.h"

/*! \brief PGA Init state machine
  *
  * Initialize the state machine of the PGAs
  * Please refer to the Datasheet for more information about state machine
  *
  * @param  device:   Device Number
  *
  * @retval None
*/
static void PGA_InitStateMachine(SPI_DeviceSelect device)
{
  /* Bring PGA state machine to defined state */
  SPI_SelectDevice(device);
  vTaskDelay( 1 / portTICK_RATE_MS);
  SPI_DeselectDevice();
  vTaskDelay( 1 / portTICK_RATE_MS);
}

/*! \brief PGA Set Gain
  *
  * Set the gain for the PGA
  *
  * @param  device: Device Number
  * @param  gain:   Gain enumeration type
  *
  * @retval None
*/
static void PGA_SetGain(SPI_DeviceSelect device, PGA_GAIN gain)
{
  /* Setting the gain */
  SPI_SelectDevice(device);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, GAIN_REGISTER);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, gain);
  SPI_DeselectDevice();
}

/*! \brief PGA Set Channel
  *
  * Select the channel of the PGA
  *
  * @param  device:  Device Number
  * @param  channel: Channel number of the PGA
  *
  * @retval None
*/
void PGA_SetChannel(SPI_DeviceSelect device, PGA_CHANNEL channel)
{
  /* Setting the channel */
  SPI_SelectDevice(device);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, CHANNEL_REGISTER);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, channel);
  SPI_DeselectDevice();
}

/*! \brief PGA Init
  *
  * Initialize the PGA
  *
  * @param  settings:    Settings table
  * @param  nr_settings: Number of settings to perform
  *
  * @retval None
*/
void PGA_Init(PGA_Settings* settings, uint8_t nr_settings)
{
  uint8_t i;

  for(i = 0; i < nr_settings; i++)
  {
    PGA_InitStateMachine(settings[i].device);
    PGA_SetGain(settings[i].device, settings[i].gain);
  }
}
