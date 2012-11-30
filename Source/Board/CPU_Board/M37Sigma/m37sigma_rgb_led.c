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

#include <string.h>

#include "config.h"
#ifdef USE_RGB_LED
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "i2c_master_bitbanging.h"
#include "board.h"

#define SLAVE_ADDR 0x55

static uint16_t rgb_value;
static uint16_t rgb_value_old;
static uint16_t rgb_value_store;

/*! \brief I2C RGB Led Write
  *
  * Write data via I2C to LED
  *
  * @param  device_id: I2C Device Id
  * @param  *byte:     Pointer to register bytes
  *
  * @retval None
*/
static void i2c_led_write(uint8_t device_id, uint8_t *byte)
{
    int i=0;
    I2C_WriteByte(1,0,device_id<<1);
    I2C_WriteByte(0,0,1);
    for(i=0;i<3;i++)
      I2C_WriteByte(0,0,*byte++);
    I2C_WriteByte(0,1,3);
    vTaskDelay( 2 / portTICK_RATE_MS );
}

#if 0
/*! \brief I2C RGB Led Read
  *
  * Read data via I2C from LED
  *
  * @param  device_id: I2C Device Id
  *
  * @retval register values
*/
static uint16_t i2c_led_read(uint8_t device_id)
{
    uint16_t status=0;
    I2C_WriteByte(1,0,(device_id<<1)|DEVICE_READ);
    status |= I2C_ReadByte(0,0)<<8;
    status |= I2C_ReadByte(0,1);
    return status;
}
#endif

/*! \brief Send LED
  *
  * Update RGB Value to LED
  *
  * @retval None
*/
static void send_led(void)
{
  unsigned char   rgb_data[3];
    
  rgb_data[0] = (rgb_value >> 8) & 0xf;
  rgb_data[1] = (rgb_value >> 4) & 0xf;
  rgb_data[2] = (rgb_value >> 0) & 0xf;
  
  i2c_led_write(SLAVE_ADDR,rgb_data);
  
  rgb_value_old = rgb_value;
}

/*! \brief Set RGB Value
  *
  * @retval None
*/
void RGB_LED_SetValue(uint16_t value)
{
  rgb_value=value;
}

/*! \brief Toggle RGB part
  *
  * Toggle bits of RGB value
  *
  * @retval None
*/
void RGB_LED_ToggleValue(uint16_t value)
{

  if ((rgb_value & value) == 0)
    rgb_value |= value;
  else
    rgb_value &= ~value;
}

/*! \brief RGB Store value
  *
  * Store actual RGB value
  *
  * @retval None
*/
void RGB_LED_StoreValue(void)
{
  rgb_value_store=rgb_value;
}

/*! \brief RGB restore value
  *
  * Restore actual RGB value to earlier saved one
  *
  * @retval None
*/
void RGB_LED_RestoreValue(void)
{
  rgb_value=rgb_value_store;
}

/*! \brief RGB get value
  *
  * Get the actual RGB value
  *
  * @retval None
*/
uint16_t RGB_LED_GetValue(uint16_t value)
{
  return (rgb_value);
}

/*! \brief RGB Value transmit task
  *
  * Update the RGB value on a regular base
  *
  * @param  pvParameters: Mandatory for FreeRTOS
  *
  * @retval None
*/
void RGBLEDTransmitTask(void* pvParameters)
{
 
  while (INIT_Done==0)                                                          /* wailt until HW setup has finished */
    vTaskDelay( 100 / portTICK_RATE_MS );

  for(;;)
  {
    if (rgb_value!=rgb_value_old)
      send_led();

    vTaskDelay( 100 / portTICK_RATE_MS );
  }
}    

/*! \brief RGB LED Init
  *
  * Initialize the RGB LED access
  *
  * @retval None
*/
void RGB_LED_Init (void)
{
  if ( xTaskCreate(RGBLEDTransmitTask,                                          /* Create Turn Task Channel 1*/
                   (signed char*) "RGB",
                   60,
                   (void*)0,
                   0,
                   NULL) != pdPASS)
    dprintf("Can't create RGBLEDTransmit Task\n");
}
 

#endif
