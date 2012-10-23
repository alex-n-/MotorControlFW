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
#ifdef USE_I2C_MASTER
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "i2c_master_bitbanging.h"

#define SLAVE_ADDR 0x55

unsigned char data[3];

void I2C_LED_Write(uint8_t device_id, uint8_t *byte)
{
    int i=0;
    I2C_WriteByte(1,0,device_id<<1);
    I2C_WriteByte(0,0,1);
    for(i=0;i<3;i++)
      I2C_WriteByte(0,0,*byte++);
    I2C_WriteByte(0,1,3);
    vTaskDelay( 2 / portTICK_RATE_MS );
}

uint16_t I2C_LED_Read(uint8_t device_id)
{
    uint16_t status=0;
    I2C_WriteByte(1,0,(device_id<<1)|DEVICE_READ);
    status |= I2C_ReadByte(0,0)<<8;
    status |= I2C_ReadByte(0,1);
    return status;
}

void I2C_LED_Test(void)
{
  uint8_t i,j;
  
  
  for (;;)
  {
    for (j=0;j<3;j++)
    {
      memset(data,0,sizeof(data));
      for (i=0;i<=15;i++)
      {
        data[j]=i;
        I2C_LED_Write(SLAVE_ADDR,data);
        vTaskDelay( 100 / portTICK_RATE_MS );
        printf("Led:%lx\n",I2C_LED_Read(SLAVE_ADDR));
      }
    }

    memset(data,0,sizeof(data));
    for (i=0;i<=15;i++)
    {
      data[0]=i;
      data[1]=i;
      I2C_LED_Write(SLAVE_ADDR,data);
      vTaskDelay( 100 / portTICK_RATE_MS );
      printf("Led:%lx\n",I2C_LED_Read(SLAVE_ADDR));
    }

    memset(data,0,sizeof(data));
    for (i=0;i<=15;i++)
    {
      data[0]=i;
      data[2]=i;
      I2C_LED_Write(SLAVE_ADDR,data);
      vTaskDelay( 100 / portTICK_RATE_MS );
      printf("Led:%lx\n",I2C_LED_Read(SLAVE_ADDR));
    }

    memset(data,0,sizeof(data));
    for (i=0;i<=15;i++)
    {
      data[1]=i;
      data[2]=i;
      I2C_LED_Write(SLAVE_ADDR,data);
      vTaskDelay( 100 / portTICK_RATE_MS );
      printf("Led:%lx\n",I2C_LED_Read(SLAVE_ADDR));
    }
    
    memset(data,0,sizeof(data));
    for (i=0;i<=15;i++)
    {
      data[0]=i;
      data[1]=i;
      data[2]=i;
      I2C_LED_Write(SLAVE_ADDR,data);
      vTaskDelay( 100 / portTICK_RATE_MS );
      printf("Led:%lx\n",I2C_LED_Read(SLAVE_ADDR));
    }
  }
    
}


void LED_Toggle(uint8_t led)
{
}

void LED_SetState(uint8_t led, uint8_t state)
{
}

uint8_t LED_GetState(uint8_t led)
{
}

void LED_Init(void)
{
}


#endif