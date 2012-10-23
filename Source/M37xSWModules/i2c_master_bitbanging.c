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

/* This source code is derived from the example code on http://en.wikipedia.org/wiki/I%C2%B2C */


#include "config.h"

#ifdef USE_I2C_MASTER

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "i2c_master_bitbanging.h"
#include "tmpm372_tmrb.h"
#include "tmpm372_gpio.h"

static uint8_t started = 0;

static const GPIO_InitTypeDef portConfigI2C =
{
  GPIO_IO_MODE_NONE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_ENABLE,
  GPIO_PULLDOWN_DISABLE,
};

static void I2C_delay(void)
{
  vTaskDelay( 1 / portTICK_RATE_MS );
}

  // Set SCL as input and return current level of line, 0 or 1
static uint8_t read_SCL(void)
{
  GPIO_SetInput(I2C_PORT, I2C_SCL);
  return GPIO_ReadDataBit(I2C_PORT, I2C_SCL);  
}

// Set SDA as input and return current level of line, 0 or 1
static uint8_t read_SDA(void)
{
  GPIO_SetInput(I2C_PORT, I2C_SDA);
  return GPIO_ReadDataBit(I2C_PORT, I2C_SDA);
}

// Actively drive SCL signal low
static void clear_SCL(void)
{
  GPIO_SetOutput(I2C_PORT, I2C_SCL);
  GPIO_WriteDataBit(I2C_PORT, I2C_SCL, 0);
}

// Actively drive SDA signal low
static void clear_SDA(void)
{
  GPIO_SetOutput(I2C_PORT, I2C_SDA);
  GPIO_WriteDataBit(I2C_PORT, I2C_SDA, 0);
}

static void arbitration_lost(void)
{
  dprintf("arbitration_lost\n");
}
 
static void i2c_start_cond(void)
{
  if (started)                                            /* if started, do a restart cond */
  { 
    read_SDA();                                           /* set SDA to 1 */
    I2C_delay();
    while (read_SCL() == 0)
    {
                                                          /* Clock stretching */
                                                          /* You should add timeout to this loop */
    }
    I2C_delay();                                          /* Repeated start setup time */
  }
  if (read_SDA() == 0)
    arbitration_lost();
  
  clear_SDA();                                            /* SCL is high, set SDA from 1 to 0. */
  I2C_delay();
  clear_SCL();
  started = 1;
}
 
static void i2c_stop_cond(void)
{
  clear_SDA();                                            /* set SDA to 0 */
  I2C_delay();
  
  while (read_SCL() == 0)                                 /* Clock stretching */
  {
                                                          /* You should add timeout to this loop. */
  }
  // Stop bit setup time, minimum 4us
  I2C_delay();
  // SCL is high, set SDA from 0 to 1
  if (read_SDA() == 0)
    arbitration_lost();

  I2C_delay();
  started = 0;
}
 
// Write a bit to I2C bus
static void i2c_write_bit(uint8_t bit)
{
  if (bit)
    read_SDA();
  else
    clear_SDA();

  I2C_delay();
  while (read_SCL() == 0)                                 /* Clock stretching */
  {
                                                          /* You should add timeout to this loop */
  }
  
  if (bit && read_SDA() == 0)                             /* SCL is high, now data is valid */
    arbitration_lost();                                   /* If SDA is high, check that nobody else is driving SDA */

  I2C_delay();
  clear_SCL();
}
 
// Read a bit from I2C bus
static uint8_t i2c_read_bit(void)
{
  uint8_t bit;
  
  read_SDA();                                             /* Let the slave drive data */
  I2C_delay();
  while (read_SCL() == 0)                                 /* Clock stretching */
  {
                                                          /* You should add timeout to this loop */
  }

  bit = read_SDA();                                       /* SCL is high, now data is valid */
  I2C_delay();
  clear_SCL();
  return bit;
}

// Write a byte to I2C bus. Return 0 if ack by the slave.
uint8_t I2C_WriteByte(uint8_t send_start, uint8_t send_stop, uint8_t byte)
{
  uint8_t bit;
  uint8_t nack;
  
  if (send_start)
    i2c_start_cond();

  for (bit = 0; bit < 8; bit++) {
    i2c_write_bit((byte & 0x80) != 0);
    byte <<= 1;
  }

  nack = i2c_read_bit();

  if (send_stop)
    i2c_stop_cond();

  return nack;
}
 
// Read a byte from I2C bus
uint8_t I2C_ReadByte(uint8_t nack, uint8_t send_stop)
{
  uint8_t byte = 0;
  uint8_t bit;
  
  for (bit = 0; bit < 8; bit++)
    byte = (byte << 1) | i2c_read_bit();

  i2c_write_bit(nack);

  if (send_stop)
    i2c_stop_cond();

  return byte;
}

void I2C_Init(void)
{
  GPIO_Init(I2C_PORT,
            I2C_SCL,
            &portConfigI2C);
}

#endif /* USE_I2C_MASTER */
