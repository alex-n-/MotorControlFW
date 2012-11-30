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

#include "config.h"
#if (defined BOARD_HITEX_M370 && defined USE_LCD)

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include BOARD_LCD_HEADER_FILE
#include BOARD_SPI_HEADER_FILE
#include TMPM_GPIO_HEADER_FILE

#define ROW_0_OFFSET 0x00
#define ROW_1_OFFSET 0x40
#define ROW_2_OFFSET 0x14
#define ROW_3_OFFSET 0x54

/*! \brief LED Transmit Data
  *
  * Transmit data to the LCD
  *
  * @param  cmdType: Command type
  * @param  data:    Data
  *
  * @retval None
*/
static void LCD_TransmitData(uint8_t cmdType, uint8_t data)
{
  /* Put all display pins low */
  GPIO_WriteData(LCD_INSTRUCTION_PINS, (uint8_t)~LCD_INSTRUCTION_MASK);
  GPIO_WriteData(LCD_DATA_PINS,        (uint8_t)~LCD_DATA_MASK);

  switch(cmdType)
  {
  case LCD_CMD_INSTR:
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_RDWR,      GPIO_BIT_VALUE_0);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_DATA_INST, GPIO_BIT_VALUE_0);
    GPIO_WriteData(LCD_DATA_PINS, data);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_ENABLE,    GPIO_BIT_VALUE_1);
    vTaskDelay(1 / portTICK_RATE_MS);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_ENABLE,    GPIO_BIT_VALUE_0);
    break;

  case LCD_CMD_DATA:
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_RDWR,      GPIO_BIT_VALUE_0);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_DATA_INST, GPIO_BIT_VALUE_1);
    GPIO_WriteData(LCD_DATA_PINS, data);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_ENABLE,    GPIO_BIT_VALUE_1);
    vTaskDelay(1 / portTICK_RATE_MS);
    GPIO_WriteDataBit(LCD_INSTRUCTION_PINS, LCD_PIN_ENABLE,    GPIO_BIT_VALUE_0);
    break;
  }
}

/*! \brief LED Set cursor position
  *
  * Place the cursor to row/column
  *
  * @param  row:    Row
  * @param  column: Column
  *
  * @retval None
*/
void LCD_SetCursorPosition(uint8_t row, uint8_t column)
{
  unsigned char address = 0;

  /*
   * The order of the display lines is a little cruel ;
   * Row 0: Address 0x00 - 0x13 ;
   * Row 1: Address 0x40 - 0x53 ;
   * Row 2: Address 0x14 - 0x27 ;
   * Row 3: Address 0x54 - 0x67
   */
  switch(row)
  {
  case 0:
    address = ROW_0_OFFSET + column;
    break;
  case 1:
    address = ROW_1_OFFSET + column;
    break;
  case 2:
    address = ROW_2_OFFSET + column;
    break;
  case 3:
    address = ROW_3_OFFSET + column;
    break;
  }

  LCD_TransmitData(LCD_CMD_INSTR, (LCD_INS_SET_ADDR | address));
}

/*! \brief LCD Clear
  *
  * Clear the LCD display
  *
  * @retval None
*/
void LCD_Clear(void)
{
  SPI_SelectDevice(SPI_DEVICE_LCD);
  LCD_TransmitData(LCD_CMD_INSTR, LCD_CFG_DISP_CLEAR);
  SPI_DeselectDevice();
}

/*! \brief Enable Cursor Blink
  *
  * Let the cursor blink
  *
  * @retval None
*/
void LCD_EnableCursorBlink(void)
{
  SPI_SelectDevice(SPI_DEVICE_LCD);
  LCD_TransmitData(LCD_CMD_INSTR,  LCD_CFG_ON_GENERAL
                                 | LCD_CFG_DISP_ON
                                 | LCD_CFG_CURSOR_ON
                                 | LCD_CFG_CURSOR_BLINK);
  SPI_DeselectDevice();
}

/*! \brief Disable Cursor Blink
  *
  * Stop the cursor blink
  *
  * @retval None
*/
void LCD_DisableCursorBlink(void)
{
  SPI_SelectDevice(SPI_DEVICE_LCD);
  LCD_TransmitData(LCD_CMD_INSTR,  LCD_CFG_ON_GENERAL
                                 | LCD_CFG_DISP_ON);
  SPI_DeselectDevice();
}

/*! \brief LCD Display Text
  *
  * @param  row:    Row to start
  * @param  column: Column to start
  * @param  text:   Text to display
  *
  * @retval None
*/
void LCD_DisplayText(uint8_t row, uint8_t column, char* text)
{
  SPI_SelectDevice(SPI_DEVICE_LCD);
  LCD_SetCursorPosition(row, column);

  while(*text != 0)
  {
    LCD_TransmitData(LCD_CMD_DATA, *text++);
    if(++column >= LCD_COLUMNS)                                                 /* Don't write over end of line */
      break;
  }

  SPI_DeselectDevice();
}

/*! \brief LCD Display Value
  *
  * @param  row:     Row to start
  * @param  column:  Column to start
  * @param  base:    Base of number display
  * @param  digits:  Digits to display
  * @param  value:   Value to display
  *
  * @retval None
*/
void LCD_DisplayValue(uint8_t row, uint8_t column, uint8_t base, uint8_t digits, uint32_t value)
{
  char    convert[12];
  char    len;
  uint8_t string[14];
  
  memset (string,0,sizeof(string));

  switch(base)
  {
  case LCD_VALUE_DECIMAL:
    len = sprintf(convert, "%010ld", value);
    memcpy(string, &convert[10 - digits], len - 11 + digits + 1);
    break;

  case LCD_VALUE_HEX:
    len = sprintf(convert, "%08lx", value);
    memcpy(&string[2], &convert[8 - digits], len - 9 + digits + 1);
    string[0] = '0';
    string[1] = 'x';
    break;

  case LCD_VALUE_OCTAL:
    len = sprintf(convert, "%011o", (unsigned int)value);
    memcpy(&string[2], &convert[11 - digits], len - 12 + digits + 1);
    string[0] = '0';
    string[1] = 'o';
    break;
  }

  LCD_DisplayText(row, column, (char*)string);
}

/*! \brief LCD Init
  *
  * Initialize the LCD Display
  *
  * @retval None
*/
void LCD_Init(void)
{
  vTaskDelay( 200 / portTICK_RATE_MS );                                         /* 200 ms needed for powering up the display */
  SPI_SelectDevice(SPI_DEVICE_LCD);
  LCD_TransmitData(LCD_CMD_INSTR,  LCD_CFG_FUNC_SET 
                                 | LCD_CFG_DATALEN_8
                                 | LCD_CFG_2LINES
                                 | LCD_CFG_DISP_ON);
  LCD_TransmitData(LCD_CMD_INSTR,  LCD_CFG_ON_GENERAL
                                 | LCD_CFG_DISP_ON);
  LCD_TransmitData(LCD_CMD_INSTR, LCD_CFG_DISP_CLEAR);
  LCD_TransmitData(LCD_CMD_INSTR,  LCD_CFG_ENRY_GENERL
                                 | LCD_CFG_INC_MODE);
  LCD_Clear();
  SPI_DeselectDevice();
}

#endif
