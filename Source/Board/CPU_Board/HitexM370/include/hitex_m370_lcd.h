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

#ifndef _LCD_H_
#define _LCD_H_

#include <stdint.h>

/*
 * LCM-S02004DSx Display ;
 * with Samsung S6A0069 Display Driver
 */
#define LCD_COLUMNS           20
#define LCD_ROWS              4

/* LCD Output Pins */
#define LCD_DATA_PINS         GPIO_PE
#define LCD_INSTRUCTION_PINS  GPIO_PA
#define LCD_DATA_MASK         GPIO_PE_OUTPUT_BIT_ALL
#define LCD_INSTRUCTION_MASK  (GPIO_BIT_0 | GPIO_BIT_1 | GPIO_BIT_2)

/* LCD ctrl pins */
#define LCD_PIN_DATA_INST     GPIO_BIT_0  /* GPIO Pin 0 - select between data / instructions */
#define LCD_PIN_RDWR          GPIO_BIT_1  /* GPIO Pin 1 - select between read / write */
#define LCD_PIN_ENABLE        GPIO_BIT_2  /* GPIO Pin 2 - enable latch in */

/* LCD ctrl commands */
#define LCD_CMD_INSTR         0x00        /* Instruction type transmission */
#define LCD_CMD_DATA          0x01        /* Data type transmission */

/* LCD function set */
#define LCD_CFG_FUNC_SET      0x20        /* This is a function set instruction */
#define LCD_CFG_DATALEN_8     0x10        /* Data are sending as 8-bits data */
#define LCD_CFG_2LINES        0x08        /* Display has 2 lines */
#define LCD_CFG_DISP_ON       0x04        /* Config for Display on */

/* LCD On/Off Control */
#define LCD_CFG_ON_GENERAL    0x08        /* Has to be set to 1 */
#define LCD_CFG_DISP_ON       0x04        /* Display on bit */
#define LCD_CFG_CURSOR_ON     0x02        /* Switch Cursor on */
#define LCD_CFG_CURSOR_BLINK  0x01        /* Cursor blinking */

/* Display Clear */
#define LCD_CFG_DISP_CLEAR    0x01        /* Clears the display */

/* Display entry mode */
#define LCD_CFG_ENRY_GENERL   0x04        /* Has to be set to 1 */
#define LCD_CFG_INC_MODE      0x02        /* Increment Mode */
#define LCD_CFG_SHFT_MODE     0x01        /* Entire shift on */

/* Display instructions */
#define LCD_INS_CLEAR_DISPLAY 0x01
#define LCD_INS_CURSOR_HOME   0x02        /* Set cursor home */
#define LCD_INS_SHIFT_CURSORL 0x10        /* Shifts cursor 1 position left */
#define LCD_INS_SHIFT_CURSORR 0x14        /* Shifts cursor 1 position right */
#define LCD_INS_SET_ADDR      0x80        /* Sets DDRAM Address */

#define LCD_VALUE_DECIMAL     0x00        /* Display value as decimal */
#define LCD_VALUE_HEX         0x01        /* Display value as hex */
#define LCD_VALUE_OCTAL       0x02        /* Display value as octal */

void  LCD_Init              (void);
void  LCD_Clear             (void);
void  LCD_SetCursorPosition (uint8_t row, uint8_t column);
void  LCD_EnableCursorBlink (void);
void  LCD_DisableCursorBlink(void);
void  LCD_DisplayText       (uint8_t row, uint8_t column, char* text);
void  LCD_DisplayValue      (uint8_t row, uint8_t column, uint8_t base, uint8_t digits, uint32_t value);

#endif /* _LCD_H_ */
