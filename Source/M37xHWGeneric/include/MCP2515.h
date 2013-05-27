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

#ifndef __MCP2515_H_
#define __MCP2515_H_

#include <stdint.h>

/*! \brief CAN Chip commands
 *
 *  This enumaration reflects the different commands for the CAN Tranceiver
 */
typedef enum 
{
  CAN_RESET,
  CAN_WAIT,
  CAN_INIT,
  CAN_TRANSMIT,
  CAN_RECEIVE,
  CAN_IDLE,
  CAN_TRANSMIT_INIT,
  CAN_WAIT2,
} CAN_COMMAND;

/*! \brief CAN Frame type
 *
 *  This enumaration represents the different CAN Frame types
 */
typedef enum 
{
  STANDARD,
  EXTENDED,
} CAN_FRAMETYPE;

/*! \brief CAN Tranceiver Registers
 *
 */
typedef enum 
{
  DIGITAL_INPUT_REGISTER             = 0x0D,
  DIGITAL_OUTPUT_REGISTER            = 0x0C,
  STATUS_REGISTER                    = 0x0E,
  CONTROL_REGISTER                   = 0x0F,
  TRANSMIT_ERROR_CONTER_REGISTER     = 0x1C,
  RECEIVE_ERROR_CONTER_REGISTER      = 0x1D,
  CNF3_REGISTER                      = 0x28,
  CNF2_REGISTER                      = 0x29,
  CNF1_REGISTER                      = 0x2A,
  INTERRUPT_ENABLE_REGISTER          = 0x2B,
  INTERRUPT_FLAG_REGISTER            = 0x2C,
  ERROR_FLAG_REGISTER                = 0x2D,
  TRANSMIT_BUFFER_0_CONTROL_REGISTER = 0x30,
  TRANSMIT_BUFFER_1_CONTROL_REGISTER = 0x40,
  TRANSMIT_BUFFER_2_CONTROL_REGISTER = 0x50,
  RECEIVE_BUFFER_0_CONTROL_REGISTER  = 0x60,
  RECEIVE_BUFFER_1_CONTROL_REGISTER  = 0x70,
} CAN_REGISTERS;

/*! \brief CAN Tranceiver Operation Modes
 *
 */
typedef enum 
{
  NORMAL_OPERATION_MODE              = 0x00,
  SLEEP_MODE                         = 0x20,
  LOOPBACK_MODE                      = 0x40,
  LISTEN_ONLY_MODE                   = 0x60,
  CONFIGUARTION_MODE                 = 0x80,
} CAN_OPERATION_MODE;

/*! \brief CAN Tranceiver Commands
 *
 */
typedef enum 
{
  WRITE_REGISTER_COMMAND             = 0x02,
  READ_REGISTER_COMMAND              = 0x03,
  BIT_MODIFY_COMMAND                 = 0x05,
  RTS_TXB0_COMMAND                   = 0x81,
  RTS_TXB1_COMMAND                   = 0x82,
  RTS_TXB2_COMMAND                   = 0x84,
  READ_STATUS_COMMAND                = 0xA0,
  RX_STATUS_COMMAND                  = 0xB0,
  RESET_COMMAND                      = 0xC0,
} CAN_COMMANDS;

/*! \brief CAN Tranceiver Interrupts
 *
 */
typedef enum 
{
  STATUS_NO_INTERRUPT                = 0x00,
  STATUS_ERROR_INTERRUPT             = 0x02,
  STATUS_WAKEUP_INTERRUPT            = 0x04,
  STATUS_TXB0_INTERRUPT              = 0x06,
  STATUS_TXB1_INTERRUPT              = 0x08,
  STATUS_TXB2_INTERRUPT              = 0x0A,
  STATUS_RXB0_INTERRUPT              = 0x0C,
  STATUS_RXB1_INTERRUPT              = 0x0E,
} CAN_INTERRUPT;

/*! \brief CAN Tranceiver Interrupt enable
 *
 */
typedef enum 
{
  ERROR_INTERRUPT_ENABLE             = 0x80,
  WAKEUP_INTERRUPT_ENABLE            = 0x40,
  ERRORFLG_INTERRUPT_ENABLE          = 0x20,
  TX2_INTERRUPT_ENABLE               = 0x10,
  TX1_INTERRUPT_ENABLE               = 0x08,
  TX0_INTERRUPT_ENABLE               = 0x04,
  RX1_INTERRUPT_ENABLE               = 0x02,
  RX0_INTERRUPT_ENABLE               = 0x01,
  NO_INTERRUPT_ENABLE                = 0x00,
} CAN_INTERRUPT_ENABLE;

/*! \brief CAN Tranceiver Interrupt flags
 *
 */
typedef enum 
{
  ERROR_INTERRUPT_FLAG               = 0x80,
  WAKEUP_INTERRUPT_FLAG              = 0x40,
  ERRORFLG_INTERRUPT_FLAG            = 0x20,
  TX2_INTERRUPT_FLAG                 = 0x10,
  TX1_INTERRUPT_FLAG                 = 0x08,
  TX0_INTERRUPT_FLAG                 = 0x04,
  RX1_INTERRUPT_FLAG                 = 0x02,
  RX0_INTERRUPT_FLAG                 = 0x01,
} CAN_INTERRUPT_FLAG;

/*! \brief CAN Tranceiver Filter Adress
 *
 */
typedef enum 
{
  FILTER_0_ADDRESS                   = 0x00,
  FILTER_1_ADDRESS                   = 0x04,
  FILTER_2_ADDRESS                   = 0x08,
  FILTER_3_ADDRESS                   = 0x10,
  FILTER_4_ADDRESS                   = 0x14,
  FILTER_5_ADDRESS                   = 0x18,
  MASK_0_ADDRESS                     = 0x20,
  MASK_1_ADDRESS                     = 0x24,
} CAN_ADDRESS;

/*! \brief CAN Tranceiver Filter Mask
 *
 */
typedef enum 
{
  MASK_FILTER_OFF_IDENT              = 0x60,
  MASK_FILTER_EXTENDED_IDENT         = 0x40,
  MASK_FILTER_STANDARD_IDENT         = 0x20,
  MASK_FILTER_BOTH_IDENT             = 0x00,
} CAN_MASK;

/*! \brief CAN Buffer selection
 *
 * Receive and Transmit Buffer selection
 */
typedef enum 
{
  RECEIVEBUFFER_0                    = 0x90,
  RECEIVEBUFFER_1                    = 0x94,
  TRANSMITBUFFER_0                   = 0x40,
  TRANSMITBUFFER_1                   = 0x42,
  TRANSMITBUFFER_2                   = 0x44,
} CAN_BUFFER_SELECT;

/*! \brief CAN Priority settings
 *
 *  PRIORITY_3 is highest priority
 *  PRIORITY_0 is lowest priority 
 */
typedef enum 
{
  TRANSMIT_BUFFER_PRIORITY_3         = 0x03,
  TRANSMIT_BUFFER_PRIORITY_2         = 0x02,
  TRANSMIT_BUFFER_PRIORITY_1         = 0x01,
  TRANSMIT_BUFFER_PRIORITY_0         = 0x00,
} CAN_TRANSMIT_BUFFER_PRIO;

/*! \brief CAN Bitrate 125 kbps at 20 MHz and 16 MHz
 *
 *  Settings for the registers
 */
#define CNF1_0125_20MHz                0x03
#define CNF2_0125_20MHz                0xBA
#define CNF3_0125_20MHz                0x07
#define CNF1_0125_16MHz                0x03
#define CNF2_0125_16MHz                0xB8
#define CNF3_0125_16MHz                0x05
#define CNF1_0125_10MHz                0x01
#define CNF2_0125_10MHz                0xBA
#define CNF3_0125_10MHz                0x07

/*! \brief CAN Bitrate 250 kbps at 20 MHz and 16 MHz
 *
 *  Settings for the registers
 */
#define CNF1_0250_20MHz                0x01
#define CNF2_0250_20MHz                0xBA
#define CNF3_0250_20MHz                0x07
#define CNF1_0250_16MHz                0x01
#define CNF2_0250_16MHz                0xB8
#define CNF3_0250_16MHz                0x05

/*! \brief CAN Bitrate 500 kbps at 20 MHz and 16 MHz
 *
 *  Settings for the registers
 */
#define CNF1_0500_20MHz                0x00
#define CNF2_0500_20MHz                0xBA   
#define CNF3_0500_20MHz                0x07
#define CNF1_0500_16MHz                0x00
#define CNF2_0500_16MHz                0xB8   
#define CNF3_0500_16MHz                0x05

/*! \brief CAN Bitrate 1 Mbps at 20 MHz and 16 MHz
 *
 *  Settings for the registers
 */
#define CNF1_1000_20MHz                0x00
#define CNF2_1000_20MHz                0xA0
#define CNF3_1000_20MHz                0x02
#define CNF1_1000_16MHz                0x00
#define CNF2_1000_16MHz                0x90
#define CNF3_1000_16MHz                0x02

/*! \brief CAN Message
 *
 *  Data structure of a CAN message
 */
typedef struct {
    uint32_t  MsgID;
    uint8_t   MsgType;
    uint8_t   dlc;
    union {
      int32_t l[2];
      int16_t w[4];
      int8_t  b[8];
    } Data;
} can_message ;

uint8_t     MCP2515_ReadRegister            (uint8_t Register);
void        MCP2515_WriteRegister           (uint8_t Register, uint8_t Data);
void        MCP2515_BitModify               (uint8_t Register, uint8_t Mask, uint8_t Data);
void        MCP2515_Reset                   (void);
uint8_t     MCP2515_QuickReadStatus         (void);
uint8_t     MCP2515_QuickReadRXstatus       (void);
void        MCP2515_SetRTS                  (uint8_t RTS);
uint8_t     MCP2515_ReadTransmitErrorCounter(void);
uint8_t     MCP2515_ReadReceiveErrorCounter (void);
uint8_t     MCP2515_ReadErrorFlags          (void);
uint8_t     MCP2515_ReadDigitalInputs       (void);
uint8_t     MCP2515_ReadStatusRegister      (void);
void        MCP2515_WriteControlRegister    (uint8_t Data);
uint8_t     MCP2515_ReadControlRegister     (void);
void        MCP2515_SetDigitalOutputs       (uint8_t Data);
void        MCP2515_SetOperationMode        (uint8_t OperationMode);
void        MCP2515_SetInterruptEnableBit   (uint8_t Interrupt);
void        MCP2515_ClrInterruptEnableBit   (uint8_t Interrupt);
void        MCP2515_ClrInterruptFlagBit     (uint8_t Interrupt);
void        MCP2515_WriteBitTimingRegister  (uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
can_message MCP2515_Read_CAN_Message        (uint8_t ReceiveBuffer);
void        MCP2515_Load_CAN_Message        (uint8_t TransmitBuffer, can_message message);
void        MCP2515_Set_CAN_Filter_Mask     (uint32_t Identifier, uint8_t TypeIdentifier, uint8_t Address);
uint8_t     MCP2515_ReadRCVB0CR             (void) ;
uint8_t     MCP2515_ReadRCVB1CR             (void) ;

#endif /* __MCP2515_H_ */
