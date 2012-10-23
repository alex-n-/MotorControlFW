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
#ifdef USE_CAN

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "MCP2515.h"
#include "spi.h"
#include BOARD_BOARD_HEADER_FILE
#include BOARD_SPI_HEADER_FILE

extern xSemaphoreHandle CanRXSemaphore;

/**
  * @brief  function to read one register from MCP2515
  * @param  Register address of the register to be read
  *         see MCP2515.h all defines with ..._REGISTER
  * @retval SPI_Rx_Buffer[0] read register
  */
uint8_t MCP2515_ReadRegister(uint8_t Register)
{
  unsigned char value;
  
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, READ_REGISTER_COMMAND);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Register);
  SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  
  value=SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  SPI_DeselectDevice();
  return value;
}

/**
  * @brief  function to write one register into MCP2515
  * @param  Register address of the register to be written
  *         see MCP2515.h all defines with ..._REGISTER
  * @param  Data for the register
  * @retval None
  */
void MCP2515_WriteRegister(uint8_t Register, uint8_t Data)
{
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, WRITE_REGISTER_COMMAND);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Register);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Data);
  SPI_DeselectDevice();
}

/**
  * @brief  function for setting or clearing individual bits in a register
  * @param  Register address of the register to be midified
  *         see MCP2515.h all defines with ..._REGISTER
  *         see datasheet to determine which registers allow the use of this command
  * @param  Mask Bits = 1 in the mask will allow a bit in the register to change
  * @param  Data determines what value the midified bits in the register will be changed to
  * @retval None
  */
void MCP2515_BitModify(uint8_t Register, uint8_t Mask, uint8_t Data)
{
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, BIT_MODIFY_COMMAND);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Register);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Mask);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, Data);
  SPI_DeselectDevice();
}

/**
  * @brief  function to re-initalize the internal registers and set the Configuration mode
  * @param  None
  * @retval None
  */
void MCP2515_Reset()
{
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, RESET_COMMAND);
  SPI_DeselectDevice();
}

/**
  * @brief  function to read status bits with a single instruction access
  * @param  None
  * @retval status register
  */
uint8_t MCP2515_QuickReadStatus()
{
  unsigned char value;
  
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, READ_STATUS_COMMAND);
  SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  
  value=SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  SPI_DeselectDevice();
  return value;  
}

/**
  * @brief  function to read RX-status bits with a single instruction access
  * @param  None
  * @retval RX status register
  */
uint8_t MCP2515_QuickReadRXstatus()
{
  unsigned char value;
  
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, RX_STATUS_COMMAND);
  SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  
  value=SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  SPI_DeselectDevice();
  return value;
}

/**
  * @brief  function to initiate message transmission for one or more of the transmit buffers
  * @param  RTS command for transmission of one buffer use RTS_TXB0_COMMAND, RTS_TXB1_COMMAND
  *         or RTS_TXB2_COMMAND, to initiate the transmission of more the one buffer you can
  *         or the RTS commands
  * @retval None
  */
void MCP2515_SetRTS(uint8_t RTS)
{
  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, RTS);
  SPI_DeselectDevice();
}

/**
  * @brief  function to read the transmission error counter
  * @param  None
  * @retval transmission error counter
  */
uint8_t MCP2515_ReadTransmitErrorCounter()
{
  return MCP2515_ReadRegister(TRANSMIT_ERROR_CONTER_REGISTER);
}

/**
  * @brief  function to read the receive error counter
  * @param  None
  * @retval receive error counter
  */
uint8_t MCP2515_ReadReceiveErrorCounter()
{
  return MCP2515_ReadRegister(RECEIVE_ERROR_CONTER_REGISTER);
}

/**
  * @brief  function to read the error flags
  * @param  None
  * @retval byte with error flags
  */
uint8_t MCP2515_ReadErrorFlags()
{
  return MCP2515_ReadRegister(ERROR_FLAG_REGISTER);
}

/**
  * @brief  function to read the TXnRTS pin status
  * @param  None
  * @retval byte with TXnRTS pin status
  *         Bit 0 TX0RTS
  *         Bit 1 TX1RTS
  *         Bit 2 TX2RTS
  */
uint8_t MCP2515_ReadDigitalInputs()
{
  return MCP2515_ReadRegister(DIGITAL_INPUT_REGISTER);
}

/**
  * @brief  function to read the status register
  * @param  None
  * @retval status register with Operation Mode bits and interrupt flags
  */
uint8_t MCP2515_ReadStatusRegister()
{
  return MCP2515_ReadRegister(STATUS_REGISTER);
}


uint8_t MCP2515_ReadRCVB0CR()
{
  return MCP2515_ReadRegister(RECEIVE_BUFFER_0_CONTROL_REGISTER);
}  

uint8_t MCP2515_ReadRCVB1CR()
{
  return MCP2515_ReadRegister(RECEIVE_BUFFER_1_CONTROL_REGISTER);
}  

/**
  * @brief  function to write the control register
  * @param  Data to setup the clock output pin
  *                           operation mode
  *                           One shot mode
  *                           abort all pending transmissions
  * @retval None
  */
void MCP2515_WriteControlRegister(uint8_t Data)
{
  MCP2515_WriteRegister(CONTROL_REGISTER, Data);
}

/**
  * @brief  function to read the control register
  * @param  None
  * @retval control register (see MCP2515_WriteControlRegister)
  */
uint8_t MCP2515_ReadControlRegister()
{
  return MCP2515_ReadRegister(CONTROL_REGISTER);
}

/**
  * @brief  function to set the RXnBF pins in output mode and state
  *         If pins are used as interrupt the function has no effect.
  * @param  Data Bit 0 state of RX0BF pin
  *              Bit 1 state of RX1BF pin
  * @retval None
  */
void MCP2515_SetDigitalOutputs(uint8_t Data)
{
  MCP2515_WriteRegister(DIGITAL_OUTPUT_REGISTER, (Data << 4) + 0x0C);
}

/**
  * @brief  function to set the Operation Mode of MCP2515
  * @param  OperationMode can be one of the following values
  *         NORMAL_OPERATION_MODE
  *         SLEEP_MODE
  *         LOOPBACK_MODE
  *         LISTEN_ONLY_MODE
  *         CONFIGUARTION_MODE
  * @retval None
  */
void MCP2515_SetOperationMode(uint8_t OperationMode)
{
  uint8_t value;
  value=MCP2515_ReadControlRegister();
  value=MCP2515_ReadControlRegister();
  MCP2515_WriteControlRegister((value & 0x1F) + OperationMode);
}

/**
  * @brief  function to enable interrupt sources to drive the INT pin to low
  * @param  Interrupt can be one of the following values
  *         ERROR_INTERRUPT_ENABLE
  *         WAKEUP_INTERRUPT_ENABLE
  *         ERRORFLG_INTERRUPT_ENABLE
  *         TX2_INTERRUPT_ENABLE
  *         TX1_INTERRUPT_ENABLE
  *         TX0_INTERRUPT_ENABLE
  *         RX1_INTERRUPT_ENABLE
  *         RX0_INTERRUPT_ENABLE
  *         The sources can be ored.
  * @retval None
  */
void MCP2515_SetInterruptEnableBit(uint8_t Interrupt)
{
  MCP2515_BitModify(INTERRUPT_ENABLE_REGISTER, Interrupt, Interrupt);
}

/**
  * @brief  function to disable interrupt sources to drive the INT pin to low
  * @param  Interrupt can be one of the following values
  *         ERROR_INTERRUPT_ENABLE
  *         WAKEUP_INTERRUPT_ENABLE
  *         ERRORFLG_INTERRUPT_ENABLE
  *         TX2_INTERRUPT_ENABLE
  *         TX1_INTERRUPT_ENABLE
  *         TX0_INTERRUPT_ENABLE
  *         RX1_INTERRUPT_ENABLE
  *         RX0_INTERRUPT_ENABLE
  *         The sources can be ored.
  * @retval None
  */
void MCP2515_ClrInterruptEnableBit(uint8_t Interrupt)
{
  MCP2515_BitModify(INTERRUPT_ENABLE_REGISTER, Interrupt, Interrupt^0xFF);
}

/**
  * @brief  function to clear interrupt flags
  * @param  Interrupt can be one of the following values
  *         ERROR_INTERRUPT_FLAG
  *         WAKEUP_INTERRUPT_FLAG
  *         ERRORFLG_INTERRUPT_FLAG
  *         TX2_INTERRUPT_FLAG
  *         TX1_INTERRUPT_FLAG
  *         TX0_INTERRUPT_FLAG
  *         RX1_INTERRUPT_FLAG
  *         RX0_INTERRUPT_FLAG
  *         The sources can be ored.
  * @retval None
  */
void MCP2515_ClrInterruptFlagBit(uint8_t Interrupt)
{
  MCP2515_BitModify(INTERRUPT_FLAG_REGISTER, Interrupt, Interrupt^0xFF);
}

/**
  * @brief  function to setup the timing of CAN bus
  * @param  cnf1 value for CNF1 register
  * @param  cnf2 value for CNF2 register
  * @param  cnf3 value for CNF3 register
  *         If the clock ist 16 MHz or 20 MHz please use the defines CNFx_yyyy_zzMHz
  *         x    numer of register 1..3
  *         yyyy bittiming in kHz
  *         zz   clock of MCP2515 (16 MHz or 20 MHz)
  * @retval None
  */
void MCP2515_WriteBitTimingRegister(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3)
{
  MCP2515_WriteRegister(CNF1_REGISTER, cnf1);
  MCP2515_WriteRegister(CNF2_REGISTER, cnf2);
  MCP2515_WriteRegister(CNF3_REGISTER, cnf3);
}


/**
  * @brief  function to read a complete received CAN message
  * @param  ReceiveBuffer one of the values RECEIVEBUFFER_0 or RECEIVEBUFFER_1
  * @retval SPI_Rx_Buffer[]
  *         SPI_Rx_Buffer[0] - SIDH
  *         SPI_Rx_Buffer[1] - SIDL
  *         SPI_Rx_Buffer[2] - EID8
  *         SPI_Rx_Buffer[3] - EID0
  *         SPI_Rx_Buffer[4] - DLC
  *         the number of data bytes is depending on DLC SPI_Rx_Buffer[5..12]
  */
#define bSIDH  0
#define bSIDL  1
#define bEID8  2
#define bEID0  3
#define bDLC   4
can_message MCP2515_Read_CAN_Message(uint8_t ReceiveBuffer)
{
  can_message   message;
  unsigned char i;
  uint8_t Rx_Buffer[14];

  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitByte(BOARD_SPI_CHANNEL, ReceiveBuffer);
  SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  
  for (i=0;i<13;i++)
    Rx_Buffer[i]=SPI_ReceiveByte(BOARD_SPI_CHANNEL);
  SPI_DeselectDevice();
  
  message.dlc = (Rx_Buffer[4] & 0x0F) ;
  if ( Rx_Buffer[bSIDL] & 0x10 ) {
    message.MsgType = EXTENDED ;
  } else {
    message.MsgType = STANDARD ;
  }
  
  message.MsgID  = (Rx_Buffer[bSIDH]<<3) + (Rx_Buffer[bSIDL]>>5);
  if ( message.MsgType == EXTENDED ) {
    message.MsgID = (message.MsgID<<2) + (Rx_Buffer[bSIDL] & 0x03) ;
    message.MsgID = (message.MsgID<<16) + (Rx_Buffer[bEID8]<<8) + Rx_Buffer[bEID0] ;
  }
  
  memcpy (message.Data.b,&Rx_Buffer[5],message.dlc);
  return(message) ;
}


/**
  * @brief  function to transmit a complete CAN message into one transmit buffer
  *         The trnamission will not started with this function!
  * @param  TransmitBuffer one of the values TRANSMITBUFFER_0, TRANSMITBUFFER_1 or TRANSMITBUFFER_2
  * @param  Identifier     CAN identifier as 11 or 29 Bit value
  * @param  TypeIdentifier STANDARD or EXTENDED
  * @param  CAN_Data       data to be transmitted into transmit buffer
  * @param  DataLength     number of data to be transmitted into transmit buffer
  * @retval None
  */
void MCP2515_Load_CAN_Message(uint8_t TransmitBuffer, can_message message)
{
  uint8_t SIDH;
  uint8_t SIDL;
  uint8_t EID8 = 0x00;
  uint8_t EID0 = 0x00;
  uint32_t canid ;
  uint8_t Tx_Buffer[6];

  Tx_Buffer[0] = TransmitBuffer;

  // preparation of the register values out of the identifier parameter and type
  canid = (message.MsgID & 0x1FFFFFFF);
  if (message.MsgType == EXTENDED) 
  {
       EID0 = (uint8_t) (canid & 0xFF);
       EID8 = (uint8_t) (canid >> 8);
       canid = (unsigned int)(canid >> 16);
       SIDL = (uint8_t) (canid & 0x03);
       SIDL += (uint8_t) ((canid & 0x1C ) << 3);
       SIDL |= 0x08 ;
       SIDH = (uint8_t) (canid >> 5 );
  }
  else
  {
       SIDH = (uint8_t)(canid >> 3 );
       SIDL = (uint8_t)((canid & 0x07 ) << 5);
       EID0 = 0;
       EID8 = 0;
  }
  
  Tx_Buffer[1] = SIDH;
  Tx_Buffer[2] = SIDL;
  Tx_Buffer[3] = EID8;
  Tx_Buffer[4] = EID0;
  Tx_Buffer[5] = message.dlc;

  SPI_SelectDevice(SPI_DEVICE_CAN);
  SPI_TransmitBytes(BOARD_SPI_CHANNEL, Tx_Buffer, sizeof(Tx_Buffer));
  SPI_TransmitBytes(BOARD_SPI_CHANNEL, (uint8_t*)message.Data.b,  message.dlc);

  SPI_DeselectDevice();
}

/**
  * @brief  function to setup CAN filter and masks
  * @param  Identifier     CAN identifier as 11 or 29 Bit value
  * @param  TypeIdentifier STANDARD or EXTENDED
  * @param  Address        FILTER_n_ADDRESS with n = [0..5] or MASK_n_ADDRESS with n = [0..1]
  * @param  Wait 0 - no wait 1 - wait for the complete SPI protocol
  *         in case of no wait the user has to check the complete transaction
  * @retval None
  */
void MCP2515_Set_CAN_Filter_Mask(uint32_t Identifier, uint8_t TypeIdentifier, uint8_t Address)
{
  uint8_t SIDH;
  uint8_t SIDL;
  uint8_t EID8 = 0x00;
  uint8_t EID0 = 0x00;
  uint32_t canid ;
  unsigned char i;
  uint8_t Tx_Buffer[6];

  Tx_Buffer[0] = WRITE_REGISTER_COMMAND;
  canid = (Identifier & 0x1FFFFFFF);
  if (TypeIdentifier == EXTENDED) 
  {
       EID0 = (uint8_t) (canid & 0xFF);
       EID8 = (uint8_t) (canid >> 8);
       canid = (unsigned int)(canid >> 16);
       SIDL = (uint8_t) (canid & 0x03);
       SIDL += (uint8_t) ((canid & 0x1C ) << 3);
       SIDL |= 0x08 ;
       SIDH = (uint8_t) (canid >> 5 );
    } else {
       SIDH = (uint8_t)(canid >> 3 );
       SIDL = (uint8_t)((canid & 0x07 ) << 5);
       EID0 = 0;
       EID8 = 0;
    }
  Tx_Buffer[1] = Address;
  Tx_Buffer[2] = SIDH;
  Tx_Buffer[3] = SIDL;
  Tx_Buffer[4] = EID8;
  Tx_Buffer[5] = EID0;

  SPI_SelectDevice(SPI_DEVICE_CAN);
  for (i=0;i<6;i++)
    SPI_TransmitByte(BOARD_SPI_CHANNEL, Tx_Buffer[i]);
  
  SPI_DeselectDevice();
}

/**
  * @brief  Inetrrupt handler for the MCP2515
  * @param  None
  * @retval None
  */
void INT5_IRQHandler()
{
  portBASE_TYPE xHigherPriorityTaskWoken;
  xSemaphoreGiveFromISR(CanRXSemaphore, &xHigherPriorityTaskWoken);
  NVIC_DisableIRQ(INT5_IRQn);

}

#endif
