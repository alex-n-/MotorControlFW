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

#include <stdio.h>

#include "config.h"

#ifdef USE_CAN

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include TMPM_GPIO_HEADER_FILE
#include BOARD_BOARD_HEADER_FILE
#include BOARD_CAN_HEADER_FILE
#include BOARD_SPI_HEADER_FILE

#ifdef USE_LED
#include BOARD_LED_HEADER_FILE
#endif

#include "motorctrl.h"
#include "action.h"
#include "debug.h"
#include "turn_control.h"
#include "encoder.h"
#include "board.h"
#include "MCP2515.h"

#define USE_CHANNEL 1

xSemaphoreHandle CanRXSemaphore;

static  struct StartMotor_q q1;
static  struct StartMotor_a a1;
static  struct StopMotor_q q2;
static  struct StopMotor_a a2;

/*! \brief Message Composer
  *
  * Generyte a CAN Message out of Group and SubGroup
  *
  * @param  Group:      Group of message
  * @param  SubGroup:   SubGroup of message
  *
  * @retval Can Message
*/
static can_message Message_Composer(can_group Group, can_sub_group SubGroup)
{
  can_message message;
 
  memset(&message,0,sizeof(message));
  
  switch (Group) {
  case 0:                                                                       /* Group 0: Reserved */
    break;
  case EMERGENCY:                                                               /* Group 1: Emergency ( receive only ) */
    break;
  case STATUS_INFORMATION:                                                      /* Group 2: Status Information */
    switch (SubGroup)
    {
    case STATUS_INFORMATION_EMERGENCY_SHUTDOWN:                                 /* SubGrp m.0.2: Emergency Shutdown Information */
      message.Data.b[0] = 0x55;                                                 /* Error Flag */
      message.dlc  = 1;
      break;
    case STATUS_INFORMATION_SPEED_TORQUE_CURRENT:                               /* SubGrp m.1.2: Speed Information */
      message.Data.w[0] = MotorStateValues[USE_CHANNEL].ActualSpeed;
      message.Data.w[1] = MotorStateValues[USE_CHANNEL].Torque;
      message.Data.w[2] = MotorStateValues[USE_CHANNEL].Current;
      message.Data.w[3] = TEE_VE1->VDC * VE_v_max[USE_CHANNEL] / (0xfff<<3);
      message.dlc  = 8;
      break;
#ifdef USE_TURN_CONTROL      
    case STATUS_INFORMATION_TURN_POSITION:                                      /* SubGrp m.1.2: Speed Information */
      message.Data.l[0] = EncoderData[USE_CHANNEL].FullTurns;
      message.dlc  = 4;
      break;
#endif /* USE_TURN_CONTROL */   
    default:
       break;
    }
      break;
    case TARGET_SETTING:                                                        /* Group 3: Target Setting ( receive only ) */
      break;
    case CONFIGURATION_INFORMATION:                                             /* Group 6: Configuration Information */
      switch (SubGroup)
      {
      case CONFIGURATION_INFORMATION_NAME:                                      /* SubGrp m.0.6: Motor Name */
        memcpy(&message.Data.b[0], MotorParameterValues[USE_CHANNEL].MotorId, sizeof(message.Data));
        message.dlc = sizeof(message.Data);
        break;
      case CONFIGURATION_INFORMATION_RPM:                                       /* SubGrp m.1.6: RPM Data */
        message.Data.w[0]= MotorParameterValues[USE_CHANNEL].HzLimit *SECONDS_PER_MINUTE / MotorParameterValues[USE_CHANNEL].PolePairs; /* MaxRPM */
        message.Data.w[2]= MotorParameterValues[USE_CHANNEL].HzChange*SECONDS_PER_MINUTE / MotorParameterValues[USE_CHANNEL].PolePairs; /* ChgRPM */
        message.dlc      = 6;
        break;
      case CONFIGURATION_INFORMATION_MAX_VA:                                    /* SubGrp m.2.6: Max Voltage, Current, Acceleration Data */
        message.Data.w[0]= ChannelValues[USE_CHANNEL].sensitivity_voltage_measure; /* Gradient of voltage measurement */
        message.Data.w[1]= ChannelValues[USE_CHANNEL].sensitivity_current_measure; /* Gradient of current measurement */
        message.Data.w[2]= MotorParameterValues[USE_CHANNEL].MaxAngAcc;         /* MaxAng */
        message.dlc      = 6;
        break;
      case CONFIGURATION_INFORMATION_TORQUE:                                    /* SubGrp m.3.6: Torque Factor */
        message.Data.w[0]= MotorParameterValues[USE_CHANNEL].TorqueFactor;      /* Torque Factor */
        message.dlc      = 2;
        break;
      default:
        break;
      }
      break;
    case CONFIGURATION_SETTING :  // Group 7: Configuration Setting ( receive only )
      break;
    default:
      return message;
    }

    message.MsgID =  Group << GROUP_OFFSET 
                   | SubGroup << SUBFROUP_OFFSET
                   | SystemValues[USE_CHANNEL].can_id << ADDRESS_OFFSET;
    message.MsgType = STANDARD ;
    
    return message;
}

/*! \brief Message Analyzer
  *
  * Analyze the receive buffer and generate action, based on the message ID
  *
  * @param  message: Receive Buffer
  *
  * @retval None
*/
static void Message_Analyzer(can_message message)
{
#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_suppress = Pe188
#elif (defined (__KEIL__))
#pragma diag_suppress 188
#endif
  can_group     Group    = (message.MsgID >> GROUP_OFFSET)    & 0x0f;
  can_sub_group SubGroup = (message.MsgID >> SUBFROUP_OFFSET) & 0x0f;
#if (defined(__ICCARM__)) /* IAR Compiler - */
#pragma diag_default = Pe188
#elif (defined (__KEIL__))
#pragma diag_default 188
#endif
  
  uint8_t can_id = (message.MsgID >> ADDRESS_OFFSET)  & 0x0f;
    
  if ((SystemValues[USE_CHANNEL].can_id != can_id) && (can_id!=0))
    return;
  else
  {
    switch (Group)
    {    
    case EMERGENCY:                                                             // Group 1: Emergency
      switch (SubGroup)
      {
        case EMERGENCY_SHUTDOWN_ALL:                                            // SubGrp m.0.1: Emergency Shutdown All
        case EMERGENCY_SHUTDOWN_NODE:                                           // SubGrp m.1.1: Emergency Shutdown Node
          q2.motor_nr=USE_CHANNEL;
          StopMotor (&q2,&a2);
          break;
        default:
          break;
      }
      break ;
    case TARGET_SETTING:                                                        // Group 3: Target Settings
      switch (SubGroup)
      {
      case TARGET_SETTING_SPEED_NODE:                                           // SubGrp m.2.2: Target Setting Speed
        /////////////////////////////////////////////////////
        // Target Speed = TargetRPM
        // ptM1->Speed =    (signed short int)canMSG.DATA.DATA_Int[0];
        /////////////////////////////////////////////////////
        MotorSetValues[USE_CHANNEL].TargetSpeed = message.Data.w[0];

        if (message.Data.w[0] == 0)
        {
          q2.motor_nr=USE_CHANNEL;
          StopMotor (&q2,&a2);
        }
        else
        {
          q1.motor_nr=USE_CHANNEL;
          StartMotor(&q1,&a1);
        }
        break;
#ifdef USE_TURN_CONTROL      
      case TARGET_SETTING_TURN_NODE:                                            // SubGrp m.2.3: Target Setting Turns
        CalculateTurnRampAndTurn(USE_CHANNEL,message.Data.w[0],message.Data.w[1]);
        break;
#endif /* USE_TURN_CONTROL */
      default:
        break;
      }
      break ;
    case CONFIGURATION_SETTING:                                                 // Group 7: Configuration Settings
      switch (SubGroup){
      case 0:                                                                   // SubGrp m.0.7: Configuration Setting: Name
        /////////////////////////////////////////////////////
        // Name = Name
        /////////////////////////////////////////////////////
        break;
      default:
        break;
      }
      break ;
    default:
      break;    
    }
  }
  return;
}

/*! \brief CAN Init
  *
  * Initalize CAN Chip
  *
  * @retval None
*/
static void CAN_Init(void)
{
  can_message message;
  
  MCP2515_Reset();                                                              /* Set MCP2515 in RESET-State */
  vTaskDelay( 50 / portTICK_RATE_MS );                                          /* wait for 50 ms to give MCP2515 time for reset */

  MCP2515_SetOperationMode(CONFIGUARTION_MODE);                                 /* Set MCP2515 in configuration mode */
  
  MCP2515_WriteBitTimingRegister(CNF1_0125_10MHz,                               /* CAN Bus timing for 125 MHz at 16 MHz clock */
                                 CNF2_0125_10MHz,
                                 CNF3_0125_10MHz);
  MCP2515_WriteRegister(RECEIVE_BUFFER_0_CONTROL_REGISTER,                      /* Receive buffer 0 ready for all identifier types with a mask and filter for 0x5678 */
                        MASK_FILTER_BOTH_IDENT);
  MCP2515_Set_CAN_Filter_Mask(0x0701,
                              STANDARD,
                              MASK_0_ADDRESS);
  MCP2515_Set_CAN_Filter_Mask((0x0001 | SystemValues[USE_CHANNEL].can_id << ADDRESS_OFFSET ),
                              STANDARD,
                              FILTER_0_ADDRESS);          
  MCP2515_Set_CAN_Filter_Mask(0x0001,
                              STANDARD,
                              FILTER_1_ADDRESS);          

  // Receive buffer 1 ready for all identifier types without any mask and filter
  MCP2515_WriteRegister(RECEIVE_BUFFER_1_CONTROL_REGISTER,
                        MASK_FILTER_BOTH_IDENT);
  MCP2515_Set_CAN_Filter_Mask(0x07FF,
                              STANDARD,
                              MASK_1_ADDRESS);
  MCP2515_Set_CAN_Filter_Mask(0x0000,
                              STANDARD,
                              FILTER_2_ADDRESS);          
  MCP2515_Set_CAN_Filter_Mask(0x0000,
                              STANDARD,
                              FILTER_3_ADDRESS);          
  MCP2515_Set_CAN_Filter_Mask(0x0000,
                              STANDARD,
                              FILTER_4_ADDRESS);          
  MCP2515_Set_CAN_Filter_Mask(0x0000,
                              STANDARD,
                              FILTER_5_ADDRESS);          

  // TX2RTS via EMG
  MCP2515_WriteRegister(DIGITAL_INPUT_REGISTER, 0x04);
  MCP2515_SetInterruptEnableBit(RX1_INTERRUPT_ENABLE +
                                RX0_INTERRUPT_ENABLE 
                                );
  
  MCP2515_SetOperationMode(NORMAL_OPERATION_MODE);
  
  message=Message_Composer(STATUS_INFORMATION,STATUS_INFORMATION_EMERGENCY_SHUTDOWN);
  MCP2515_BitModify(TRANSMIT_BUFFER_2_CONTROL_REGISTER, 0x08, 0x00);
  MCP2515_Load_CAN_Message(TRANSMITBUFFER_0,message);
}

/*! \brief CAN Receive
  *
  * Receive a CAN Message
  *
  * @retval CAN Message
*/
static can_message CAN_Receive(void)
{
  uint8_t     RXStatus;
  uint8_t     Status;
  can_message message;

  memset (&message,0,sizeof(message));
  
  RXStatus  = MCP2515_QuickReadRXstatus();
  
  MCP2515_ReadErrorFlags() ;

  if ((RXStatus & 0x40) == 0x40)
    message=MCP2515_Read_CAN_Message(RECEIVEBUFFER_0);
  if ((RXStatus & 0x80) == 0x80)
    message=MCP2515_Read_CAN_Message(RECEIVEBUFFER_1);

  Status = MCP2515_QuickReadStatus();
  if (Status != 0x00)
  {
    /* clear interrupt flags of message transmission */
    MCP2515_ClrInterruptFlagBit(RX1_INTERRUPT_FLAG);
    MCP2515_ClrInterruptFlagBit(RX0_INTERRUPT_FLAG);
  }
  return message;
}

/*! \brief CAN Send
  *
  * Send a CAN Message
  *
  * @param  buffer_nr: Buffer number of MCP2515
  * @param  message:   CAN Message
  *
  * @retval None
*/
static void CAN_Send(uint8_t buffer_nr, can_message message)
{
  switch (buffer_nr)
  {
  case TRANSMITBUFFER_0:
    MCP2515_BitModify(TRANSMIT_BUFFER_0_CONTROL_REGISTER, 0x08, 0x00);
    break;
  case TRANSMITBUFFER_1:
    MCP2515_BitModify(TRANSMIT_BUFFER_1_CONTROL_REGISTER, 0x08, 0x00);
    break;
  case TRANSMITBUFFER_2:
    MCP2515_BitModify(TRANSMIT_BUFFER_2_CONTROL_REGISTER, 0x08, 0x00);
    break;
  default:
    break;
  }

  MCP2515_Load_CAN_Message(buffer_nr,message);

  switch (buffer_nr)
  {
  case TRANSMITBUFFER_0:
    MCP2515_SetRTS(RTS_TXB0_COMMAND);
    break;
  case TRANSMITBUFFER_1:
    MCP2515_SetRTS(RTS_TXB1_COMMAND);
    break;
  case TRANSMITBUFFER_2:
    MCP2515_SetRTS(RTS_TXB2_COMMAND);
    break;
  default:
    break;
  }
}

/*! \brief CAN Send Init Data
  *
  * Send System data as described in CAN Protocol specification
  *
  * @retval None
*/
static void CAN_SendInitData(void)
{
  can_message message;
  
  message=Message_Composer( CONFIGURATION_INFORMATION , CONFIGURATION_INFORMATION_NAME);
  CAN_Send(TRANSMITBUFFER_0,message);
  vTaskDelay( 50 / portTICK_RATE_MS );

  message=Message_Composer( CONFIGURATION_INFORMATION , CONFIGURATION_INFORMATION_RPM);
  CAN_Send(TRANSMITBUFFER_0,message);
  vTaskDelay( 50 / portTICK_RATE_MS );

  message=Message_Composer( CONFIGURATION_INFORMATION , CONFIGURATION_INFORMATION_MAX_VA);
  CAN_Send(TRANSMITBUFFER_0,message);
  vTaskDelay( 50 / portTICK_RATE_MS );

  message=Message_Composer( CONFIGURATION_INFORMATION , CONFIGURATION_INFORMATION_TORQUE);
  CAN_Send(TRANSMITBUFFER_0,message);
  vTaskDelay( 50 / portTICK_RATE_MS );
}

/*! \brief Display CAN Message
  *
  * Prints out CAN Message (for Debug)
  *
  * @param  message:   CAN Message
  *
  * @retval None
*/
static void Display_CAN_Message(can_message message)
{
  unsigned char i;
  
  if (message.dlc>0)
  {
    dprintf("MessageID:0x%x\n",message.MsgID);
    dprintf("MessageType:0x%x\n",message.MsgType);
    dprintf("MessageLength:0x%x\n",message.dlc);
    for(i=0;i<message.dlc;i++)
      dprintf("Data:0x%x\n",message.Data.b[i]&0xff);
  }
}

static const GPIO_InitTypeDef portConfigRX =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_ENABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_DISABLE,
};

/*! \brief CAN Task
  *
  * @param  pvParameters: Needed due to FreeRTOS
  *
  * @retval None
*/
void CanTask( void *pvParameters )
{
  can_message   message;
  
  while (INIT_Done==0)
    vTaskDelay( 100 / portTICK_RATE_MS );

  if (BoardRevision == 1)                                                       /* Don't use on Revision 1 Boards */
    return;

  GPIO_Init(GPIO_PE, GPIO_BIT_4, &portConfigRX);
  GPIO_EnableFuncReg(GPIO_PE, GPIO_FUNC_REG_2,GPIO_BIT_4);
  
  vSemaphoreCreateBinary(CanRXSemaphore);
  xSemaphoreTake(CanRXSemaphore, 0);

  NVIC_SetPriority(INT5_IRQn, INTERRUPT_PRIORITY_CAN);
  NVIC_EnableIRQ(INT5_IRQn);
  
  CAN_Init();
  CAN_SendInitData();
  
  for( ;; )
  {
    int ret;
#ifdef USE_LED
    LED_Toggle(LED_SIGNAL_CAN_COMMUNICATION_RUNNING);
#endif
    
    ret = xSemaphoreTake(CanRXSemaphore, 200 / portTICK_RATE_MS);
    if(ret != pdFALSE)
    {
      message=CAN_Receive();
      Message_Analyzer(message);
      Display_CAN_Message(message);
      NVIC_EnableIRQ(INT5_IRQn);
    }

    message=Message_Composer(STATUS_INFORMATION, STATUS_INFORMATION_SPEED_TORQUE_CURRENT);
    CAN_Send(TRANSMITBUFFER_0,message);
    vTaskDelay(  10 / portTICK_RATE_MS );
#ifdef USE_TURN_CONTROL    
    message=Message_Composer(STATUS_INFORMATION, STATUS_INFORMATION_TURN_POSITION);
    CAN_Send(TRANSMITBUFFER_0,message);
    vTaskDelay( 100 / portTICK_RATE_MS );
#endif /* USE_TURN_CONTROL */
  }
}

#endif /* USE_CAN */
