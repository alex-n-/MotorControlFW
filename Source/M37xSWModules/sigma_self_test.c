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

#ifdef SIGMA_PRODUCTION

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "motorctrl.h"
#include "board.h"

#include TMPM_GPIO_HEADER_FILE
#include TMPM_UART_HEADER_FILE
#include TMPM_ADC_HEADER_FILE
#include BOARD_RGB_LED_HEADER_FILE

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

extern FWVersion              FirmwareVersion;

static uint8_t port_table[][2] =
{
  {GPIO_PI,GPIO_BIT_3},         //# 0
  {GPIO_PJ,GPIO_BIT_0},         //# 1
  {GPIO_PJ,GPIO_BIT_6},         //# 2
  {GPIO_PJ,GPIO_BIT_7},         //# 3
  {GPIO_PK,GPIO_BIT_0},         //# 4
  {GPIO_PK,GPIO_BIT_1},         //# 5
  {GPIO_PA,GPIO_BIT_2},         //# 6
  {GPIO_PF,GPIO_BIT_0},         //# 7
  {GPIO_PF,GPIO_BIT_1},         //# 8
  {GPIO_PF,GPIO_BIT_2},         //# 9
  {GPIO_PF,GPIO_BIT_3},         //# 10
  {GPIO_PF,GPIO_BIT_4},         //# 11
  {GPIO_PE,GPIO_BIT_4},         //# 12
  {GPIO_PE,GPIO_BIT_7},         //# 13
  {GPIO_PB,GPIO_BIT_3},         //# 14
  {GPIO_PB,GPIO_BIT_4},         //# 15
  {GPIO_PB,GPIO_BIT_5},         //# 16
  {GPIO_PB,GPIO_BIT_6},         //# 17
  {GPIO_PA,GPIO_BIT_5},         //# 18
  {GPIO_PD,GPIO_BIT_6},         //# 19
  {GPIO_PE,GPIO_BIT_2},         //# 20
  {GPIO_PE,GPIO_BIT_6},         //# 21
};

static uint8_t connection_table[][2] =
{
  { 0, 7},      // PI3->PF0     #0
  { 7, 0},      // PF0->PI3     #1
  { 1, 6},      // PJ0->PA2     #2
  { 6, 1},      // PA2->PJ0     #3
  { 2, 8},      // PJ6->PF1     #4
  { 8, 2},      // PF1->PJ6     #5
  { 3,11},      // PJ7->PF4     #6
  {11, 3},      // PF4->PJ7     #7
  { 4,10},      // PK0->PF3     #8
  {10, 4},      // PF3->PK0     #9
  { 5, 9},      // PK1->PF2     #10
  { 9, 5},      // PF2->PK1     #11
  {13,12},      // PE7->PE4     #12
  {12,13},      // PE4->PE7     #13
  {15,18},      // PB4->PA5     #15
  {17,19},      // PB6->PD6     #17
  {14,20},      // PB3->PE2     #14
  {16,21},      // PB5->PE6     #16
};

struct fet_test_s
{
  char          name[4];
  uint8_t       pattern;
  uint16_t      expected;
};

static struct fet_test_s fet_test[5] =
{
  {"LS  ",0x15,0x7ff},
  {"HS  ",0x2a,0x7ff},
  {"H1L1",0x03,0x841},
  {"H2L2",0x0c,0x841},
  {"H3L3",0x30,0x841},
};

static const GPIO_InitTypeDef portConfigInput =
{
  GPIO_INPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_ENABLE,
};

static const GPIO_InitTypeDef portConfigOutput =
{
  GPIO_OUTPUT_MODE,
  GPIO_PULLUP_DISABLE,
  GPIO_OPEN_DRAIN_DISABLE,
  GPIO_PULLDOWN_ENABLE,
};


static char bit_nr(uint8_t value)
{
  unsigned char nr = 0;

  for (; value; value >>= 1)
    nr++;

  return nr-1;
}  

static int check_connection(uint8_t connection)
{
  uint8_t i,state;
  
  for (i=0;i<(sizeof(port_table)/(2*sizeof(uint8_t)));i++)
  {
    if (port_table[i][0]!= GPIO_PB)
      GPIO_Init          ((GPIO_Port)port_table[i][0],port_table[i][1],&portConfigInput);
    else
      GPIO_Init          ((GPIO_Port)port_table[i][0],port_table[i][1],&portConfigOutput);
        
    GPIO_DisableFuncReg((GPIO_Port)port_table[i][0],GPIO_FUNC_REG_1,port_table[i][1]);
    if (port_table[i][0]!= GPIO_PB)
      GPIO_WriteDataBit  ((GPIO_Port)port_table[i][0],port_table[i][1],0x0);
    else
      GPIO_WriteDataBit  ((GPIO_Port)port_table[i][0],port_table[i][1],0x1);
  }
  
  GPIO_Init((GPIO_Port)port_table[connection_table[connection][0]][0],
                       port_table[connection_table[connection][0]][1],
                       &portConfigOutput);

  if (port_table[connection_table[connection][0]][0]!= GPIO_PB)
    GPIO_WriteDataBit((GPIO_Port)port_table[connection_table[connection][0]][0],
                                 port_table[connection_table[connection][0]][1],
                                 0x1);
  else
    GPIO_WriteDataBit((GPIO_Port)port_table[connection_table[connection][0]][0],
                                 port_table[connection_table[connection][0]][1],
                                 0x0);

  vTaskDelay( 2 / portTICK_RATE_MS );

  for (i=0;i<(sizeof(port_table)/(2*sizeof(uint8_t)));i++)
  {
    state = GPIO_ReadDataBit((GPIO_Port)port_table[i][0],
                                        port_table[i][1]);
    
    if (state == 0)
      continue;
    
    if ((state == 1)
        && ((connection_table[connection][0]==i) || (connection_table[connection][1]==i)))
      continue;

    if (port_table[i][0]== GPIO_PB)
      continue;
    
    return 0;
  }
  
  return 1;
  
}

static int check_fets(uint8_t pattern, uint16_t expected)
{
  ADC_Result         result;
  
  memset(&result,0,sizeof(result));

  GPIO_Init     (GPIO_PG,pattern,&portConfigOutput);
  GPIO_WriteData(GPIO_PG,pattern);

  vTaskDelay( 50 / portTICK_RATE_MS );
  
  ADC_Enable(TSB_ADB);  
  ADC_SetClk(TSB_ADB, ADC_HOLD_FIX, ADC_FC_DIVIDE_LEVEL_2);
  ADC_SetSWTrg(TSB_ADB, (ADC_REGx) AIN_1PHASE_CURRENT, TRG_ENABLE(AIN_1PHASE_CURRENT));
  ADC_Start(TSB_ADB, ADC_TRG_SW);
  while (ADC_GetConvertState(TSB_ADB, ADC_TRG_SW) == BUSY);
  result=ADC_GetConvertResult(TSB_ADB, (ADC_REGx) AIN_1PHASE_CURRENT);
  ADC_Disable(TSB_ADB);  

  GPIO_WriteData(GPIO_PG,0x0);
  
  if (abs(result.Bit.ADResult - expected) < 0x15)
    return 1;
  else
    return 0;
}

/*! \brief Perform Sigma Self Test on special test board
  *
  * Board test for Toshiba production
  *
  * @param  text: text to be printed
  * @retval finished all tests
*/
int SigmaSelfTest (char* text)
{
  static uint8_t testcase = 0;
  static uint8_t subtest  = 0;
  static uint8_t error_counter = 0;
  uint8_t        test_val;
  
  switch (testcase)
  {
  case 0:
    sprintf(text, "Sigma Selftest\n");
    testcase++;
    break;
  case 1:
    sprintf(text, "Version: %d.%d\n",FirmwareVersion.fw_version[0],FirmwareVersion.fw_version[1]);
    testcase++;
    break;
  case 2:
    if (external_clock_working == 1)
      sprintf(text, "External Clock test passed");
    else
      sprintf(text, "External Clock test failed");
    testcase++;
    break;
  case 3:
    text[0]='P';
    text[1]=port_table[connection_table[subtest][0]][0]+ 65;
    text[2]=bit_nr(port_table[connection_table[subtest][0]][1])+ 48;
    text[3]='-';
    text[4]='>';
    text[5]='P';
    text[6]=port_table[connection_table[subtest][1]][0]+ 65;
    text[7]=bit_nr(port_table[connection_table[subtest][1]][1])+ 48;
    
    test_val=check_connection(subtest);

    if (test_val)
      sprintf(&text[8]," passed\n");
    else
    {
      sprintf(&text[8]," failed\n");
      error_counter++;
    }

    subtest++;
    
    if (subtest >= sizeof(connection_table)/(2*sizeof(uint8_t)))
    {
      subtest=0;
      testcase++;
    }
    break;

  case 4:
    sprintf(text,"FETTest %s ",fet_test[subtest].name);
    test_val=check_fets(fet_test[subtest].pattern,fet_test[subtest].expected);

    if (test_val)
      sprintf(&text[12]," passed\n");
    else
    {
      sprintf(&text[12]," failed\n");
      error_counter++;
    }

    subtest++;
    
    if (subtest >= sizeof(fet_test)/(sizeof(fet_test[0])))
    {
      subtest=0;
      testcase++;
    }
    break;
  default:
   
#ifdef USE_RGB_LED
    if (error_counter>0)
      RGB_LED_SetValue(LED_RGB_RED);
    else
      RGB_LED_SetValue(LED_RGB_GREEN);
#endif /* USE_RGB_LED */ 
    testcase=0;  
    error_counter=0;
    
    return 0;
    break;
  }

  return strlen(text)+1;
}

#endif /* SIGMA_PRODUCTION */
