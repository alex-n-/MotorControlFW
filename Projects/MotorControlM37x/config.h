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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "user_config.h"

#ifdef __ICCARM__
/* for assert_param() */
#include "tx03_common.h"
#define __attribute__(x)
#endif /* __ICCARM__ */

#ifdef __KEIL__
/* for assert_param() */
#include "tx03_common.h"
#define __attribute__(x)
#endif /* __KEIL__ */

/* Version information */
#define FW_VERSION_MAJOR             1                                          /*!< Major number of the firmware */
#define FW_VERSION_MINOR             9                                          /*!< Minor number of the firmware */

/* Stack sizes & task priorities (lower priority value is higher importance) */
#define INIT_TASK_STACK_SIZE        ((unsigned portSHORT)      50)              /*!< Stack Site for Init Process Task */
#define SYSTEM_LOAD_TASK_STACK_SIZE ((unsigned portSHORT)      40)              /*!< Stack Site for System Load Task */
#define VE_TASK_STACK_SIZE          ((unsigned portSHORT)      55)              /*!< Stack Site for Vector Engine Task */
#define UI_TASK_STACK_SIZE          ((unsigned portSHORT)      80)              /*!< Stack Site for User Interface Task */
#define PROTOCOL_TASK_STACK_SIZE    ((unsigned portSHORT)     130)              /*!< Stack Site for Serial Protocol Task */
#define CAN_TASK_STACK_SIZE         ((unsigned portSHORT)     100)              /*!< Stack Site for CAN Communication Task */
#define STALL_TASK_STACK_SIZE       ((unsigned portSHORT)      65)              /*!< Stack Site for Stall Detection Task */
#define TURN_TASK_STACK_SIZE        ((unsigned portSHORT)      60)              /*!< Stack Site for Turn Task */

#define INIT_TASK_PRIORITY          ((unsigned portBASE_TYPE )  4)              /*!< Task Priority for Init Process */
#define SYSTEM_LOAD_TASK_PRIORITY   ((unsigned portBASE_TYPE )  0)              /*!< Task Priority for System Load */
#define VE_TASK_PRIORITY            ((unsigned portBASE_TYPE )  4)              /*!< Task Priority for Vector Engine */
#define UI_TASK_PRIORITY            ((unsigned portBASE_TYPE )  0)              /*!< Task Priority for User Interface */
#define PROTOCOL_TASK_PRIORITY      ((unsigned portBASE_TYPE )  1)              /*!< Task Priority for Serial Protocol */
#define CAN_TASK_PRIORITY           ((unsigned portBASE_TYPE )  2)              /*!< Task Priority for CAN Communication */
#define STALL_TASK_PRIORITY         ((unsigned portBASE_TYPE )  3)              /*!< Task Priority for Stall Detection */
#define TURN_TASK_PRIORITY          ((unsigned portBASE_TYPE )  3)              /*!< Task Priority for Turn Task */

/* Interrupt priorities - lower value is higher priority */
#define INTERRUPT_PRIORITY_ERROR     0                                          /*!< Interrupt Priority for External Error Signal */
#define INTERRUPT_PRIORITY_VE        1                                          /*!< Interrupt Priority for Vector Engine */
#define INTERRUPT_PRIORITY_PMD       1                                          /*!< Interrupt Priority for Programable Motor Driver */
#define INTERRUPT_PRIORITY_ENCODER   2                                          /*!< Interrupt Priority for Encoder */
#define INTERRUPT_PRIORITY_CAN       3                                          /*!< Interrupt Priority for CAN */

#ifdef __TMPM_370__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_0|VE_CHANNEL_1                  /* M370 has VE channel 0 & 1 available */
/* Specifiction for DSO storage size */
#ifdef USE_DSO
#define BOARD_DSO_SIZE               1536                                       /* Number of uint16_t */
#else
#define BOARD_DSO_SIZE               0                                          /* Number of uint16_t */
#endif /* USE_DSO */
#endif /* __TMPM_370__ */

#if ( (defined __TMPM_372__) || (defined __TMPM_373__) || (defined __TMPM_374__) )
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_1                               /* M37_234 has only channel 1 available */
#undef  MOTOR_CHANNEL_0
/* Specifiction for DSO storage size */
#ifdef USE_DSO
#define BOARD_DSO_SIZE               402                                        /* Number of uint16_t */
#else
#define BOARD_DSO_SIZE               0                                          /* Number of uint16_t */
#endif /* USE_DSO */
#endif /* (defined __TMPM_372__) || (defined __TMPM_373__) || (defined __TMPM_372__) */

#ifdef __TMPM_375__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_1                               /* M375 has VE channel 1 available */
#undef  MOTOR_CHANNEL_0
#undef USE_DSO
#endif /* __TMPM_370__ */

#ifdef __TMPM_376__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_0|VE_CHANNEL_1                  /* M376 has VE channel 0 & 1 available */
/* Specifiction for DSO storage size */
#ifdef USE_DSO
#define BOARD_DSO_SIZE               1536                                       /* Number of uint16_t */
#else
#define BOARD_DSO_SIZE               0                                          /* Number of uint16_t */
#endif /* USE_DSO */
#endif /* __TMPM_370__ */

/* Board header files */
#ifdef BOARD_HITEX_M370
#define BOARD_NAME               "Hitex M370"                                   /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE  "hitex_m370_board.h"
#define BOARD_LED_HEADER_FILE    "hitex_m370_led.h"
#define BOARD_SPI_HEADER_FILE    "hitex_m370_spi_device.h"
#define BOARD_GAIN_HEADER_FILE   "amp_cmp.h"
#define BOARD_LCD_HEADER_FILE    "hitex_m370_lcd.h"
#define BOARD_KEYPAD_HEADER_FILE "hitex_m370_keypad.h"

#undef BOARD_PWR_HEADER_FILE_0
#undef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_0   "hitex_m370_pwr_hv.h"                         /*! Powerboard to be used */
#define BOARD_PWR_HEADER_FILE_1   "hitex_m370_pwr_lv.h"                         /*! Powerboard to be used */

#define USE_KEYPAD                                                              /*!< Activate the keypad input */
#define USE_UI                                                                  /*!< Activate the User Interface */
#define USE_LCD                                                                 /*!< Activate the LCD */
#undef  USE_CAN                                                                 /*!< Disable CAN communication */
#endif /* BOARD_HITEX_M370 */

#ifdef BOARD_M372STK
#define BOARD_NAME               "M372STK"                                      /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE  "m374stk_board.h"
#define BOARD_LED_HEADER_FILE    "m374stk_led.h"
#define BOARD_SPI_HEADER_FILE    "m374stk_spi_device.h"
#define BOARD_GAIN_HEADER_FILE   "m374stk_gain.h"
#define BOARD_PGA_HEADER_FILE    "m374stk_pga.h"
#define BOARD_CAN_HEADER_FILE    "m374stk_can.h"

#ifndef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_1   "m374pwr_lv.h"                                /*! Powerboard to be used */
#endif /* BOARD_PWR_HEADER_FILE_1 */

#endif /* BOARD_M372STK */

#ifdef BOARD_M37SIGMA
#define BOARD_NAME                "M37Sigma"                                    /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE   "m37sigma_board.h"
#define BOARD_LED_HEADER_FILE     "m37sigma_led.h"
#define BOARD_SPI_HEADER_FILE     "m37sigma_spi_device.h"
#define BOARD_GAIN_HEADER_FILE    "m37sigma_gain.h"
#define BOARD_RGB_LED_HEADER_FILE "m37sigma_rgb_led.h"

#ifndef DEBUG
#undef USE_INTERNAL_MOTOR_PARAMS
#endif /* DEBUG */

#undef  USE_CONFIG_STORAGE_EEPROM
#define USE_CONFIG_STORAGE_FLASH
#undef  USE_CAN
#ifndef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_1   "m37Sigma_pwr.h"                              /*! Powerboard to be used */
#endif /* BOARD_PWR_HEADER_FILE_1 */

#endif /* BOARD_M37SIGMA */

#ifdef BOARD_M374STK
#define BOARD_NAME               "M374STK"                                      /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE  "m374stk_board.h"
#define BOARD_LED_HEADER_FILE    "m374stk_led.h"
#define BOARD_SPI_HEADER_FILE    "m374stk_spi_device.h"
#define BOARD_GAIN_HEADER_FILE   "m374stk_gain.h"
#define BOARD_PGA_HEADER_FILE    "m374stk_pga.h"
#define BOARD_CAN_HEADER_FILE    "m374stk_can.h"

#ifndef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_1  "m374pwr_lv.h"                                 /*! Powerboard to be used */
#endif /* BOARD_PWR_HEADER_FILE_1 */

#endif /* BOARD_M374STK */

/* Chip header files */

#ifdef __TMPM_370__
#define TMPM_UART_HEADER_FILE    "tmpm370_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm370_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm370_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm370_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm370_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm370_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM370.h"
#endif /* __TMPM_370__ */

#ifdef __TMPM_372__
#define TMPM_UART_HEADER_FILE    "tmpm372_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm372_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm372_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm372_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm372_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm372_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM372.h"
#endif /* __TMPM_372__ */

#ifdef __TMPM_373__
#define TMPM_UART_HEADER_FILE    "tmpm373_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm373_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm373_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm373_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm373_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm373_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM373.h"
#endif /* __TMPM_373__ */

#ifdef __TMPM_374__
#define TMPM_UART_HEADER_FILE    "tmpm374_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm374_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm374_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm374_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm374_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm374_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM374.h"
#endif /* __TMPM_374__ */

#ifdef __TMPM_375__
#define TMPM_UART_HEADER_FILE    "tmpm375_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm375_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm375_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm375_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm375_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm375_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM375.h"
#endif /* __TMPM_375__ */

#ifdef __TMPM_376__
#define TMPM_UART_HEADER_FILE    "tmpm376_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm376_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm376_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm376_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm376_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm376_tmrb.h"
#define TMPM_HEADER_FILE         "TMPM376.h"
#endif /* __TMPM_376__ */

/* Checking for consistency */

#if ( !(defined USE_CONFIG_STORAGE) && !(defined USE_INTERNAL_MOTOR_PARAMS) )
#error EITHER USE_CONFIG_STORAGE OR USE_INTERNAL_MOTOR_PARAMS OR BOTH HAVE TO BE DEFINED!!!
#endif

#if ( (defined USE_UI) &&  !(defined USE_LCD) )
#error USING THE UI WITHOUT LCD MAKES NO SENSE
#endif

#if ( !(defined USE_ENCODER) && (defined USE_TURN_CONTROL))
#error WHEN USING TURN CONTROL ENCODER HAS TO BE USED
#endif

#ifdef USE_CONFIG_STORAGE
#if ( (defined USE_CONFIG_STORAGE_FLASH) && (defined USE_CONFIG_STORAGE_EEPROM))
#error Only either USE_CONFIG_STORAGE_FLASH or USE_CONFIG_STORAGE_EEPROM must be defined
#endif
#if ( (!defined USE_CONFIG_STORAGE_FLASH) && (!defined USE_CONFIG_STORAGE_EEPROM))
#error One of USE_CONFIG_STORAGE_FLASH or USE_CONFIG_STORAGE_EEPROM must be defined
#endif
#endif

#endif /* _CONFIG_H_ */
