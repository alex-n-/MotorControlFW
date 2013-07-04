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

#ifdef __GNUC__
#define __packed
#endif

/* Version information */
#define FW_VERSION_MAJOR             1                                          /*!< Major number of the firmware */
#define FW_VERSION_MINOR             14                                         /*!< Minor number of the firmware */

/* Stack sizes & task priorities (lower priority value is higher importance) */
#define INIT_TASK_STACK_SIZE        ( 95)                                       /*!< Stack Site for Init Process Task */
#define SYSTEM_LOAD_TASK_STACK_SIZE ( 40)                                       /*!< Stack Site for System Load Task */
#define VE_TASK_STACK_SIZE          ( 60)                                       /*!< Stack Site for Vector Engine Task */
#define UI_TASK_STACK_SIZE          ( 80)                                       /*!< Stack Site for User Interface Task */
#define PROTOCOL_TASK_STACK_SIZE    (110)                                       /*!< Stack Site for Serial Protocol Task */
#define CAN_TASK_STACK_SIZE         (100)                                       /*!< Stack Site for CAN Communication Task */
#define STALL_TASK_STACK_SIZE       ( 70)                                       /*!< Stack Site for Stall Detection Task */
#define TURN_TASK_STACK_SIZE        ( 60)                                       /*!< Stack Site for Turn Task */
#define RGB_LED_TASK_STACK_SIZE     ( 60)                                       /*!< Stack Site for RGB Led Task */

#define INIT_TASK_PRIORITY          ((unsigned portBASE_TYPE )  4)              /*!< Task Priority for Init Process */
#define SYSTEM_LOAD_TASK_PRIORITY   ((unsigned portBASE_TYPE )  0)              /*!< Task Priority for System Load */
#define VE_TASK_PRIORITY            ((unsigned portBASE_TYPE )  4)              /*!< Task Priority for Vector Engine */
#define UI_TASK_PRIORITY            ((unsigned portBASE_TYPE )  0)              /*!< Task Priority for User Interface */
#define PROTOCOL_TASK_PRIORITY      ((unsigned portBASE_TYPE )  1)              /*!< Task Priority for Serial Protocol */
#define CAN_TASK_PRIORITY           ((unsigned portBASE_TYPE )  2)              /*!< Task Priority for CAN Communication */
#define STALL_TASK_PRIORITY         ((unsigned portBASE_TYPE )  3)              /*!< Task Priority for Stall Detection */
#define TURN_TASK_PRIORITY          ((unsigned portBASE_TYPE )  3)              /*!< Task Priority for Turn Task */
#define RGB_LED_TASK_PRIORITY       ((unsigned portBASE_TYPE )  0)              /*!< Task Priority for Turn Task */

/* Interrupt priorities - lower value is higher priority */
#define INTERRUPT_PRIORITY_ERROR     0                                          /*!< Interrupt Priority for External Error Signal */
#define INTERRUPT_PRIORITY_ENCODER   1                                          /*!< Interrupt Priority for Encoder */
#define INTERRUPT_PRIORITY_VE        2                                          /*!< Interrupt Priority for Vector Engine */
#define INTERRUPT_PRIORITY_PMD       2                                          /*!< Interrupt Priority for Programable Motor Driver */
#define INTERRUPT_PRIORITY_CAN       3                                          /*!< Interrupt Priority for CAN */

#ifdef __TMPM_370__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_0|VE_CHANNEL_1                  /* M370 has VE channel 0 & 1 available */
#endif /* __TMPM_370__ */

#if ( (defined __TMPM_372__) || (defined __TMPM_373__) || (defined __TMPM_374__) )
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_1                               /* M37_234 has only channel 1 available */
#undef  MOTOR_CHANNEL_0
#endif /* (defined __TMPM_372__) || (defined __TMPM_373__) || (defined __TMPM_372__) */

#ifdef __TMPM_375__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_1                               /* M375 has VE channel 1 available */
#undef  MOTOR_CHANNEL_0
#endif /* __TMPM_370__ */

#ifdef __TMPM_376__
#define MAX_CHANNEL                  2                                          /*!< Number of vector engine channels */
#define BOARD_AVAILABLE_CHANNELS     VE_CHANNEL_0|VE_CHANNEL_1                  /* M376 has VE channel 0 & 1 available */
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

#undef USE_LED
#ifdef SIGMA_PRODUCTION
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

#ifdef BOARD_M375STK
#define BOARD_NAME               "M375STK"                                      /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE  "m375stk_board.h"
#define BOARD_GAIN_HEADER_FILE   "m375stk_gain.h"

#undef  USE_CONFIG_STORAGE_EEPROM
#undef  USE_CONFIG_STORAGE 
//#define USE_CONFIG_STORAGE_FLASH
#undef  USE_CAN
#undef  USE_LED
#undef  USE_DSO
#undef  USE_ENCODER
#ifndef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_1  "m374pwr_lv.h"                                 /*! Powerboard to be used */
#endif /* BOARD_PWR_HEADER_FILE_1 */

#endif /* BOARD_M375STK */

#ifdef BOARD_EFTORCOS
#define BOARD_NAME               "EFTORCOS"                                     /* Name of the Board */
#define BOARD_BOARD_HEADER_FILE  "eftorcos_board.h"
#define BOARD_LED_HEADER_FILE    "eftorcos_led.h"
#define BOARD_SPI_HEADER_FILE    "eftorcos_spi_device.h"
#define BOARD_GAIN_HEADER_FILE   "eftorcos_gain.h"
#define BOARD_PGA_HEADER_FILE    "eftorcos_pga.h"
#define BOARD_CAN_HEADER_FILE    "eftorcos_can.h"

#undef  USE_CONFIG_STORAGE_EEPROM
#define USE_CONFIG_STORAGE_FLASH
#undef  USE_HSDSO

#ifndef BOARD_PWR_HEADER_FILE_1
#define BOARD_PWR_HEADER_FILE_1  "eftorcos_pwr.h"                               /*! Powerboard to be used */
#endif /* BOARD_PWR_HEADER_FILE_1 */

#endif /* BOARD_EFTORCOS */

/* Chip header files and information */

#ifdef __TMPM_370__
#define TMPM_UART_HEADER_FILE    "tmpm370_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm370_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm370_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm370_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm370_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm370_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm370_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm370_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm370_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm370_ofd.h"
#define TMPM_HEADER_FILE         "TMPM370.h"
#define MEMORY_SIZE              0x2800
#define PLL_MULTIPLIER           8                                              /* Multiplier of the PLL */
#endif /* __TMPM_370__ */

#ifdef __TMPM_372__
#define TMPM_UART_HEADER_FILE    "tmpm372_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm372_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm372_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm372_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm372_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm372_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm372_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm372_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm372_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm372_ofd.h"
#define TMPM_HEADER_FILE         "TMPM372.h"
#define MEMORY_SIZE              0x1800
#define PLL_MULTIPLIER           8                                              /* Multiplier of the PLL */
#endif /* __TMPM_372__ */

#ifdef __TMPM_373__
#define TMPM_UART_HEADER_FILE    "tmpm373_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm373_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm373_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm373_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm373_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm373_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm373_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm373_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm373_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm373_ofd.h"
#define TMPM_HEADER_FILE         "TMPM373.h"
#define MEMORY_SIZE              0x1800
#define PLL_MULTIPLIER           8                                              /* Multiplier of the PLL */
#endif /* __TMPM_373__ */

#ifdef __TMPM_374__
#define TMPM_UART_HEADER_FILE    "tmpm374_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm374_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm374_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm374_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm374_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm374_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm374_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm374_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm374_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm374_ofd.h"
#define TMPM_HEADER_FILE         "TMPM374.h"
#define MEMORY_SIZE              0x1800
#define PLL_MULTIPLIER           8                                              /* Multiplier of the PLL */
#endif /* __TMPM_374__ */

#ifdef __TMPM_375__
#define TMPM_UART_HEADER_FILE    "tmpm375_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm375_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm375_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm375_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm375_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm375_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm375_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm375_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm375_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm375_ofd.h"
#define TMPM_HEADER_FILE         "TMPM375.h"
#define MEMORY_SIZE              0x1800
#define PLL_MULTIPLIER           4                                              /* Multiplier of the PLL */
#endif /* __TMPM_375__ */

#ifdef __TMPM_376__
#define TMPM_UART_HEADER_FILE    "tmpm376_uart.h"
#define TMPM_GPIO_HEADER_FILE    "tmpm376_gpio.h"
#define TMPM_WDT_HEADER_FILE     "tmpm376_wdt.h"
#define TMPM_FC_HEADER_FILE      "tmpm376_fc.h"
#define TMPM_ADC_HEADER_FILE     "tmpm376_adc.h"
#define TMPM_TIMER_HEADER_FILE   "tmpm376_tmrb.h"
#define TMPM_CG_HEADER_FILE      "tmpm376_cg.h"
#define TMPM_WDT_HEADER_FILE     "tmpm376_wdt.h"
#define TMPM_VLTD_HEADER_FILE    "tmpm376_vltd.h"
#define TMPM_OFD_HEADER_FILE     "tmpm376_ofd.h"
#define TMPM_HEADER_FILE         "TMPM376.h"
#define MEMORY_SIZE              0x8000
#define PLL_MULTIPLIER           8                                              /* Multiplier of the PLL */
#endif /* __TMPM_376__ */


/* Needed stack size calculations */
#define TASK_OVERHEAD                   28

#define STACK_SIZE1                     ( 90 )                                  /*!< Scheduler Stack Size */
#define STACK_SIZE2                     (INIT_TASK_STACK_SIZE        + TASK_OVERHEAD)
#ifdef USE_LOAD_STATISTICS
#define STACK_SIZE3                     (SYSTEM_LOAD_TASK_STACK_SIZE + TASK_OVERHEAD)
#else
#define STACK_SIZE3                     0
#endif /* USE_LOAD_STATISTICS */
#ifdef USE_SERIAL_COMMUNICATION
#define STACK_SIZE4                     (PROTOCOL_TASK_STACK_SIZE    + TASK_OVERHEAD)
#else 
#define STACK_SIZE4                     0
#endif /* USE_SERIAL_COMMUNICATION */
#ifdef USE_CAN
#define STACK_SIZE5                     (CAN_TASK_STACK_SIZE         + TASK_OVERHEAD)
#else 
#define STACK_SIZE5                     0
#endif /* USE_CAN */
#ifdef USE_STALL_DETECT
#define STACK_SIZE6                     (STALL_TASK_STACK_SIZE       + TASK_OVERHEAD)
#else 
#define STACK_SIZE6                     0
#endif /* USE_STALL_DETECT */
#ifdef USE_RGB_LED
#define STACK_SIZE7                     (RGB_LED_TASK_STACK_SIZE     + TASK_OVERHEAD)
#else 
#define STACK_SIZE7                     0
#endif /* USE_RGB_LED */

#if defined __TMPM_370__ || defined __TMPM_376__
#ifdef USE_TURN_CONTROL
#define STACK_SIZE8                     (2*TURN_TASK_STACK_SIZE      + 2*TASK_OVERHEAD)
#else 
#define STACK_SIZE8                     0
#endif /* USE_TURN_CONTROL */
#define STACK_SIZE9                     (2*VE_TASK_STACK_SIZE        + 2*TASK_OVERHEAD)
#else
#ifdef USE_TURN_CONTROL
#define STACK_SIZE8                     (TURN_TASK_STACK_SIZE        + TASK_OVERHEAD)
#else 
#define STACK_SIZE8                     0
#endif /* USE_TURN_CONTROL */
#define STACK_SIZE9                     (VE_TASK_STACK_SIZE          + TASK_OVERHEAD)
#endif /* defined __TMPM_370_ || defined __TMPM_376__ */
#ifdef USE_UI
#define STACK_SIZE10                    (UI_TASK_STACK_SIZE          + TASK_OVERHEAD)
#else 
#define STACK_SIZE10                    0
#endif /* USE_UI */

/* Memory calculations */
#define NEEDED_HEAP_SIZE                (4* (  STACK_SIZE1  \
                                             + STACK_SIZE2  \
                                             + STACK_SIZE3  \
                                             + STACK_SIZE4  \
                                             + STACK_SIZE5  \
                                             + STACK_SIZE6  \
                                             + STACK_SIZE7  \
                                             + STACK_SIZE8  \
                                             + STACK_SIZE9  \
                                             + STACK_SIZE10 \
                                             )              \
                                        )

#define FW_RAM_NEEDED                   2950

#ifdef USE_DSO
#define BOARD_DSO_SIZE                  ((MEMORY_SIZE-FW_RAM_NEEDED-NEEDED_HEAP_SIZE)/2)
#else
#define BOARD_DSO_SIZE                  0
#endif /* USE_DSO */

/* Clock calculations */
#include BOARD_BOARD_HEADER_FILE

#define INTERNAL_OSCILLATOR_FREQUENCY   9500000

#if 0
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#pragma message ( STR(BOARD_BOARD_HEADER_FILE) )
#endif

#if   BOARD_CLOCK_GEAR_DIVIDER == CG_DIVIDE_1
#define GEAR_DIVIDER                    1
#elif BOARD_CLOCK_GEAR_DIVIDER == CG_DIVIDE_2
#define GEAR_DIVIDER                    2
#elif BOARD_CLOCK_GEAR_DIVIDER == CG_DIVIDE_4
#define GEAR_DIVIDER                    4
#elif BOARD_CLOCK_GEAR_DIVIDER == CG_DIVIDE_8
#define GEAR_DIVIDER                    8
#elif BOARD_CLOCK_GEAR_DIVIDER == CG_DIVIDE_16
#define GEAR_DIVIDER                    16
#endif

#if   BOARD_PERIPHERIAL_CLOCK_DIVIDER == CG_DIVIDE_1
#define PER_GEAR_DIVIDER                1
#elif BOARD_PERIPHERIAL_CLOCK_DIVIDER == CG_DIVIDE_2
#define PER_GEAR_DIVIDER                2
#elif BOARD_PERIPHERIAL_CLOCK_DIVIDER == CG_DIVIDE_4
#define PER_GEAR_DIVIDER                4
#elif BOARD_PERIPHERIAL_CLOCK_DIVIDER == CG_DIVIDE_8
#define PER_GEAR_DIVIDER                8
#elif BOARD_PERIPHERIAL_CLOCK_DIVIDER == CG_DIVIDE_16
#define PER_GEAR_DIVIDER                16
#endif

#ifdef BOARD_USE_PLL
#define CPU_CLOCK                       ( osc_frequency  \
                                        / GEAR_DIVIDER   \
                                        * PLL_MULTIPLIER)
#else
#define CPU_CLOCK                       ( osc_frequency  \
                                        / GEAR_DIVIDER )
#endif /*  BOARD_USE_PLL */

#define PERIPHERIAL_CLOCK               ( CPU_CLOCK / PER_GEAR_DIVIDER )

/* Checking for consistency */

#if ( !(defined USE_CONFIG_STORAGE) && !(defined USE_INTERNAL_MOTOR_PARAMS) )
#error EITHER USE_CONFIG_STORAGE OR USE_INTERNAL_MOTOR_PARAMS OR BOTH HAVE TO BE DEFINED!!!
#endif

#if ( (defined USE_UI) &&  !(defined USE_LCD) )
#error USING THE UI WITHOUT LCD MAKES NO SENSE
#endif

#if ( (defined USE_WDT) && !(defined USE_LOAD_STATISTICS))
#error USING WDT ONLY WITH USE_LOAD_STATISTICS
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
