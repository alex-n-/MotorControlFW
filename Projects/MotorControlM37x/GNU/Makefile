CC=arm-none-eabi-gcc.exe
AS=arm-none-eabi-as.exe

CFLAGS=-c -Wall -mthumb -mcpu=cortex-m3
OBJDIR=obj
LDFLAGS=
BOARD?=M374STK

ifeq ($(BOARD),M374STK)
CPU=M374
SYMBOLS= -DBOARD_M374STK -D__TMPM_374__
endif
ifeq ($(BOARD),M372STK)
CPU=M372
SYMBOLS= -DBOARD_M372STK -D__TMPM_372__ -DDEBUG
endif
ifeq ($(BOARD),HitexM370)
CPU=M370
SYMBOLS= -DBOARD_HitexM370 -D__TMPM_370__ -DDEBUG
endif
ifeq ($(BOARD),M37Sigma)
CPU=M373
SYMBOLS= -DBOARD_M37Sigma -D__TMPM_373__ -DDEBUG
endif

INCLUDE=	-I../../../Source/Board/Baseboard/M374Baseboard/include				\
			-I../../../Source/Board/CPU_Board/HitexM370/include					\
			-I../../../Source/Board/CPU_Board/M37Sigma/include					\
			-I../../../Source/Board/Powerboard									\
			-I../../../Source/CMSIS/TX03_Periph_Driver/inc						\
			-I../../../Source/CMSIS/TX03_CMSIS/									\
			-I../../../Source/FreeRTOS/Source/include							\
			-I../../../Source/FreeRTOS/Source/portable/GCC/ARM_CM3				\
			-I../../../Source/M37xHWGeneric/include								\
			-I../../../Source/M37xSWModules/include								\
			-I../../../Source/Motors											\
			-I../../../Projects/MotorControlM37x/								\
			-I.
			
ifeq ($(BOARD),M374STK)
BOARDSOURCE=../../../Source/Board/Baseboard/M374Baseboard/m374stk_board.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_can.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_eeprom.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_gain.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_led.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_pga.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_spi_device.c
endif

ifeq ($(BOARD),M372STK)
BOARDSOURCE=../../../Source/Board/Baseboard/M374Baseboard/m374stk_board.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_can.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_eeprom.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_gain.c		\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_led.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_pga.c			\
			../../../Source/Board/Baseboard/M374Baseboard/m374stk_spi_device.c
endif

ifeq ($(BOARD),HitexM370)
BOARDSOURCE=../../../Source/Board/CPU_Board/HitexM370/hitex_m370_board.c		\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_eeprom.c		\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_gain.c			\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_keypad.c		\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_lcd.c			\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_led.c			\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_spi_device.c	\
			../../../Source/Board/CPU_Board/HitexM370/hitex_m370_ui.c
endif

ifeq ($(BOARD),M37Sigma)
BOARDSOURCE=../../../Source/Board/CPU_Board/M37Sigma/m37sigma_board.c			\
			../../../Source/Board/CPU_Board/M37Sigma/m37sigma_gain.c			\
			../../../Source/Board/CPU_Board/M37Sigma/m37sigma_led.c				\
			../../../Source/Board/CPU_Board/M37Sigma/m37sigma_rgb_led.c
endif

ifeq ($(CPU),M370)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm370_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM370.c
endif

ifeq ($(CPU),M372)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm372_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM372.c
endif			

ifeq ($(CPU),M373)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm373_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM373.c
endif
			
ifeq ($(CPU),M374)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm374_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM374.c

STARTUP=	../../../Source/CMSIS/TX03_CMSIS/startup/arm/startup_TMPM374.s
endif

ifeq ($(CPU),M375)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_enc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_sbi.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm375_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM375.c
endif

ifeq ($(CPU),M376)
CMSIS=		../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_adc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_cg.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_fc.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_gpio.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_ofd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_tmrb.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_uart.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_vltd.c			\
			../../../Source/CMSIS/TX03_Periph_Driver/src/tmpm376_wdt.c			\
			../../../Source/CMSIS/TX03_CMSIS/system_TMPM376.c
endif

#CMSIS+=		../../../Source/CMSIS/TX03_CMSIS/startup/Atollic/startup.c


FREERTOS=	../../../Source/FreeRTOS/Source/croutine.c							\
			../../../Source/FreeRTOS/Source/list.c								\
			../../../Source/FreeRTOS/Source/queue.c								\
			../../../Source/FreeRTOS/Source/tasks.c								\
			../../../Source/FreeRTOS/Source/timers.c							\
			../../../Source/FreeRTOS/Source/portable/MemMang/heap_2.c			\
			../../../Source/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c			\

HWGENERIC=	../../../Source/M37xHWGeneric/adc.c									\
			../../../Source/M37xHWGeneric/amp_cmp.c								\
			../../../Source/M37xHWGeneric/encoder.c								\
			../../../Source/M37xHWGeneric/mcp2515.c								\
			../../../Source/M37xHWGeneric/pmd.c									\
			../../../Source/M37xHWGeneric/spi.c									\
			../../../Source/M37xHWGeneric/ve.c

SWMODULES=	../../../Source/M37xSWModules/action.c								\
			../../../Source/M37xSWModules/board.c								\
			../../../Source/M37xSWModules/compiled_in_motor.c					\
			../../../Source/M37xSWModules/config_storage.c						\
			../../../Source/M37xSWModules/crc8.c								\
			../../../Source/M37xSWModules/debug.c								\
			../../../Source/M37xSWModules/dso.c									\
			../../../Source/M37xSWModules/external_speed_control.c				\
			../../../Source/M37xSWModules/hsdso.c								\
			../../../Source/M37xSWModules/hv_serial_communication.c				\
			../../../Source/M37xSWModules/i2c_master_bitbanging.c				\
			../../../Source/M37xSWModules/load_statistics.c						\
			../../../Source/M37xSWModules/protocol.c							\
			../../../Source/M37xSWModules/stall_detect.c						\
			../../../Source/M37xSWModules/storage_eeprom.c						\
			../../../Source/M37xSWModules/storage_flash.c						\
			../../../Source/M37xSWModules/temperature_control.c					\
			../../../Source/M37xSWModules/turn_control.c						\
			../../../Source/M37xSWModules/user_callbacks.c

SOURCES=	../../../Projects/MotorControlM37x/main.c ../../../Projects/MotorControlM37x/GNU/stub.c $(SWMODULES) $(HWGENERIC) $(FREERTOS) $(CMSIS) $(BOARDSOURCE)

OBJECTS=$(SOURCES:.c=.o)
BIN=MotorControl.bin

all: $(SOURCES) $(START) $(BIN)
	
$(BIN): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

$(START): $(STARTUP)
	$(AS) $(STARTUP) -o$@
	
.s.o:
	$(AS) $< 
	
.c.o:
	$(CC) $(SYMBOLS) $(INCLUDE) $(CFLAGS) $< -o $@

clean:
	rm -rf $(OBJDIR)/*.o $(BIN)