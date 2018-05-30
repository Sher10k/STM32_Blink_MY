# Name project
#-------------------------------------------------------------------------------
TARGET = main

# Using libraries modules, periphery
#-------------------------------------------------------------------------------
# PERIPHDRIVERS += stm32f10x_adc
# PERIPHDRIVERS += stm32f10x_bkp
# PERIPHDRIVERS += stm32f10x_can
# PERIPHDRIVERS += stm32f10x_cec
# PERIPHDRIVERS += stm32f10x_crc
# PERIPHDRIVERS += stm32f10x_dbgmcu
# PERIPHDRIVERS += stm32f10x_exti
# PERIPHDRIVERS += stm32f10x_flash
# PERIPHDRIVERS += stm32f10x_fsmc
# PERIPHDRIVERS += stm32f10x_gpio
# PERIPHDRIVERS += stm32f10x_i2c
# PERIPHDRIVERS += stm32f10x_iwdg
# PERIPHDRIVERS += stm32f10x_pwr
# PERIPHDRIVERS += stm32f10x_rcc
# PERIPHDRIVERS += stm32f10x_rtc
# PERIPHDRIVERS += stm32f10x_sdio
# PERIPHDRIVERS += stm32f10x_spi
# PERIPHDRIVERS += stm32f10x_tim
# PERIPHDRIVERS += stm32f10x_usart
# PERIPHDRIVERS += stm32f10x_wwdg
# PERIPHDRIVERS += misc.c

# Defines
#-------------------------------------------------------------------------------
DEFINES += USE_STDPERIPH_DRIVER
DEFINES += STM32F10X_MD_VL

DEFINES += GCC_ARMCM3
DEFINES += VECT_TAB_FLASH

OUTPUT = output
INPUT = input

# Instruments
#-------------------------------------------------------------------------------
#AS = arm-none-eabi-gcc
AS = arm-none-eabi-as
CC = arm-none-eabi-gcc
#LD = arm-none-eabi-gcc
LD = arm-none-eabi-ld
CP = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
#OC = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
RM = rm

# Ways to CMSIS, StdPeriph Lib
#-------------------------------------------------------------------------------
CMSIS_PATH         = Libraries/CMSIS
STDPERIPH_INC_PATH = Libraries/STM32F10x_StdPeriph_Driver/inc
STDPERIPH_SRC_PATH = Libraries/STM32F10x_StdPeriph_Driver/src

# startup file
#-------------------------------------------------------------------------------
STARTUP = Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/arm/startup_stm32f10x_md_vl.s

# Paths to search for source files
#-------------------------------------------------------------------------------
SOURCEDIRS := src
SOURCEDIRS += $(CMSIS_PATH)

# Paths to search for header files
#-------------------------------------------------------------------------------
INCLUDES += .
INCLUDES += $(SOURCEDIRS)
INCLUDES += $(CMSIS_PATH)
INCLUDES += $(STDPERIPH_INC_PATH)

# Libraries
#-------------------------------------------------------------------------------
LIBPATH +=
LIBS    +=

# Compiler settings
#-------------------------------------------------------------------------------
# архитектура и система комманд
CFLAGS += -mthumb -mcpu=cortex-m3
# стандарт языка С
CFLAGS += -std=gnu99
# Выводить все предупреждения
CFLAGS += -Wall -pedantic
# Optimization  (-O0 without optimization, -Os optimization by volume)
CFLAGS += -O0
# Генерировать отладочную информацию для gdb
CFLAGS += -ggdb

CFLAGS += -fno-builtin
CFLAGS += $(addprefix -I, $(INCLUDES))
CFLAGS += $(addprefix -D, $(DEFINES))

# Linker script
#-------------------------------------------------------------------------------
LDSCR_PATH = ld-scripts
LDSCRIPT   = stm32_ld_scripts_5.ld

# Linker settings
#-------------------------------------------------------------------------------
LDFLAGS += -nostartfiles
LDFLAGS += -L$(LDSCR_PATH)
LDFLAGS += -T$(LDSCR_PATH)/$(LDSCRIPT)
LDFLAGS += $(addprefix -L, $(LIBPATH))
LDFLAGS += $(LIBS)

# Assembler settings
#-------------------------------------------------------------------------------
AFLAGS += -ahls -mapcs-32

# List of object files
#-------------------------------------------------------------------------------
OBJS += $(patsubst %.c, %.o, $(wildcard  $(addsuffix /*.c, $(SOURCEDIRS))))
OBJS += $(addprefix $(STDPERIPH_SRC_PATH)/, $(addsuffix .o, $(PERIPHDRIVERS)))
OBJS += $(patsubst %.s, %.o, $(STARTUP))

# Parhs to search for make
#-------------------------------------------------------------------------------
VPATH := $(SOURCEDIRS)

# The list of files to be deleted with the "make clean" command
#-------------------------------------------------------------------------------
TOREMOVE += *.elf *.hex *.bin *.lst *.o
TOREMOVE += $(OUTPUT)/*.elf $(OUTPUT)/*.hex $(OUTPUT)/*.bin $(OUTPUT)/*.lst $(OUTPUT)/*.o
TOREMOVE += $(addsuffix /*.o, $(SOURCEDIRS))
TOREMOVE += $(addsuffix /*.d, $(SOURCEDIRS))
TOREMOVE += $(STDPERIPH_SRC_PATH)/*.o
TOREMOVE += $(patsubst %.s, %.o, $(STARTUP))
TOREMOVE += $(TARGET)



# Build all
#-------------------------------------------------------------------------------
all: $(TARGET).hex size
#disassm info flash

# Build .bin file
#-------------------------------------------------------------------------------
$(TARGET).hex: $(TARGET).elf
	@ echo "		#...Extract binary code for firmware..."
	#@ $(CP) $(TARGET).elf $(TARGET).bin -O binary
	@ $(CP) -Oihex $(TARGET).elf $(TARGET).hex
	#$(CP) $(OUTPUT)main.elf $(OUTPUT)main.bin -O binary

# Build .lst file
#-------------------------------------------------------------------------------
disassm: $(APP).elf
	@ echo "		#...Disassemble..."
	@ $(OD) $(ODFLAGS) $(OUTPUT)$(APP).elf > $(OUTPUT)$(APP).lst

# Linkage (build .elf file)
#-------------------------------------------------------------------------------
$(TARGET).elf: $(OBJS)
	#startup.o main.o
	@ echo "		#...Linkage..."
	@ $(LD) $(LDFLAGS) $^ -o $@
	#$(LD) -o $(OUTPUT)main.elf -T stm32f103.ld $(OUTPUT)startup.o $(OUTPUT)main.o

# Compile
#-------------------------------------------------------------------------------
%.o: %.c
	@ $(CC) $(CFLAGS) -MD -c $< -o $@
%.o: %.s
	@ $(AS) $(AFLAGS) -c $< -o $@

# Сгенерированные gcc зависимости
#-------------------------------------------------------------------------------
include $(wildcart *.d)



# Показываем размер
#-------------------------------------------------------------------------------
size:
	@ echo "Size ----------------------------------------------------------------"
	@$(SZ) $(TARGET).elf

#clean
#-------------------------------------------------------------------------------
clean:
	@$(RM) -f $(TOREMOVE)
	#rm $(OUTPUT)*.bin $(OUTPUT)*.o $(OUTPUT)*.elf $(OUTPUT)*.lst


# Start
#-------------------------------------------------------------------------------
start:
	@ echo "Start ---------------------------------------------------------------"

# Stop
#-------------------------------------------------------------------------------
stop:
	@ echo "Stop ----------------------------------------------------------------"

info:
	@ echo "		#...Info about ARM..."
	st-info --version
	st-info --flash
	st-info --sram
	st-info --descr
	st-info --pagesize
	st-info --chipid
	st-info --serial
	st-info --hla-serial
	st-info --probe

flash_R:
	@ echo "		#...Flash_read_input/input.bin..."
	st-flash read $(INPUT)input.bin 0x08000000 0x10000

flash_W:
	@ echo "		#...Flash_write_output/main.bin..."
	st-flash --reset write $(OUTPUT)$(APP).bin 0x08000000


# Old
#-------------------------------------------------------------------------------

#CFLAGS = -std=gnu99 -O0 -nostdlib -nostartfiles -ffreestanding -Wall -mthumb -mcpu=cortex-m3 -I.
#AFLAGS = -g -c -mcpu=cortex-m3 -mthumb
#LDFLAGS = -std=gnu99 -g -O0 -Wall -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m3 -fsingle-precision-constant -Wdouble-promotion
#ODFLAGS = -D

#main.o:
#	@ echo "		#...Compil_main.c..."
#	$(CC) $(CFLAGS) -c main.c -o $(OUTPUT)main.o

#startup.o:
#	@ echo "		#...Compil_startup.c..."
#	$(CC) $(CFLAGS) -c startup.c -o $(OUTPUT)startup.o

#stm32f10x_rcc.o:
#	@ echo "		#...Compil_stm32f10x_rcc.c..."
#	$(CC) $(CFLAGS) -c $(SRC)stm32f10x_rcc.c -o $(OUTPUT)stm32f10x_rcc.o
