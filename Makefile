APP = main

CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OD = arm-none-eabi-objdump
OC = arm-none-eabi-objcopy

OUTPUT = output/
INPUT = input/
INCLUDE = inc/
SRC = src/

CFLAGS = -std=gnu99 -O0 -nostdlib -nostartfiles -ffreestanding -Wall -mthumb -mcpu=cortex-m3 -I.
#AFLAGS = -g -c -mcpu=cortex-m3 -mthumb
#LDFLAGS = -std=gnu99 -g -O0 -Wall -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m3 -fsingle-precision-constant -Wdouble-promotion
ODFLAGS = -D

all: $(APP).bin disassm #info flash

main.bin: main.elf
	@ echo "		#...Extract binary code for firmware..."
	$(OC) $(OUTPUT)main.elf $(OUTPUT)main.bin -O binary

disassm: $(APP).elf
	@ echo "		#...Disassemble..."
	$(OD) $(ODFLAGS) $(OUTPUT)$(APP).elf > $(OUTPUT)$(APP).lst



main.elf: startup.o main.o
	@ echo "		#...Linkage..."
	$(LD) -o $(OUTPUT)main.elf -T stm32f103.ld $(OUTPUT)startup.o $(OUTPUT)main.o
#$(OUTPUT)stm32f10x_rcc.o

main.o:
	@ echo "		#...Compil_main.c..."
	$(CC) $(CFLAGS) -c main.c -o $(OUTPUT)main.o

startup.o:
	@ echo "		#...Compil_startup.c..."
	$(CC) $(CFLAGS) -c startup.c -o $(OUTPUT)startup.o

stm32f10x_rcc.o:
	@ echo "		#...Compil_stm32f10x_rcc.c..."
	$(CC) $(CFLAGS) -c $(SRC)stm32f10x_rcc.c -o $(OUTPUT)stm32f10x_rcc.o

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



clean:
	rm $(OUTPUT)*.bin $(OUTPUT)*.o $(OUTPUT)*.elf $(OUTPUT)*.lst
