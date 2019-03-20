PROJ_NAME=stdemojtag

################################################################################
#                   SETUP TOOLS                                                #
################################################################################

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
AS      = arm-none-eabi-as

##### Preprocessor options
# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

# #defines needed when working with the STM peripherals library
DEFS    = -DUSE_HAL_DRIVER -DSTM32L476xx -DUSE_STM32L476G_DISCO_REVB

##### Assembler options

AFLAGS  = -mcpu=cortex-m4
AFLAGS += -mthumb
AFLAGS += -mthumb-interwork
AFLAGS += -mlittle-endian
AFLAGS += -mfloat-abi=hard
AFLAGS += -mfpu=fpv4-sp-d16

##### Compiler options

CFLAGS  = -ggdb
CFLAGS += -Os
CFLAGS += -Wall -Wextra -Warray-bounds -ffunction-sections -fdata-sections
CFLAGS += $(AFLAGS)

##### Linker options

# tell ld which linker file to use
# (this file is in the current directory)
LFLAGS  = -Tsrc/STM32L476VGTx_FLASH.ld --specs=rdimon.specs -Wl,--gc-sections

################################################################################
#                   SOURCE FILES DIRECTORIES                                   #
################################################################################
STM_SRC_DIR     += src

vpath %.c $(STM_SRC_DIR)
vpath %.s $(STM_SRC_DIR)

################################################################################
#                   HEADER FILES DIRECTORIES                                   #
################################################################################

# The header files we use are located here
INC_DIRS += src

################################################################################
#                   SOURCE FILES TO COMPILE                                    #
################################################################################

SRCS  += $(wildcard src/[^~]*.c)

# startup file, calls main
ASRC  = src/startup_stm32l476xx.s

OBJS  = $(patsubst src/%.c,build/%.o,$(SRCS))
OBJS += $(patsubst src/%.s,build/%.o,$(ASRC))

######################################################################
#                         SETUP TARGETS                              #
######################################################################

.PHONY: all

all: $(PROJ_NAME).elf


build/%.o : src/%.c
	@mkdir -p build
	@echo "[Compiling  ]  $^"
	@$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

build/%.o : src/%.s
	@mkdir -p build
	@echo "[Assembling ]" $^
	@$(AS) $(AFLAGS) $< -o $@

$(PROJ_NAME).elf: $(OBJS)
	@echo "[Linking    ]  $@"
	@$(CC) $(CFLAGS) $(LFLAGS) $^ -o $@
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf   $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin -r build

flash: all
	st-flash write $(PROJ_NAME).bin 0x8000000
