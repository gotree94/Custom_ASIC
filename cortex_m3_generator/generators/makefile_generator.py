"""Makefile 생성기"""


def generate_makefile(gen) -> str:
    """Makefile 생성"""
    mem = gen.config['memory']
    flash_start = gen.parse_size(mem['flash']['start'])
    
    return f'''\
# ============================================================================
# Makefile for {gen.chip_name}
# Generated: {gen.get_timestamp()}
# ============================================================================

# Project name
TARGET = {gen.chip_name_lower}_firmware

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# MCU configuration
CPU = -mcpu=cortex-m3
FPU = 
FLOAT-ABI = -mfloat-abi=soft

# MCU flags
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Source files
C_SOURCES = \\
    main.c \\
    system_{gen.chip_name_lower}.c \\
    {gen.chip_name_lower}_it.c

ASM_SOURCES = \\
    startup_{gen.chip_name_lower}.s

# Include paths
C_INCLUDES = \\
    -I. \\
    -ICMSIS/Include

# Compiler flags
CFLAGS = $(MCU) $(C_INCLUDES) -Wall -fdata-sections -ffunction-sections

# Debug/Release
ifdef DEBUG
CFLAGS += -g -gdwarf-2 -O0 -DDEBUG
else
CFLAGS += -Os -DNDEBUG
endif

# Assembler flags
ASFLAGS = $(MCU) -Wall -fdata-sections -ffunction-sections

# Linker flags
LDSCRIPT = {gen.chip_name_lower}.ld
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) -lc -lm -lnosys 
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections

# Build directory
BUILD_DIR = build

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# ============================================================================
# Build targets
# ============================================================================

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

# ============================================================================
# Clean and utility targets
# ============================================================================

clean:
	rm -rf $(BUILD_DIR)

# Flash using OpenOCD (adjust interface and target as needed)
flash: $(BUILD_DIR)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \\
		-c "program $(BUILD_DIR)/$(TARGET).bin verify reset exit 0x{flash_start:08X}"

# Flash using J-Link
flash-jlink: $(BUILD_DIR)/$(TARGET).hex
	JLinkExe -device CORTEX-M3 -if SWD -speed 4000 \\
		-CommanderScript flash.jlink

# Debug with GDB
debug: $(BUILD_DIR)/$(TARGET).elf
	$(PREFIX)gdb -x gdbinit $(BUILD_DIR)/$(TARGET).elf

# Print size information
size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) --format=berkeley $(BUILD_DIR)/$(TARGET).elf

.PHONY: all clean flash flash-jlink debug size
'''
