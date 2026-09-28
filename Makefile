# hs611-min-ab — standalone A/B acquisition firmware for the Huion HS611
# (GD32F350R8T6), built as the protected vendor DFU bootloader's application at
# 0x08004000. Keep the bootloader (0x08000000..0x08003FFF) and flash over DFU.
#
# This is the isolated, forkable example of the DEBUG_MIN build: vendor-class
# USB (256c:6111), hardware-timed coil acquisition, and a runtime command
# protocol (see src/protocol.h). There are no release/HID/DEBUG_DUMP variants.

TARGET   := app
BUILD    := build
SDK      := sdk
CROSS    := arm-none-eabi-

CC       := $(CROSS)gcc
OBJCOPY  := $(CROSS)objcopy
SIZE     := $(CROSS)size

SW_BURST ?= 29

DEFS     := -DGD32F350 -DUSE_STDPERIPH_DRIVER -DHXTAL_VALUE=12000000U \
            -DSW_BURST_PERIODS=$(SW_BURST)

CPU      := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
OPT      := -Og -g3
WARN     := -Wall -Wextra
STD      := -std=gnu11
SDK_USB  := $(SDK)/GD32F3x0_usbfs_library
SDK_INC  := -isystem $(SDK)/CMSIS -isystem $(SDK)/CMSIS/GD/GD32F3x0/Include \
            -isystem $(SDK)/GD32F3x0_standard_peripheral/Include \
            -isystem $(SDK_USB)/driver/Include -isystem $(SDK_USB)/device/core/Include \
            -isystem $(SDK_USB)/ustd/common -isystem $(SDK_USB)/ustd/class/hid
INC      := -Isrc $(SDK_INC)
CFLAGS   := $(CPU) $(STD) $(DEFS) $(INC) $(OPT) $(WARN) -ffunction-sections -fdata-sections
ASFLAGS  := $(CPU) -x assembler-with-cpp
LDSCRIPT := ld/gd32f350r8_app.ld
LDFLAGS  := $(CPU) -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections \
            -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage
LDLIBS   := -lc -lm -lnosys

# USBFS device library (core + device core). The vendor-class class core is ours.
USB_SRCS := $(SDK_USB)/driver/Source/drv_usb_core.c \
            $(SDK_USB)/driver/Source/drv_usb_dev.c \
            $(SDK_USB)/driver/Source/drv_usbd_int.c \
            $(SDK_USB)/device/core/Source/usbd_core.c \
            $(SDK_USB)/device/core/Source/usbd_enum.c \
            $(SDK_USB)/device/core/Source/usbd_transc.c

C_SRCS   := $(wildcard src/*.c) \
            $(SDK)/CMSIS/GD/GD32F3x0/Source/system_gd32f3x0.c \
            $(wildcard $(SDK)/GD32F3x0_standard_peripheral/Source/*.c) \
            $(USB_SRCS)
ASM_SRCS := $(SDK)/CMSIS/GD/GD32F3x0/Source/GCC/startup_gd32f3x0.S

VPATH    := src:$(SDK)/CMSIS/GD/GD32F3x0/Source:$(SDK)/GD32F3x0_standard_peripheral/Source:$(SDK)/CMSIS/GD/GD32F3x0/Source/GCC:$(SDK_USB)/driver/Source:$(SDK_USB)/device/core/Source
OBJS     := $(addprefix $(BUILD)/,$(notdir $(C_SRCS:.c=.o))) \
            $(addprefix $(BUILD)/,$(notdir $(ASM_SRCS:.S=.o)))

.PHONY: all clean flash flash-fast dump reset size
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD):
	mkdir -p $@

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) $<

# Enter DFU first: hold the tablet button (PA1) while plugging USB in. The vendor
# bootloader enumerates as 28e9:0189 and owns 0x08000000..0x08003FFF.
#
# NOTE: dfu-util does NOT work with this bootloader (it issues SET_INTERFACE,
# which the vendor stack mishandles). tools/flash.py talks to the DFU class
# interface directly, exactly like the WebUSB flasher. It runs via `uv`.
DFU := uv run tools/flash.py

flash: $(BUILD)/$(TARGET).bin
	$(DFU) write 0x08004000 $< --verify

flash-fast: $(BUILD)/$(TARGET).bin
	$(DFU) write 0x08004000 $<

dump:
	$(DFU) dump 0x08000000 0x10000 hs611_stock_firmware.bin

reset:
	$(DFU) reset

clean:
	rm -rf $(BUILD)
