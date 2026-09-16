MCU            ?= 12F675
XC8_HOME       ?= /Applications/microchip/xc8/v3.10
XC8_CC         ?= $(XC8_HOME)/bin/xc8-cc
DFP            ?= /Applications/microchip/mplabx/v6.35/packs/Microchip/PIC10-12Fxxx_DFP/1.9.189/xc8

SOURCE         := main.c
TARGET         := hob2hood_pic12f675_v4_5_button
BUILD_DIR      ?= build
HEX            := $(BUILD_DIR)/$(TARGET).hex

# -O0 does not fit in the PIC12F675 program memory. Keep -O2 enabled.
# The firmware writes its measured OSCCAL value itself, so do not emit the
# XC8 startup call to the factory calibration word at address 0x3FF.
XC8_FLAGS      ?= -std=c99 -O2 -mno-osccal -Wall -Wextra -Wconversion
DEVICE_FLAGS   := -mcpu=$(MCU) -mdfp=$(DFP)

.DEFAULT_GOAL := all

.PHONY: all check-toolchain clean help

all: check-toolchain $(HEX)

check-toolchain:
	@test -x "$(XC8_CC)" || { \
		echo "XC8 compiler not found: $(XC8_CC)" >&2; \
		echo "Override XC8_HOME or XC8_CC, for example: make XC8_HOME=/path/to/xc8" >&2; \
		exit 1; \
	}
	@test -d "$(DFP)" || { \
		echo "PIC10/12F device pack not found: $(DFP)" >&2; \
		echo "Override DFP, for example: make DFP=/path/to/PIC10-12Fxxx_DFP/version/xc8" >&2; \
		exit 1; \
	}

$(HEX): $(SOURCE) Makefile | $(BUILD_DIR)
	"$(XC8_CC)" $(DEVICE_FLAGS) $(XC8_FLAGS) "$<" -o "$@"

$(BUILD_DIR):
	mkdir -p "$@"

clean:
	$(RM) -r -- "$(BUILD_DIR)"

help:
	@echo "Targets:"
	@echo "  make          Build $(HEX)"
	@echo "  make clean    Remove $(BUILD_DIR)"
	@echo "  make help     Show this help"
	@echo
	@echo "Toolchain overrides:"
	@echo "  XC8_HOME=/path/to/xc8"
	@echo "  XC8_CC=/path/to/xc8-cc"
	@echo "  DFP=/path/to/PIC10-12Fxxx_DFP/version/xc8"
	@echo "  BUILD_DIR=/path/to/output"
