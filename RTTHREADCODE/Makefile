# Standalone build entry for the RT-Thread port.
# The SDK remains the source of the RT-Thread kernel and RK3568 libcpu.
PROJECT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SDK_ROOT ?= $(abspath $(PROJECT_DIR)/../../rk356x-linux-2026051515)
RTT_ROOT ?= $(SDK_ROOT)/rtos
RTT_EXEC_PATH ?= $(SDK_ROOT)/prebuilts/gcc/linux-x86/arm/gcc-arm-none-eabi/bin
RTT_CC_PREFIX ?= arm-none-eabi-

export RTT_ROOT RTT_EXEC_PATH RTT_CC_PREFIX
export PATH := $(RTT_EXEC_PATH):$(PATH)

.PHONY: all build dry-run clean
all build:
	$(if $(wildcard $(RTT_EXEC_PATH)/$(RTT_CC_PREFIX)gcc),,$(error missing $(RTT_EXEC_PATH)/$(RTT_CC_PREFIX)gcc))
	scons -C $(PROJECT_DIR) -j$${JOBS:-4}

dry-run:
	$(if $(wildcard $(RTT_EXEC_PATH)/$(RTT_CC_PREFIX)gcc),,$(error missing $(RTT_EXEC_PATH)/$(RTT_CC_PREFIX)gcc))
	scons -C $(PROJECT_DIR) -n

clean:
	scons -C $(PROJECT_DIR) -c
	rm -f $(PROJECT_DIR)/rtthread.bin $(PROJECT_DIR)/rtthread.map $(PROJECT_DIR)/rtthread.elf
