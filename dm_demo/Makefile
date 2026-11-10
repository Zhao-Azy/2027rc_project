# ============================================================================
#  R1 固件 —— arm-none-eabi-gcc 构建（替代 Keil MDK）
# ----------------------------------------------------------------------------
#  用法（在 dm_demo/ 目录下）：
#      make -j8            # 编译 + 链接 + 生成 hex（同时更新 MDK-ARM/R1/R1.hex）
#      make clean
#
#  源文件清单不用手抄：scripts/gen_sources.py 直接从 MDK-ARM/R1.uvprojx 读，
#  因此"Keil 工程里有哪个 .c"就是"这里编哪个 .c"，不会两边走样。
#
#  工具链：本机装在 ~/.local/opt/arm-gnu（无 root 解包的 Ubuntu 包），
#  想换工具链就 TOOLDIR=/path/to/bin make。
# ============================================================================

TOOLDIR ?= $(HOME)/.local/opt/arm-gnu/usr/bin
PYTHON  ?= python3

CC      := $(TOOLDIR)/arm-none-eabi-gcc
OBJCOPY := $(TOOLDIR)/arm-none-eabi-objcopy
SIZE    := $(TOOLDIR)/arm-none-eabi-size

OUT     := build
TARGET  := $(OUT)/R1

# Cortex-M7 + 双精度 FPU（和 Keil 工程 target 配置一致）
CPU     := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

DEFS    := -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx
INCS    := \
	-ICore/Inc \
	-IDrivers/STM32H7xx_HAL_Driver/Inc \
	-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
	-IMiddlewares/Third_Party/FreeRTOS/Source/include \
	-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
	-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
	-IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
	-IDrivers/CMSIS/Include \
	-Ibsp/chassis -Ibsp/uper -Iapp_lib -Iapp

# Ubuntu 的 newlib 包头文件单独放在 /usr/include/newlib，不在工具链默认搜索路径里
NEWLIB_INC ?= $(HOME)/.local/opt/arm-gnu/usr/include/newlib

CFLAGS  := $(CPU) $(DEFS) $(INCS) -isystem $(NEWLIB_INC) -std=gnu99 -O2 -g3 \
           -Wall -ffunction-sections -fdata-sections \
           -fno-common -MMD -MP

ASFLAGS := $(CPU) -g3

LDFLAGS := $(CPU) -T MDK-ARM/R1.ld -Wl,--gc-sections \
           -Wl,-Map=$(OUT)/R1.map -Wl,--print-memory-usage \
           -specs=nosys.specs

C_SRCS := $(shell $(PYTHON) ../scripts/gen_sources.py c)
S_SRCS := $(shell $(PYTHON) ../scripts/gen_sources.py s)
OBJS   := $(addprefix $(OUT)/,$(C_SRCS:.c=.o) $(S_SRCS:.s=.o))

.PHONY: all clean flash size check

all: check $(TARGET).hex

check:
	@$(PYTHON) ../scripts/check_project.py || true

$(OUT)/%.o: %.c
	@mkdir -p $(dir $@)
	@printf '  CC   %s\n' '$<'
	@$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/%.o: %.s
	@mkdir -p $(dir $@)
	@printf '  AS   %s\n' '$<'
	@$(CC) $(ASFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJS)
	@mkdir -p $(OUT)
	@printf '  LD   %s\n' '$@'
	@$(CC) $(LDFLAGS) $(OBJS) -lm -o $@
	@$(SIZE) $@

$(TARGET).hex: $(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@
	@mkdir -p MDK-ARM/R1
	@cp $@ MDK-ARM/R1/R1.hex
	@printf '  HEX  %s\n' 'MDK-ARM/R1/R1.hex'

size: $(TARGET).elf
	@$(SIZE) $(TARGET).elf

flash: $(TARGET).hex
	@bash ../scripts/flash.sh --force

clean:
	rm -rf $(OUT)
