PROJECT_NAME := iot_lwip_mqtt_publisher_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
 components/libraries/button/app_button.c \
 components/libraries/util/app_error.c \
 components/libraries/fifo/app_fifo.c \
 components/libraries/scheduler/app_scheduler.c \
 components/libraries/timer/app_timer.c \
 components/libraries/timer/app_timer_appsh.c \
 components/libraries/trace/app_trace.c \
 components/libraries/mem_manager/mem_manager.c \
 components/libraries/util/nrf_assert.c \
 components/libraries/uart/retarget.c \
 external/lwip/src/app/mqtt/mqtt.c \
 external/lwip/src/port/nrf_platform_port.c \
 components/drivers_nrf/uart/app_uart_fifo.c \
 components/drivers_nrf/hal/nrf_delay.c \
 components/drivers_nrf/common/nrf_drv_common.c \
 components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
 main.c \
 nrf_log.c \
 components/ble/common/ble_advdata.c \
 components/ble/common/ble_srv_common.c \
 components/iot/context_manager/iot_context_manager.c \
 components/toolchain/system_nrf51.c \
 components/softdevice/common/softdevice_handler/softdevice_handler.c \
 components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c \


#assembly files common to all targets
ASM_SOURCE_FILES  = components/toolchain/gcc/gcc_startup_nrf51.s

#assembly files common to all targets
LIBS  = external/lwip/lib/liblwip.a
LIBS += components/iot/ble_6lowpan/lib/ble_6lowpan.a

#includes common to all targets
INC_PATHS  = -I config
INC_PATHS += -I external/lwip/src/app/mqtt
INC_PATHS += -I components/softdevice/s1xx_iot/headers
INC_PATHS += -I components/drivers_nrf/common
INC_PATHS += -I components/softdevice/common/softdevice_handler
INC_PATHS += -I external/lwip/src/include
INC_PATHS += -I components/iot/ble_6lowpan
INC_PATHS += -I components/drivers_nrf/hal
INC_PATHS += -I components/ble/common
INC_PATHS += -I components/ble/device
INC_PATHS += -I external/lwip/src/port
INC_PATHS += -I components/libraries/fifo
INC_PATHS += -I components/libraries/trace
INC_PATHS += -I components/device
INC_PATHS += -I components/drivers_nrf/config
INC_PATHS += -I components/libraries/mem_manager
INC_PATHS += -I components/drivers_nrf/uart
INC_PATHS += -I components/iot/ble_ipsp
INC_PATHS += -I components/libraries/scheduler
INC_PATHS += -I external/lwip/src/include/netif
INC_PATHS += -I components/iot/include
INC_PATHS += -I.
INC_PATHS += -I components/iot/context_manager
INC_PATHS += -I components/toolchain/gcc
INC_PATHS += -I components/drivers_nrf/gpiote
INC_PATHS += -I components/libraries/timer
INC_PATHS += -I external/lwip/src/port/arch
INC_PATHS += -I external/lwip/src/app/mqtt
INC_PATHS += -I components/libraries/button
INC_PATHS += -I components/libraries/util
INC_PATHS += -I bsp
INC_PATHS += -I components/toolchain
INC_PATHS += -I components/drivers_nrf/hal

#components/drivers_nrf/delay




OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -D__HEAP_SIZE=512
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
#CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -D__HEAP_SIZE=512
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBSP_DEFINES_ONLY
#default target - first one defined
default: clean nrf51422_xxac_s1xx_iot

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s1xx_iot

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s1xx_iot


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxac_s1xx_iot: OUTPUT_FILENAME := nrf51422_xxac_s1xx_iot
nrf51422_xxac_s1xx_iot: LINKER_SCRIPT=iot_lwip_mqtt_publisher_gcc_nrf51.ld
nrf51422_xxac_s1xx_iot: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --reset --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex

## Flash softdevice