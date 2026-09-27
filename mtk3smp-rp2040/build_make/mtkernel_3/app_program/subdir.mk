################################################################################
# micro T-Kernel 3.0 BSP  makefile  -  PicoCar application
#
# Changes from the template:
#   * Pico C SDK header paths for app_program/hal.c (header-only use).
#   * APP_MODE=<MISSION|TEST_MOTOR|TEST_MOTION|TEST_IR|TEST_BARCODE|
#               TEST_IMU|TEST_ULTRASONIC|TEST_TELEMETRY>
#   * WIFI_MQTT=1 enables lwIP TCP + lwIP's MQTT client (needs WIFI_DNS=1).
#   * Strict C99 and extra warnings for application files only (BARR-C).
#   * Stale-object guard: changing APP_MODE / WIFI_MQTT rebuilds what depends
#     on them (the template's own guard does not know these knobs).
################################################################################

APP_MODE  ?= MISSION
WIFI_MQTT ?= 0
PICO_SDK_PATH ?= ../../sdk/pico-sdk

ifeq ($(WIFI_MQTT),1)
ifneq ($(WIFI_DNS),1)
$(error WIFI_MQTT=1 requires WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1)
endif
ifeq ($(wildcard ../config/mqtt_config.h),)
$(error WIFI_MQTT=1 needs config/mqtt_config.h; copy config/mqtt_config.example.h)
endif
CFLAGS += -DTM_WIFI_MQTT=1
# lwIP's own MQTT client; the template's lwIP pattern rule compiles it.
OBJS   += ./mtkernel_3/lwip/apps/mqtt/mqtt.o
C_DEPS += ./mtkernel_3/lwip/apps/mqtt/mqtt.d
endif

APP_SDK_SRC := $(PICO_SDK_PATH)/src
APP_SDK_INC := -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
    -I"$(APP_SDK_SRC)/common/pico_base_headers/include" \
    -I"$(APP_SDK_SRC)/common/hardware_claim/include" \
    -I"$(APP_SDK_SRC)/common/pico_binary_info/include" \
    -I"$(APP_SDK_SRC)/rp2040/pico_platform/include" \
    -I"$(APP_SDK_SRC)/rp2040/hardware_regs/include" \
    -I"$(APP_SDK_SRC)/rp2040/hardware_structs/include" \
    -I"$(APP_SDK_SRC)/rp2_common/pico_platform_compiler/include" \
    -I"$(APP_SDK_SRC)/rp2_common/pico_platform_sections/include" \
    -I"$(APP_SDK_SRC)/rp2_common/pico_platform_panic/include" \
    -I"$(APP_SDK_SRC)/rp2_common/pico_platform_common/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_base/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_gpio/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_pwm/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_adc/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_resets/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_irq/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_sync/include" \
    -I"$(APP_SDK_SRC)/rp2_common/hardware_sync_spin_lock/include"

ifeq ($(wildcard $(APP_SDK_SRC)/rp2_common/hardware_pwm/include/hardware/pwm.h),)
$(error Pico SDK not found at $(PICO_SDK_PATH); the PicoCar HAL needs its headers)
endif

# BARR-C:2018: application files are strict C99 (Rule 1.1.a); extra
# warnings catch signed/unsigned mixing (5.3.c), float equality (5.4.b.iv),
# silent float->double promotion and shadowed names.  Unused parameters
# are warned about too (they are marked with (void) where a kernel or
# callback signature fixes them).  APP_STD overrides the template's
# -std=gnu11 for these files only.
# Documented deviation: hal.c is the single driver module that includes the
# Pico SDK hardware headers, which themselves require C11 (static_assert,
# anonymous unions), so that one file is compiled as C11.
APP_STD := -std=c99 -Wpedantic
mtkernel_3/app_program/hal.o: APP_STD := -std=gnu11
APP_CFLAGS := -Wall -Wextra \
    -Wsign-conversion -Wfloat-equal -Wdouble-promotion -Wshadow \
    -DAPP_MODE=APP_MODE_$(APP_MODE) \
    -DPICO_RP2040=1 -DPICO_32BIT=1 -DPICO_ON_DEVICE=1 -DPICO_BUILD=1 \
    -DPICO_NO_HARDWARE=0 -DNDEBUG

TEMP_SRCS = $(wildcard ../app_program/*.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

APP_PROFILE_ID   := app-$(APP_MODE)-mqtt$(WIFI_MQTT)
APP_PROFILE_FILE := .app_profile
ifneq ($(strip $(shell cat $(APP_PROFILE_FILE) 2>/dev/null)),$(APP_PROFILE_ID))
$(info App profile is now $(APP_PROFILE_ID); rebuilding app/lwIP objects.)
$(shell rm -f mtkernel_3/app_program/*.o mtkernel_3/app_program/*.d; \
    find mtkernel_3/lwip mtkernel_3/lib/libnet mtkernel_3/lib/libwifi \
    -name '*.o' 2>/dev/null | xargs -r rm -f; \
    rm -f $(EXE_FILE).elf; echo $(APP_PROFILE_ID) > $(APP_PROFILE_FILE))
endif

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) $(APP_STD) $(APP_CFLAGS) -D$(TARGET) $(INCPATH) $(APP_SDK_INC) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
