# Host unit tests for the pure-C PicoCar modules:  make -C tests/host
#
# The same C99 and warning flags as the firmware (see the template's
# build_make/mtkernel_3/app_program/subdir.mk), with warnings as errors.
# mqtt_lwip.c runs against a fake lwIP (lwip/), with mqtt_config.h here.
APP    := ../../app_program
S1     := $(APP)/subsystem1
SRCS   := test_main.c $(APP)/car_math.c $(APP)/subsystem2/pid.c \
          $(APP)/subsystem3/line_follow.c $(APP)/subsystem3/barcode_decode.c \
          $(APP)/subsystem4/terrain.c $(APP)/subsystem5/avoidance.c \
          $(S1)/mqtt_bridge.c $(S1)/mqtt_lwip.c $(S1)/command.c \
          hal_stub.c lwip/fake_lwip.c
CFLAGS := -std=c99 -Wpedantic -Wall -Wextra -Wsign-conversion \
          -Wfloat-equal -Wdouble-promotion -Wshadow -Werror -O1 -I$(APP) \
          $(addprefix -I,$(wildcard $(APP)/*/)) -I. \
          -DTM_WIFI_MQTT=1

run: picocar_tests
	./picocar_tests

picocar_tests: $(SRCS) $(wildcard $(APP)/*.h $(APP)/*/*.h) test_main.h hal_stub.h \
               mqtt_config.h lwip/apps/mqtt.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f picocar_tests
.PHONY: run clean
