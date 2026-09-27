# Host unit tests for the pure-C PicoCar modules:  make -C tests/host
#
# The same C99 and warning flags as the firmware (see
# build_make/mtkernel_3/app_program/subdir.mk), with warnings as errors.
# mqtt_lwip.c runs against a fake lwIP (lwip/), with mqtt_config.h here.
APP    := ../../app_program
SRCS   := test_main.c $(APP)/car_math.c $(APP)/pid.c $(APP)/line_follow.c \
          $(APP)/barcode_decode.c $(APP)/terrain.c $(APP)/avoidance.c \
          $(APP)/mqtt_bridge.c $(APP)/mqtt_lwip.c $(APP)/command.c \
          hal_stub.c lwip/fake_lwip.c
CFLAGS := -std=c99 -Wpedantic -Wall -Wextra -Wsign-conversion \
          -Wfloat-equal -Wdouble-promotion -Wshadow -Werror -O1 -I$(APP) -I. \
          -DTM_WIFI_MQTT=1

run: picocar_tests
	./picocar_tests

picocar_tests: $(SRCS) $(wildcard $(APP)/*.h) test_main.h hal_stub.h \
               mqtt_config.h lwip/apps/mqtt.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f picocar_tests
.PHONY: run clean
