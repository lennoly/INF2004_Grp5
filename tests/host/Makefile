# Host unit tests for the pure-C PicoCar modules:  make -C tests/host
#
# The same C99 and warning flags as the firmware (see the template's
# build_make/mtkernel_3/app_program/subdir.mk), with warnings as errors.
# mqtt_lwip.c runs against a fake lwIP (lwip/), with mqtt_config.h here.
# vehicle.c, obstacle.c and servo.c run in a simulation (sim/) on a fake
# micro T-Kernel (tk/, tm/).
APP    := ../../app_program
S1     := $(APP)/subsystem1
S5     := $(APP)/subsystem5
SRCS   := test_main.c test_data.c $(APP)/car_math.c $(APP)/subsystem2/pid.c \
          $(APP)/subsystem3/line_follow.c $(APP)/subsystem3/barcode_decode.c \
          $(APP)/subsystem4/terrain.c $(S5)/avoidance.c $(S5)/obstacle.c \
          $(S5)/servo.c $(S1)/mqtt_bridge.c $(S1)/mqtt_lwip.c \
          $(S1)/command.c $(APP)/bitpack.c $(APP)/packed.c $(S1)/frames.c \
          hal_stub.c timing_stub.c fault_stub.c lwip/fake_lwip.c \
          sim/vehicle_sim.c
CFLAGS := -std=c99 -Wpedantic -Wall -Wextra -Wsign-conversion \
          -Wfloat-equal -Wdouble-promotion -Wshadow -Werror -O1 -I$(APP) \
          $(addprefix -I,$(wildcard $(APP)/*/)) -I. \
          -DTM_WIFI_MQTT=1

run: picocar_tests
	./picocar_tests

picocar_tests: $(SRCS) $(wildcard $(APP)/*.h $(APP)/*/*.h) $(APP)/vehicle.c \
               test_main.h hal_stub.h timing_stub.h sim/vehicle_sim.h \
               tk/tkernel.h tm/tmonitor.h \
               mqtt_config.h lwip/apps/mqtt.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) -lm

clean:
	rm -f picocar_tests picocar_tests.exe
.PHONY: run clean
