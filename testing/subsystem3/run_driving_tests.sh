#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
TEST_BIN=$(mktemp /tmp/picocar-driving-XXXXXX)
trap 'rm -f "$TEST_BIN"' EXIT
gcc -std=c99 -Wall -Wextra -Werror -O2 -ffunction-sections -fdata-sections \
    -DAPP_MODE=APP_MODE_TEST_BUDDY3 -Itesting/subsystem3/host -Iapp_program \
    -Iapp_program/subsystem2 -Iapp_program/subsystem3 \
    -Iapp_program/subsystem4 -Iapp_program/subsystem5 \
    testing/subsystem3/host/test_driving.c app_program/car_math.c \
    app_program/subsystem2/pid.c app_program/subsystem3/line_follow.c \
    app_program/subsystem3/barcode_decode.c -Wl,--gc-sections -lm -o "$TEST_BIN"
"$TEST_BIN"
