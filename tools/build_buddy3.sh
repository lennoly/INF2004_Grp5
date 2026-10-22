#!/bin/sh
# Run with WSL/Linux: sh tools/build_buddy3.sh
# Keep dependencies on the Linux filesystem (paths without spaces).
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD=${1:-$HOME/.cache/picocar-buddy3}
mkdir -p "$BUILD"
if [ ! -d "$BUILD/template" ]; then
    git clone -q https://github.com/sirfonzie/mtk3smp-rp2040.git "$BUILD/template"
    git -C "$BUILD/template" checkout -q 2e8e6ad
    git -C "$BUILD/template" apply "$ROOT/docs/template_changes.patch"
    mv "$BUILD/template/app_program" "$BUILD/template/app_program_original"
    mkdir "$BUILD/template/app_program"
fi
if [ ! -d "$BUILD/sdk/pico-sdk" ]; then
    git clone -q --depth 1 -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git \
        "$BUILD/sdk/pico-sdk"
fi
git -C "$BUILD/sdk/pico-sdk" submodule update -q --init --depth 1 \
    lib/tinyusb lib/cyw43-driver lib/lwip
cp -R "$ROOT/app_program/." "$BUILD/template/app_program/"
make -C "$BUILD/template/build_make" -j8 APP_MODE=TEST_BUDDY3 CONSOLE=uart
cp "$BUILD/template/build_make/mtk3pico_smp0_uart.uf2" \
    "$ROOT/firmware/picocar_TEST_BUDDY3_uart.uf2"
