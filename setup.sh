#!/bin/sh
# Fetch what this project builds on, next to it (both are git-ignored):
#
#   template/      the course template (sirfonzie/mtk3smp-rp2040) at the
#                  commit docs/template_changes.patch was made against, with
#                  that patch applied and template/app_program linked to our
#                  app_program/
#   sdk/pico-sdk   Pico SDK 2.2.0 and the three submodules the template uses
#                  (skipped when PICO_SDK_PATH is set)
#
# Run once; running it again skips what is already there.  Needs git, and
# a system with symbolic links (macOS, Linux or WSL).
set -e
cd "$(dirname "$0")"

TEMPLATE_URL=https://github.com/sirfonzie/mtk3smp-rp2040.git
TEMPLATE_REV=2e8e6ad   # the template version our patch was made against
SDK_TAG=2.2.0          # the template is qualified against this SDK

if [ ! -d template ]; then
    git clone -q "$TEMPLATE_URL" template
    git -C template checkout -q "$TEMPLATE_REV"
    git -C template apply ../docs/template_changes.patch
    cp config/mqtt_config.example.h template/config/
    rm -r template/app_program
    ln -s ../app_program template/app_program
fi

if [ -z "$PICO_SDK_PATH" ] && [ ! -d sdk/pico-sdk ]; then
    git -c advice.detachedHead=false clone -q --depth 1 -b "$SDK_TAG" \
        https://github.com/raspberrypi/pico-sdk.git sdk/pico-sdk
    git -C sdk/pico-sdk submodule update -q --init --depth 1 \
        lib/tinyusb lib/cyw43-driver lib/lwip
fi

echo "Ready.  Build with:  gmake -C template/build_make -j8 APP_MODE=MISSION CONSOLE=usb_cdc"
