#!/bin/sh
# Run with WSL/Linux: sh tools/build_buddy3.sh [cache-directory] [uart|mqtt]
# Keep dependencies on the Linux filesystem (paths without spaces).
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD=${1:-$HOME/.cache/picocar-buddy3}
PROFILE=${2:-uart}
case "$PROFILE" in
    uart|mqtt) ;;
    *) echo "Profile must be uart or mqtt" >&2; exit 1 ;;
esac
# Also support a compiler extracted locally rather than installed system-wide.
if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    PATH="$BUILD/toolchain/usr/bin:$PATH"
    export PATH
fi
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
if [ "$PROFILE" = uart ]; then
    make -C "$BUILD/template/build_make" -j8 APP_MODE=TEST_BUDDY3 CONSOLE=uart
    cp "$BUILD/template/build_make/mtk3pico_smp0_uart.uf2" \
        "$ROOT/firmware/picocar_TEST_BUDDY3_uart.uf2"
else
    NETWORK_CONFIG="$ROOT/config"
    for config in wifi_credentials.h mqtt_config.h; do
        if [ ! -f "$NETWORK_CONFIG/$config" ]; then
            echo "Missing $NETWORK_CONFIG/$config; add network settings first." >&2
            exit 1
        fi
    done
    # Keep networking build objects and settings separate from the UART build.
    MQTT_TEMPLATE="$BUILD/template-$PROFILE"
    mkdir -p "$MQTT_TEMPLATE"
    cp -a "$BUILD/template/." "$MQTT_TEMPLATE/"
    cp "$NETWORK_CONFIG/wifi_credentials.h" "$NETWORK_CONFIG/mqtt_config.h" \
        "$MQTT_TEMPLATE/config/"
    make -C "$MQTT_TEMPLATE/build_make" -j8 APP_MODE=TEST_BUDDY3 CONSOLE=uart \
        WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1
    cp "$MQTT_TEMPLATE/build_make/mtk3pico_smp0_uart_wifi_dns.uf2" \
        "$ROOT/firmware/picocar_TEST_BUDDY3_$PROFILE.uf2"
fi
