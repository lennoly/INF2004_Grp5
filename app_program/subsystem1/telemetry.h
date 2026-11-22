/** @file telemetry.h
 *
 * @brief Buddy 1 - telemetry framework: builds binary frames (frames.h)
 *        from every subsystem, publishes them through the MQTT bridge (or
 *        the console when built without WIFI_MQTT) and executes commands
 *        from picocar/<id>/cmd.
 *
 * Topics (see the MQTT section of the report):
 *   telemetry  5 Hz  full snapshot
 *   heartbeat  1 Hz  uptime, link statistics, health
 *   event      on change: state, barcode, hump, obstacle, cmd reply
 *   status     retained "online" / last-will "offline"
 *   cmd        (subscribed) command frames, see command.h:
 *              start | stop | calibrate | gains | help | timing
 *              speed= | pid= | ff= | line= | rate= | nav= | map=
 *              detect= | bypass=
 *
 * The same commands are accepted as text lines on the USB / UART serial
 * console, so gains can be tuned live in every build (no WiFi needed).
 * Every command is answered with a reply frame; gain changes are followed
 * by a gains frame with the values now in use.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* b_stream = true : mission build - telemetry, heartbeat and events.
   b_stream = false: test builds - commands only; "rate=" starts streaming. */
int32_t telemetry_init(bool b_stream);

#endif /* TELEMETRY_H */

/*** end of file ***/
