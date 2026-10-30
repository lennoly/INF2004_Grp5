/** @file mqtt_bridge.h
 *
 * @brief Buddy 1 - communication API between the application tasks and
 *        the lwIP / MQTT client that runs inside the CYW43 service task.
 *
 * Two lock-free single-producer / single-consumer rings decouple the two
 * worlds (which may not share headers, see hal.h):
 *   TX: telemetry task -> MQTT client     (binary frames to publish)
 *   RX: MQTT client    -> telemetry task  (binary command frames)
 * Plain C only: included from both kernel-side and lwIP-side files.
 *
 * Change Requirement 1: payloads are compact binary frames (frames.h), so
 * a TX slot holds 47 bytes instead of 512 and an RX slot 15 instead of 64.
 * The link statistics are packed too: the counters written by the CYW43
 * task share words, while the drop counter, written by the telemetry
 * task, keeps a word of its own so that no two tasks read-modify-write the
 * same word.  Status texts are stored as small codes.
 */

#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#define MQTT_BRIDGE_PAYLOAD_MAX (47u) /* longest frame (FRAMES_MAX_BYTES) */
#define MQTT_BRIDGE_CMD_MAX     (15u) /* longest command frame            */

typedef enum
{
    TOPIC_TELEMETRY = 0,
    TOPIC_EVENT,
    TOPIC_HEARTBEAT,
    TOPIC_STATUS,
    TOPIC_COUNT
} mqtt_bridge_topic_t;

typedef enum
{
    LINK_DISABLED = 0, /* built without WIFI_MQTT */
    LINK_WAIT_NETWORK,
    LINK_CONNECTING,
    LINK_CONNECTED,
    LINK_BACKOFF
} mqtt_bridge_link_t;

/* Unpacked copy for readers (mqtt_bridge_get_stats). */
typedef struct
{
    mqtt_bridge_link_t link;
    uint32_t           published; /* 16-bit, wraps: use differences      */
    uint32_t           dropped;   /* 16-bit, wraps: TX ring full or down */
    uint32_t           commands;  /* 16-bit, wraps                       */
    uint32_t           reconnects; /* 8-bit, saturating                  */
    uint32_t           rejoins;    /* WiFi access-point re-join attempts */
    uint32_t           wifi_restarts; /* WiFi chip power-cycles          */
    uint32_t           attempts;      /* broker connection attempts      */
    uint32_t           address;       /* own IPv4 address, network order */
    uint32_t           why_code;      /* 4-bit code of p_why             */
    uint32_t           wifi_code;     /* 3-bit code of p_wifi            */
    char const *       p_why;         /* how the last attempt ended      */
    char const *       p_wifi;        /* WiFi connection state           */
} mqtt_bridge_stats_t;

bool mqtt_bridge_publish(mqtt_bridge_topic_t topic, uint8_t const * p_data,
                         uint32_t len);
bool mqtt_bridge_get_command(uint8_t * p_out, uint32_t size, uint32_t * p_len);
void mqtt_bridge_get_stats(mqtt_bridge_stats_t * p_out);
void mqtt_bridge_set_restart_ok(bool b_ok);
bool mqtt_bridge_peek_tx(mqtt_bridge_topic_t * p_topic, char * p_data,
                         uint32_t size, uint32_t * p_len);
void mqtt_bridge_drop_tx(void);
void mqtt_bridge_put_command(uint8_t const * p_data, uint32_t len);
void mqtt_bridge_set_link(mqtt_bridge_link_t link);
void mqtt_bridge_count_reconnect(void);
void mqtt_bridge_count_rejoin(void);
void mqtt_bridge_count_wifi_restart(void);
bool mqtt_bridge_restart_ok(void);
void mqtt_bridge_count_attempt(void);
void mqtt_bridge_set_address(uint32_t address);
void mqtt_bridge_set_why(char const * p_why);
void mqtt_bridge_set_wifi(char const * p_wifi);

#endif /* MQTT_BRIDGE_H */

/*** end of file ***/
