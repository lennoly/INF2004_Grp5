/** @file mqtt_bridge.h
 *
 * @brief Buddy 1 - communication API between the application tasks and
 *        the lwIP / MQTT client that runs inside the CYW43 service task.
 *
 * Two lock-free single-producer / single-consumer rings decouple the two
 * worlds (which may not share headers, see hal.h):
 *   TX: telemetry task -> MQTT client     (publish requests)
 *   RX: MQTT client    -> telemetry task  (commands from the "cmd" topic)
 * Plain C only: included from both kernel-side and lwIP-side files.
 */

#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#define MQTT_BRIDGE_PAYLOAD_MAX (512u)
#define MQTT_BRIDGE_CMD_MAX     (64u)

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

typedef struct
{
    mqtt_bridge_link_t link;
    uint32_t           published;
    uint32_t           dropped; /* TX ring full or link down */
    uint32_t           commands;
    uint32_t           reconnects;
} mqtt_bridge_stats_t;

/* Application side. */
bool mqtt_bridge_publish(mqtt_bridge_topic_t topic, char const * p_json);
bool mqtt_bridge_get_command(char * p_out, uint32_t size);
void mqtt_bridge_get_stats(mqtt_bridge_stats_t * p_out);

/* Network side. */
bool mqtt_bridge_peek_tx(mqtt_bridge_topic_t * p_topic, char * p_data,
                         uint32_t size, uint32_t * p_len);
void mqtt_bridge_drop_tx(void);
void mqtt_bridge_put_command(uint8_t const * p_data, uint32_t len);
void mqtt_bridge_set_link(mqtt_bridge_link_t link);
void mqtt_bridge_count_reconnect(void);

#endif /* MQTT_BRIDGE_H */

/*** end of file ***/
