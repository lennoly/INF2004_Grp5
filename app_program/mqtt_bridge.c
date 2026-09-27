/** @file mqtt_bridge.c
 *
 * @brief Buddy 1 - single-producer / single-consumer rings (see
 *        mqtt_bridge.h).
 *
 * The producer writes the slot, then a memory barrier, then advances the
 * head; the consumer reads the slot, then a barrier, then advances the
 * tail.  This is correct on the single-core profile and across the two
 * RP2040 cores (template rule 3).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "mqtt_bridge.h"

#define TX_SLOTS   (6u)
#define RX_SLOTS   (4u)
#define TERMINATOR ('\0')

typedef struct
{
    mqtt_bridge_topic_t topic;
    uint32_t            len;
    char                data[MQTT_BRIDGE_PAYLOAD_MAX];
} tx_slot_t;

/* Ring storage is shared with the CYW43 task: volatile (Rule 1.8.c). */
static volatile tx_slot_t           g_tx[TX_SLOTS];
static volatile uint32_t            g_tx_head = 0u;
static volatile uint32_t            g_tx_tail = 0u;
static volatile char                g_rx[RX_SLOTS][MQTT_BRIDGE_CMD_MAX];
static volatile uint32_t            g_rx_head = 0u;
static volatile uint32_t            g_rx_tail = 0u;
static volatile mqtt_bridge_stats_t g_stats;

/*!
 * @brief Queue a message for publishing (application side).
 *
 * @param[in] topic  Topic to publish on.
 * @param[in] p_json NUL-terminated payload.
 *
 * @return false if the message was dropped (ring full, payload too long
 *         or not connected).
 */
bool
mqtt_bridge_publish (mqtt_bridge_topic_t topic, char const * p_json)
{
    uint32_t next     = (g_tx_head + 1u) % TX_SLOTS;
    uint32_t len      = 0u;
    uint32_t idx      = 0u;
    bool     b_queued = false;

    /* Cast: payloads are built in buffers far shorter than 2^32. */
    len = (uint32_t) strlen(p_json);

    if ((next == g_tx_tail) || (len >= MQTT_BRIDGE_PAYLOAD_MAX)
        || (LINK_CONNECTED != g_stats.link))
    {
        g_stats.dropped++;
    }
    else
    {
        g_tx[g_tx_head].topic = topic;
        g_tx[g_tx_head].len   = len;

        for (idx = 0u; idx < len; idx++)
        {
            g_tx[g_tx_head].data[idx] = p_json[idx];
        }

        hal_memory_barrier();
        g_tx_head = next;
        b_queued  = true;
    }

    return (b_queued);
}

/*!
 * @brief Take the oldest command received on the "cmd" topic
 *        (application side).
 *
 * @param[out] p_out Command text, NUL-terminated.
 * @param[in]  size  Size of p_out in bytes.
 *
 * @return true if a command was copied.
 */
bool
mqtt_bridge_get_command (char * p_out, uint32_t size)
{
    uint32_t idx     = 0u;
    bool     b_found = false;

    if ((g_rx_tail != g_rx_head) && (size > 0u))
    {
        hal_memory_barrier();

        while ((idx < (size - 1u)) && (TERMINATOR != g_rx[g_rx_tail][idx]))
        {
            p_out[idx] = g_rx[g_rx_tail][idx];
            idx++;
        }

        p_out[idx] = TERMINATOR;
        hal_memory_barrier();
        g_rx_tail = (g_rx_tail + 1u) % RX_SLOTS;
        b_found   = true;
    }

    return (b_found);
}

/*!
 * @brief Copy the link state and counters.
 *
 * @param[out] p_out Statistics.
 */
void
mqtt_bridge_get_stats (mqtt_bridge_stats_t * p_out)
{
    p_out->link       = g_stats.link;
    p_out->published  = g_stats.published;
    p_out->dropped    = g_stats.dropped;
    p_out->commands   = g_stats.commands;
    p_out->reconnects = g_stats.reconnects;
}

/*!
 * @brief Copy the oldest queued message without removing it (network
 *        side); mqtt_bridge_drop_tx() removes it once it has been sent.
 *
 * @param[out] p_topic Topic.
 * @param[out] p_data  Payload (not NUL-terminated).
 * @param[in]  size    Size of p_data in bytes.
 * @param[out] p_len   Payload length.
 *
 * @return true if a message was copied.
 */
bool
mqtt_bridge_peek_tx (mqtt_bridge_topic_t * p_topic, char * p_data,
                     uint32_t size, uint32_t * p_len)
{
    uint32_t idx     = 0u;
    uint32_t len     = 0u;
    bool     b_found = false;

    if (g_tx_tail != g_tx_head)
    {
        hal_memory_barrier();
        len = g_tx[g_tx_tail].len;
        len = (len > size) ? size : len;

        for (idx = 0u; idx < len; idx++)
        {
            p_data[idx] = g_tx[g_tx_tail].data[idx];
        }

        *p_topic = g_tx[g_tx_tail].topic;
        *p_len   = len;
        b_found  = true;
    }

    return (b_found);
}

/*!
 * @brief Remove the oldest queued message after it was sent (network
 *        side).  Call only after mqtt_bridge_peek_tx() returned true.
 */
void
mqtt_bridge_drop_tx (void)
{
    hal_memory_barrier();
    g_tx_tail = (g_tx_tail + 1u) % TX_SLOTS;
    g_stats.published++;
}

/*!
 * @brief Queue a received command (network side).  Dropped if the ring is
 *        full; truncated to MQTT_BRIDGE_CMD_MAX - 1 characters.
 *
 * @param[in] p_data Command bytes (not NUL-terminated).
 * @param[in] len    Number of bytes.
 */
void
mqtt_bridge_put_command (uint8_t const * p_data, uint32_t len)
{
    uint32_t next  = (g_rx_head + 1u) % RX_SLOTS;
    uint32_t count = len;
    uint32_t idx   = 0u;

    if (next != g_rx_tail)
    {
        count =
            (count >= MQTT_BRIDGE_CMD_MAX) ? (MQTT_BRIDGE_CMD_MAX - 1u) : count;

        for (idx = 0u; idx < count; idx++)
        {
            /* Cast: command text is 7-bit ASCII, identical as char. */
            g_rx[g_rx_head][idx] = (char) p_data[idx];
        }

        g_rx[g_rx_head][count] = TERMINATOR;
        hal_memory_barrier();
        g_rx_head = next;
        g_stats.commands++;
    }
}

/*!
 * @brief Record the connection state (network side).
 *
 * @param[in] link New link state.
 */
void
mqtt_bridge_set_link (mqtt_bridge_link_t link)
{
    g_stats.link = link;
}

/*!
 * @brief Count one reconnection attempt (network side).
 */
void
mqtt_bridge_count_reconnect (void)
{
    g_stats.reconnects++;
}

/*** end of file ***/
