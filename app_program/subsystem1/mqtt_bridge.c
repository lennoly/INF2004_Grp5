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

#define TX_SLOTS    (6u)
#define RX_SLOTS    (4u)
#define TOPIC_MASK  (0x3u) /* TX slot header: bits 0-1 topic  */
#define LEN_SHIFT   (2u)   /*                 bits 2-7 length */
#define LEN_MASK    (0x3Fu)
#define COUNT8_MAX  (0xFFu)
#define WIDE_MASK   (0xFFFFu)

/* One TX slot: a 1-byte header (topic + length) and one frame. */
typedef struct
{
    uint8_t meta;
    uint8_t data[MQTT_BRIDGE_PAYLOAD_MAX];
} tx_slot_t;

/* One RX slot: length and one command frame. */
typedef struct
{
    uint8_t len;
    uint8_t data[MQTT_BRIDGE_CMD_MAX];
} rx_slot_t;

/* Link statistics written only by the CYW43 task (network side): packed,
   one writer for every word. */
typedef struct
{
    uint32_t published  : 16; /* wraps                    */
    uint32_t commands   : 16; /* wraps                    */
    uint32_t reconnects : 8;  /* saturating               */
    uint32_t rejoins    : 8;
    uint32_t restarts   : 8;
    uint32_t attempts   : 8;
    uint32_t why        : 4;  /* index into g_why_texts   */
    uint32_t wifi       : 3;  /* index into g_wifi_texts  */
    uint32_t address;         /* IPv4, network byte order */
} net_stats_t;

/* Status texts, stored as their index (unknown text: index 0). */
static char const * const g_why_texts[] = {
    "-",
    "connected",
    "closed by broker or network (TCP reset)",
    "refused: protocol version",
    "refused: client id",
    "refused: broker unavailable",
    "refused: bad username or password",
    "refused: not authorised (allow_anonymous true?)",
    "broker did not reply to CONNECT",
    "no answer from broker (firewall? address?)",
    "could not start (bad broker address?)"};
static char const * const g_wifi_texts[] = {
    "-",
    "joined",
    "joined, waiting for an IP address",
    "joining",
    "hotspot not found (2.4 GHz? name?)",
    "wrong WiFi password",
    "join failed",
    "not joined"};

#define WHY_COUNT  (sizeof(g_why_texts) / sizeof(g_why_texts[0]))
#define WIFI_COUNT (sizeof(g_wifi_texts) / sizeof(g_wifi_texts[0]))

/* Ring storage is shared with the CYW43 task: volatile (Rule 1.8.c). */
static volatile tx_slot_t   g_tx[TX_SLOTS];
static volatile uint32_t    g_tx_head = 0u;
static volatile uint32_t    g_tx_tail = 0u;
static volatile rx_slot_t   g_rx[RX_SLOTS];
static volatile uint32_t    g_rx_head = 0u;
static volatile uint32_t    g_rx_tail = 0u;
static volatile net_stats_t g_net;
static volatile uint8_t     g_link        = 0u; /* mqtt_bridge_link_t */
static volatile uint16_t    g_dropped     = 0u; /* application side   */
static volatile bool        gb_restart_ok = true; /* car not driving  */

static uint32_t text_code(char const * p_text, char const * const * pp_texts,
                          uint32_t count);

/*!
 * @brief Queue a frame for publishing (application side).
 *
 * @param[in] topic  Topic to publish on.
 * @param[in] p_data Frame bytes.
 * @param[in] len    Frame length.
 *
 * @return false if the frame was dropped (ring full, frame too long or
 *         not connected).
 */
bool
mqtt_bridge_publish (mqtt_bridge_topic_t topic, uint8_t const * p_data,
                     uint32_t len)
{
    uint32_t next     = (g_tx_head + 1u) % TX_SLOTS;
    uint32_t idx      = 0u;
    bool     b_queued = false;

    if ((next == g_tx_tail) || (0u == len) || (len > MQTT_BRIDGE_PAYLOAD_MAX)
        || (LINK_CONNECTED != g_link))
    {
        g_dropped++;
    }
    else
    {
        /* Cast: topic (2 bits) and length (6 bits) fill one byte. */
        g_tx[g_tx_head].meta =
            (uint8_t) (((uint32_t) topic & TOPIC_MASK) | (len << LEN_SHIFT));

        for (idx = 0u; idx < len; idx++)
        {
            g_tx[g_tx_head].data[idx] = p_data[idx];
        }

        hal_memory_barrier();
        g_tx_head = next;
        b_queued  = true;
    }

    return (b_queued);
}

/*!
 * @brief Take the oldest command frame received on the "cmd" topic
 *        (application side).
 *
 * @param[out] p_out Frame bytes.
 * @param[in]  size  Size of p_out in bytes.
 * @param[out] p_len Frame length.
 *
 * @return true if a frame was copied.
 */
bool
mqtt_bridge_get_command (uint8_t * p_out, uint32_t size, uint32_t * p_len)
{
    uint32_t idx     = 0u;
    uint32_t len     = 0u;
    bool     b_found = false;

    if (g_rx_tail != g_rx_head)
    {
        hal_memory_barrier();
        len = g_rx[g_rx_tail].len;
        len = (len > size) ? size : len;

        for (idx = 0u; idx < len; idx++)
        {
            p_out[idx] = g_rx[g_rx_tail].data[idx];
        }

        *p_len = len;
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
    /* Cast: g_link only ever holds a mqtt_bridge_link_t value. */
    p_out->link          = (mqtt_bridge_link_t) g_link;
    p_out->published     = g_net.published;
    p_out->dropped       = g_dropped;
    p_out->commands      = g_net.commands;
    p_out->reconnects    = g_net.reconnects;
    p_out->rejoins       = g_net.rejoins;
    p_out->wifi_restarts = g_net.restarts;
    p_out->attempts      = g_net.attempts;
    p_out->address       = g_net.address;
    p_out->why_code      = g_net.why;
    p_out->wifi_code     = g_net.wifi;
    p_out->p_why  = (g_net.why < WHY_COUNT) ? g_why_texts[g_net.why] : "-";
    /* The 3-bit field covers exactly the 8 WiFi texts. */
    p_out->p_wifi = g_wifi_texts[g_net.wifi];
}

/*!
 * @brief Copy the oldest queued frame without removing it (network side);
 *        mqtt_bridge_drop_tx() removes it once it has been sent.
 *
 * @param[out] p_topic Topic.
 * @param[out] p_data  Frame bytes.
 * @param[in]  size    Size of p_data in bytes.
 * @param[out] p_len   Frame length.
 *
 * @return true if a frame was copied.
 */
bool
mqtt_bridge_peek_tx (mqtt_bridge_topic_t * p_topic, char * p_data,
                     uint32_t size, uint32_t * p_len)
{
    uint32_t idx     = 0u;
    uint32_t len     = 0u;
    uint32_t meta    = 0u;
    bool     b_found = false;

    if (g_tx_tail != g_tx_head)
    {
        hal_memory_barrier();
        meta = g_tx[g_tx_tail].meta;
        len  = (meta >> LEN_SHIFT) & LEN_MASK;
        len  = (len > size) ? size : len;

        for (idx = 0u; idx < len; idx++)
        {
            /* Cast: lwIP takes the payload as char; the bytes are copied
               unchanged. */
            p_data[idx] = (char) g_tx[g_tx_tail].data[idx];
        }

        /* Cast: the two bits hold a topic written by mqtt_bridge_publish. */
        *p_topic = (mqtt_bridge_topic_t) (meta & TOPIC_MASK);
        *p_len   = len;
        b_found  = true;
    }

    return (b_found);
}

/*!
 * @brief Remove the oldest queued frame after it was sent (network side).
 *        Call only after mqtt_bridge_peek_tx() returned true.
 */
void
mqtt_bridge_drop_tx (void)
{
    hal_memory_barrier();
    g_tx_tail = (g_tx_tail + 1u) % TX_SLOTS;
    g_net.published = (g_net.published + 1u) & WIDE_MASK;
}

/*!
 * @brief Queue a received command frame (network side).  Dropped if the
 *        ring is full or the frame is longer than MQTT_BRIDGE_CMD_MAX.
 *
 * @param[in] p_data Frame bytes.
 * @param[in] len    Number of bytes.
 */
void
mqtt_bridge_put_command (uint8_t const * p_data, uint32_t len)
{
    uint32_t next = (g_rx_head + 1u) % RX_SLOTS;
    uint32_t idx  = 0u;

    if ((next != g_rx_tail) && (len > 0u) && (len <= MQTT_BRIDGE_CMD_MAX))
    {
        for (idx = 0u; idx < len; idx++)
        {
            g_rx[g_rx_head].data[idx] = p_data[idx];
        }

        /* Cast: len <= MQTT_BRIDGE_CMD_MAX (15). */
        g_rx[g_rx_head].len = (uint8_t) len;
        hal_memory_barrier();
        g_rx_head      = next;
        g_net.commands = (g_net.commands + 1u) & WIDE_MASK;
    }
}

/*!
 * @brief Record the connection state.  Written once by telemetry_init()
 *        before the CYW43 task runs, then only by the network side.
 *
 * @param[in] link New link state.
 */
void
mqtt_bridge_set_link (mqtt_bridge_link_t link)
{
    /* Cast: the link states are 0..4. */
    g_link = (uint8_t) link;
}

/*!
 * @brief Count one reconnection attempt (network side; saturating).
 */
void
mqtt_bridge_count_reconnect (void)
{
    g_net.reconnects = (g_net.reconnects < COUNT8_MAX)
                           ? (g_net.reconnects + 1u)
                           : COUNT8_MAX;
}

/*!
 * @brief Count one WiFi re-join attempt (network side; saturating).
 */
void
mqtt_bridge_count_rejoin (void)
{
    g_net.rejoins =
        (g_net.rejoins < COUNT8_MAX) ? (g_net.rejoins + 1u) : COUNT8_MAX;
}

/*!
 * @brief Count one WiFi chip power-cycle (network side; saturating).
 */
void
mqtt_bridge_count_wifi_restart (void)
{
    g_net.restarts =
        (g_net.restarts < COUNT8_MAX) ? (g_net.restarts + 1u) : COUNT8_MAX;
}

/*!
 * @brief Allow or forbid WiFi chip restarts, which block the CYW43 task
 *        (above motion) while the firmware loads (application side).
 *
 * @param[in] b_ok true while the car is not driving.
 */
void
mqtt_bridge_set_restart_ok (bool b_ok)
{
    gb_restart_ok = b_ok;
}

/*!
 * @brief Whether a WiFi chip restart is allowed now (network side).
 *
 * @return true while the car is not driving.
 */
bool
mqtt_bridge_restart_ok (void)
{
    return (gb_restart_ok);
}

/*!
 * @brief Count one broker connection attempt (network side; saturating).
 */
void
mqtt_bridge_count_attempt (void)
{
    g_net.attempts =
        (g_net.attempts < COUNT8_MAX) ? (g_net.attempts + 1u) : COUNT8_MAX;
}

/*!
 * @brief Record the board's own IPv4 address (network side).
 *
 * @param[in] address Address in network byte order, as lwIP stores it.
 */
void
mqtt_bridge_set_address (uint32_t address)
{
    g_net.address = address;
}

/*!
 * @brief Record how the last connection attempt ended (network side),
 *        as a 4-bit code.
 *
 * @param[in] p_why One of the texts in g_why_texts.
 */
void
mqtt_bridge_set_why (char const * p_why)
{
    g_net.why = text_code(p_why, g_why_texts, WHY_COUNT);
}

/*!
 * @brief Record the WiFi connection state (network side), as a 3-bit
 *        code.
 *
 * @param[in] p_wifi One of the texts in g_wifi_texts.
 */
void
mqtt_bridge_set_wifi (char const * p_wifi)
{
    g_net.wifi = text_code(p_wifi, g_wifi_texts, WIFI_COUNT);
}

/*!
 * @brief Index of a status text in its table.
 *
 * @param[in] p_text  Text (may be NULL).
 * @param[in] pp_texts Table.
 * @param[in] count   Number of entries.
 *
 * @return Index, or 0 if the text is not in the table.
 */
static uint32_t
text_code (char const * p_text, char const * const * pp_texts,
           uint32_t count)
{
    uint32_t idx  = 0u;
    uint32_t code = 0u;

    for (idx = 1u; (NULL != p_text) && (idx < count); idx++)
    {
        if ((0u == code) && (0 == strcmp(p_text, pp_texts[idx])))
        {
            code = idx;
        }
    }

    return (code);
}

/*** end of file ***/
