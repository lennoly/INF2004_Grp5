/** @file mqtt_lwip.c
 *
 * @brief Buddy 1 - MQTT connection manager on lwIP's MQTT client
 *        (lwip/src/apps/mqtt), which does the MQTT 3.1.1 encoding, the
 *        keep-alive pings and the broker watchdog.
 *
 * Runs entirely inside the template's CYW43 service task through the
 * cyw43_utk_app_poll() hook, so every lwIP call (and every lwIP callback
 * below) happens in the single NO_SYS context; the module's state is
 * therefore private to that task.  Must NOT include <tk/tkernel.h>.
 *
 * Recovery: once the network is up, a connection attempt starts; each
 * attempt gets 2, 4, then 8 s (doubling, reset on success) before it is
 * abandoned and the next one starts.  A dropped session reconnects at
 * once.  Last will: picocar/<id>/status = "offline" (retained); on
 * connect the client publishes "online" (retained) and subscribes to
 * picocar/<id>/cmd.
 */

#include <stdint.h>
#include <stdbool.h>
#include "mqtt_lwip.h"

#if defined(TM_WIFI_MQTT) && TM_WIFI_MQTT

#include <string.h>
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h" /* static client, as lwIP documents */
#include "mqtt_config.h"
#include "mqtt_bridge.h"

#define TOPIC_ROOT          "picocar/" MQTT_CAR_ID "/"
#define TOPIC_CMD           TOPIC_ROOT "cmd"
#define TOPIC_STATUS        TOPIC_ROOT "status"
#define RETRY_BASE_MS       (1000u)
#define RETRY_DOUBLINGS_MAX (3u) /* attempt windows 2, 4, then 8 s */
#define QOS_0               (0u)
#define RETAIN              (1u)
#define NO_RETAIN           (0u)

/* Topic names indexed by mqtt_bridge_topic_t. */
static char const * const g_topic_names[TOPIC_COUNT] = {
    TOPIC_ROOT "telemetry", TOPIC_ROOT "event", TOPIC_ROOT "heartbeat",
    TOPIC_STATUS};

static char const g_online_msg[]  = "online";
static char const g_offline_msg[] = "offline";

/* Client state: used only in the CYW43 service task (see above). */
static mqtt_client_t g_client;
static uint32_t      g_attempt_ms   = 0u; /* start of the latest attempt   */
static uint32_t      g_tries        = 0u; /* attempts since the last success */
static bool          gb_ever_online = false;
static bool          gb_cmd_topic   = false; /* incoming publish is a command */
static uint8_t       g_cmd[MQTT_BRIDGE_CMD_MAX];
static uint32_t      g_cmd_len = 0u;
static char          g_msg[MQTT_BRIDGE_PAYLOAD_MAX];

static void start_connect(uint32_t now);
static void flush_tx(void);
static void on_connection(mqtt_client_t * p_client, void * p_arg,
                          mqtt_connection_status_t status);
static void on_publish(void * p_arg, char const * p_topic, u32_t tot_len);
static void on_data(void * p_arg, u8_t const * p_data, u16_t len, u8_t flags);

/*!
 * @brief Connection manager, called by the CYW43 service task every 2 ms.
 *
 * @param[in] p_netif The WiFi network interface.
 */
void
cyw43_utk_app_poll (struct netif * p_netif)
{
    uint32_t now   = sys_now();
    bool     b_net = (netif_is_up(p_netif)) && (netif_is_link_up(p_netif))
                 && (!ip4_addr_isany_val(*netif_ip4_addr(p_netif)));

    if (0u != mqtt_client_is_connected(&g_client))
    {
        flush_tx();
    }
    else if (!b_net)
    {
        mqtt_bridge_set_link(LINK_WAIT_NETWORK);
    }
    else if ((now - g_attempt_ms) >= (RETRY_BASE_MS << g_tries))
    {
        start_connect(now);
    }
    else
    {
        /* An attempt is running, or the car is backing off. */
    }
}

/*!
 * @brief Abandon any attempt still hanging and start a new one.
 *
 * @param[in] now Current time in ms.
 */
static void
start_connect (uint32_t now)
{
    struct mqtt_connect_client_info_t info = {.client_id   = MQTT_CLIENT_ID,
                                              .client_user = MQTT_USERNAME,
                                              .client_pass = MQTT_PASSWORD,
                                              .keep_alive  = MQTT_KEEPALIVE_S,
                                              .will_topic  = TOPIC_STATUS,
                                              .will_msg    = g_offline_msg,
                                              .will_qos    = QOS_0,
                                              .will_retain = RETAIN};
    ip_addr_t                         broker;
    err_t                             err = ERR_ARG;

    g_attempt_ms = now;
    g_tries      = (g_tries < RETRY_DOUBLINGS_MAX) ? (g_tries + 1u) : g_tries;
    mqtt_disconnect(&g_client); /* does nothing when already idle */
    mqtt_set_inpub_callback(&g_client, on_publish, on_data, NULL);

    if (0 != ipaddr_aton(MQTT_BROKER_IP, &broker))
    {
        err = mqtt_client_connect(&g_client, &broker, MQTT_BROKER_PORT,
                                  on_connection, NULL, &info);
    }

    mqtt_bridge_set_link((ERR_OK == err) ? LINK_CONNECTING : LINK_BACKOFF);
}

/*!
 * @brief Hand queued messages to the client; one it cannot take yet
 *        (buffer or request slots full) stays queued for the next poll.
 */
static void
flush_tx (void)
{
    mqtt_bridge_topic_t topic  = TOPIC_TELEMETRY;
    uint32_t            len    = 0u;
    bool                b_sent = true;

    while (b_sent && (mqtt_bridge_peek_tx(&topic, g_msg, sizeof(g_msg), &len)))
    {
        /* Cast: len < MQTT_BRIDGE_PAYLOAD_MAX (512), so it fits u16_t. */
        b_sent = (ERR_OK
                  == mqtt_publish(&g_client, g_topic_names[topic], g_msg,
                                  (u16_t) len, QOS_0, NO_RETAIN, NULL, NULL));

        if (b_sent)
        {
            mqtt_bridge_drop_tx();
        }
    }
}

/*!
 * @brief Connection result or loss (lwIP callback).
 *
 * @param[in] p_client The client.
 * @param[in] p_arg    Unused.
 * @param[in] status   MQTT_CONNECT_ACCEPTED, a refusal, or a disconnect.
 */
static void
on_connection (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void) p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_tries = 0u;
        mqtt_bridge_set_link(LINK_CONNECTED);

        if (gb_ever_online)
        {
            mqtt_bridge_count_reconnect();
        }

        gb_ever_online = true;

        /* Cast: the message is a few bytes long, so it fits u16_t. */
        (void) mqtt_publish(p_client, TOPIC_STATUS, g_online_msg,
                            (u16_t) (sizeof(g_online_msg) - 1u), QOS_0, RETAIN,
                            NULL, NULL);
        (void) mqtt_subscribe(p_client, TOPIC_CMD, QOS_0, NULL, NULL);
    }
    else
    {
        mqtt_bridge_set_link(LINK_BACKOFF);
    }
}

/*!
 * @brief Start of an incoming publish (lwIP callback).
 *
 * @param[in] p_arg   Unused.
 * @param[in] p_topic Topic of the message.
 * @param[in] tot_len Payload length (unused; on_data() collects it).
 */
static void
on_publish (void * p_arg, char const * p_topic, u32_t tot_len)
{
    (void) p_arg;
    (void) tot_len;
    gb_cmd_topic = (0 == strcmp(p_topic, TOPIC_CMD));
    g_cmd_len    = 0u;
}

/*!
 * @brief Payload fragment of an incoming publish (lwIP callback); a whole
 *        command goes to the bridge.  Bytes beyond MQTT_BRIDGE_CMD_MAX are
 *        dropped, as the bridge would truncate them anyway.
 *
 * @param[in] p_arg  Unused.
 * @param[in] p_data Fragment.
 * @param[in] len    Fragment length.
 * @param[in] flags  MQTT_DATA_FLAG_LAST on the final fragment.
 */
static void
on_data (void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    uint32_t idx = 0u;

    (void) p_arg;

    for (idx = 0u;
         gb_cmd_topic && (idx < len) && (g_cmd_len < MQTT_BRIDGE_CMD_MAX);
         idx++)
    {
        g_cmd[g_cmd_len] = p_data[idx];
        g_cmd_len++;
    }

    if (gb_cmd_topic && (0 != (flags & MQTT_DATA_FLAG_LAST)))
    {
        mqtt_bridge_put_command(g_cmd, g_cmd_len);
        gb_cmd_topic = false;
    }
}

#endif /* TM_WIFI_MQTT */

/*** end of file ***/
