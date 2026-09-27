/* Host-test fake of the lwIP calls made by app_program/mqtt_lwip.c (see
 * lwip/apps/mqtt.h).  It records what the connection manager asks for and
 * lets the test play the broker's side. */
#include <string.h>
#include "lwip/apps/mqtt.h"

u32_t    g_fake_now_ms      = 0u;
uint32_t g_fake_connects    = 0u;
err_t    g_fake_publish_err = ERR_OK;
char     g_fake_topic[FAKE_TOPIC_MAX];
u8_t     g_fake_retain = 0u;
char     g_fake_subscribed[FAKE_TOPIC_MAX];

static mqtt_client_t *             gp_client  = NULL;
static mqtt_connection_cb_t        gp_conn_cb = NULL;
static mqtt_incoming_publish_cb_t  gp_pub_cb  = NULL;
static mqtt_incoming_data_cb_t     gp_data_cb = NULL;

static void copy_topic(char * p_dst, char const * p_src)
{
    (void) strncpy(p_dst, p_src, FAKE_TOPIC_MAX - 1u);
    p_dst[FAKE_TOPIC_MAX - 1u] = '\0';
}

u32_t sys_now(void)
{
    return g_fake_now_ms;
}

int ipaddr_aton(char const * cp, ip_addr_t * addr)
{
    (void) cp;
    addr->addr = 1u;
    return 1;
}

err_t mqtt_client_connect(mqtt_client_t * client, ip_addr_t const * ipaddr,
                          u16_t port, mqtt_connection_cb_t cb, void * arg,
                          struct mqtt_connect_client_info_t const * info)
{
    (void) ipaddr;
    (void) port;
    (void) arg;
    (void) info;
    g_fake_connects++;
    gp_client   = client;
    gp_conn_cb  = cb;
    client->b_connected = 0u;
    return ERR_OK;
}

void mqtt_disconnect(mqtt_client_t * client)
{
    client->b_connected = 0u;
}

u8_t mqtt_client_is_connected(mqtt_client_t * client)
{
    return client->b_connected;
}

void mqtt_set_inpub_callback(mqtt_client_t * client,
                             mqtt_incoming_publish_cb_t pub_cb,
                             mqtt_incoming_data_cb_t data_cb, void * arg)
{
    (void) client;
    (void) arg;
    gp_pub_cb  = pub_cb;
    gp_data_cb = data_cb;
}

err_t mqtt_sub_unsub(mqtt_client_t * client, char const * topic, u8_t qos,
                     mqtt_request_cb_t cb, void * arg, u8_t sub)
{
    (void) client;
    (void) qos;
    (void) cb;
    (void) arg;
    (void) sub;
    copy_topic(g_fake_subscribed, topic);
    return ERR_OK;
}

err_t mqtt_publish(mqtt_client_t * client, char const * topic,
                   void const * payload, u16_t payload_length, u8_t qos,
                   u8_t retain, mqtt_request_cb_t cb, void * arg)
{
    (void) client;
    (void) payload;
    (void) payload_length;
    (void) qos;
    (void) cb;
    (void) arg;

    if (ERR_OK == g_fake_publish_err)
    {
        copy_topic(g_fake_topic, topic);
        g_fake_retain = retain;
    }

    return g_fake_publish_err;
}

/* The broker answers the connection attempt (or the session drops). */
void fake_mqtt_connection(mqtt_connection_status_t status)
{
    gp_client->b_connected = (MQTT_CONNECT_ACCEPTED == status) ? 1u : 0u;
    gp_conn_cb(gp_client, NULL, status);
}

/* The broker delivers a message, in one fragment. */
void fake_mqtt_incoming(char const * topic, char const * text)
{
    size_t len = strlen(text);

    gp_pub_cb(NULL, topic, (u32_t) len);
    gp_data_cb(NULL, (u8_t const *) text, (u16_t) len, MQTT_DATA_FLAG_LAST);
}
