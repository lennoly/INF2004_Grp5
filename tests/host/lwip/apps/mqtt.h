/* Host-test stand-in for the parts of lwIP that app_program/mqtt_lwip.c
 * uses (lwip/netif.h, lwip/sys.h, lwip/apps/mqtt.h, mqtt_priv.h), plus the
 * controls the test drives them with (fake_lwip.c).  Names and types
 * mirror lwIP 2.2, so this file follows lwIP's naming, not BARR-C's. */
#ifndef FAKE_LWIP_MQTT_H
#define FAKE_LWIP_MQTT_H

#include <stdint.h>

typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t   err_t;

#define ERR_OK  (0)
#define ERR_MEM (-1)
#define ERR_ARG (-16)

typedef struct { u32_t addr; } ip_addr_t;

struct netif { u8_t b_up; ip_addr_t ip; };
#define netif_is_up(p_netif)      ((p_netif)->b_up)
#define netif_is_link_up(p_netif) ((p_netif)->b_up)
#define netif_ip4_addr(p_netif)   (&(p_netif)->ip)
#define ip4_addr_isany_val(ip)    (0u == (ip).addr)

typedef struct mqtt_client_s { u8_t b_connected; } mqtt_client_t;

struct mqtt_connect_client_info_t {
    char const * client_id;
    char const * client_user;
    char const * client_pass;
    u16_t        keep_alive;
    char const * will_topic;
    char const * will_msg;
    u8_t         will_msg_len;
    u8_t         will_qos;
    u8_t         will_retain;
};

typedef enum {
    MQTT_CONNECT_ACCEPTED     = 0,
    MQTT_CONNECT_DISCONNECTED = 256
} mqtt_connection_status_t;

enum { MQTT_DATA_FLAG_LAST = 1 };

typedef void (*mqtt_connection_cb_t)(mqtt_client_t * client, void * arg,
                                     mqtt_connection_status_t status);
typedef void (*mqtt_incoming_publish_cb_t)(void * arg, char const * topic,
                                           u32_t tot_len);
typedef void (*mqtt_incoming_data_cb_t)(void * arg, u8_t const * data,
                                        u16_t len, u8_t flags);
typedef void (*mqtt_request_cb_t)(void * arg, err_t err);

u32_t sys_now(void);
int   ipaddr_aton(char const * cp, ip_addr_t * addr);
err_t mqtt_client_connect(mqtt_client_t * client, ip_addr_t const * ipaddr,
                          u16_t port, mqtt_connection_cb_t cb, void * arg,
                          struct mqtt_connect_client_info_t const * info);
void  mqtt_disconnect(mqtt_client_t * client);
u8_t  mqtt_client_is_connected(mqtt_client_t * client);
void  mqtt_set_inpub_callback(mqtt_client_t * client,
                              mqtt_incoming_publish_cb_t pub_cb,
                              mqtt_incoming_data_cb_t data_cb, void * arg);
err_t mqtt_sub_unsub(mqtt_client_t * client, char const * topic, u8_t qos,
                     mqtt_request_cb_t cb, void * arg, u8_t sub);
#define mqtt_subscribe(client, topic, qos, cb, arg) \
    mqtt_sub_unsub(client, topic, qos, cb, arg, 1)
err_t mqtt_publish(mqtt_client_t * client, char const * topic,
                   void const * payload, u16_t payload_length, u8_t qos,
                   u8_t retain, mqtt_request_cb_t cb, void * arg);

/* Test controls. */
#define FAKE_TOPIC_MAX (64u)
extern u32_t    g_fake_now_ms;
extern uint32_t g_fake_connects;             /* mqtt_client_connect() calls */
extern err_t    g_fake_publish_err;          /* what mqtt_publish() returns */
extern char     g_fake_topic[FAKE_TOPIC_MAX]; /* last accepted publish      */
extern u8_t     g_fake_retain;
extern char     g_fake_subscribed[FAKE_TOPIC_MAX];
void fake_mqtt_connection(mqtt_connection_status_t status);
void fake_mqtt_incoming(char const * topic, char const * text);

#endif /* FAKE_LWIP_MQTT_H */
