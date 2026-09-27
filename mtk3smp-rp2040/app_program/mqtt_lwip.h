/** @file mqtt_lwip.h
 *
 * @brief Buddy 1 - MQTT connection manager on lwIP's MQTT client, run
 *        from the template's CYW43 service task through its application
 *        poll hook.
 *
 * NOTE: the function name is fixed by the weak hook declared in the
 * template (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c), so it cannot
 * carry this module's prefix (documented deviation from Rule 6.1.i).
 */

#ifndef MQTT_LWIP_H
#define MQTT_LWIP_H

struct netif;

void cyw43_utk_app_poll(struct netif * p_netif);

#endif /* MQTT_LWIP_H */

/*** end of file ***/
