/** @file mqtt_config.h
 *
 * @brief Broker settings for the host test of mqtt_lwip.c (the firmware
 *        uses config/mqtt_config.h).
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_BROKER_IP   "10.0.0.1"
#define MQTT_BROKER_PORT (1883u)
#define MQTT_CAR_ID      "t"
#define MQTT_CLIENT_ID   "picocar-t"
#define MQTT_USERNAME    NULL
#define MQTT_PASSWORD    NULL
#define MQTT_KEEPALIVE_S (30u)

#endif /* MQTT_CONFIG_H */

/*** end of file ***/
