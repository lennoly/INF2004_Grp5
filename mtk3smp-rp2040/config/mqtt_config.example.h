/** @file mqtt_config.example.h
 *
 * @brief PicoCar MQTT settings.  Copy to config/mqtt_config.h (ignored by
 *        git), edit it, and set MQTT_CONFIG_SET to 1.
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_CONFIG_SET      (0)

#if !MQTT_CONFIG_SET
#error "config/mqtt_config.h: set the broker address, then MQTT_CONFIG_SET 1"
#endif

#define MQTT_BROKER_IP       "192.168.1.100"  /* laptop running mosquitto */
#define MQTT_BROKER_PORT     (1883u)
#define MQTT_CAR_ID          "car1"           /* topic prefix picocar/car1 */
#define MQTT_CLIENT_ID       "picocar-car1"
#define MQTT_USERNAME        NULL             /* or "user"                 */
#define MQTT_PASSWORD        NULL
#define MQTT_KEEPALIVE_S     (30u)

#endif /* MQTT_CONFIG_H */

/*** end of file ***/
