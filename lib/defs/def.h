/**
 * @file def.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2022-06-18
 *
 * @copyright Copyright (c) 2021 Sen Morgan
 *
 */

#ifndef _DEF_H_
#define _DEF_H_

// MQTT definitions
#define DEFAULT_TOPIC             "/inverter/"
#define MQTT_WILL_TOPIC           DEFAULT_TOPIC "availability"
#define MQTT_QOS                  1
#define MQTT_RETAIN               0
#define MQTT_WILL_MESSAGE         DEFAULT_TOPIC "offline"
#define MQTT_SUBSCRIBE_TOPIC      DEFAULT_TOPIC "#"
#define MQTT_PUBLISH_TOPIC        DEFAULT_TOPIC "state"
#define MQTT_AVAILABILITY_TOPIC   DEFAULT_TOPIC "availability"
#define MQTT_AVAILABILITY_MESSAGE "online"
#define MQTT_UPTIME_TOPIC         DEFAULT_TOPIC "uptime"

// Publishing data period
#define PUBLISH_DELAY_MS   1000
// Reconnecting to MQTT server delay
#define RECONNECT_DELAY_MS 5000

// IO pins
#define SDA_PIN          0
#define SCL_PIN          2
#define INA226_ALERT_PIN 3
/** WARN: ESP-01 built-in LED can't be used if I2C is enabled on GPIO 2
 * so we are using GPIO 1 (TX pin) and external LED
 */
#define STATUS_LED 1

#endif // _DEF_H_