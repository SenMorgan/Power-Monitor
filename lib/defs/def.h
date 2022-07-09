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

// Reconnecting to MQTT server delay
#define RECONNECT_DELAY_MS      1000
/* How often will data been published to broker and WiFi will be reconnected.
The period of 1 reading is ina226_averages_t * ina226_busConvTime_t
or ina226_averages_t * ina226_busConvTime_t (the lowest one)
When set INA226_AVERAGES_64 and INA226_SHUNT_CONV_TIME_8244US, the period is about 1 second
*/
#define SEND_DATA_AFTER_N_READS 30

// IO pins
#define SDA_PIN          0
#define SCL_PIN          2
#define INA226_ALERT_PIN 3
/** WARN: ESP-01 built-in LED can't be used if I2C is enabled on GPIO 2
 * so we are using GPIO 1 (TX pin) and external LED
 */
#define STATUS_LED       1

#endif // _DEF_H_