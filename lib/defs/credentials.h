/**
 * @file credentials.h
 * @author SenMorgan https://github.com/SenMorgan
 * @date 2022-06-18
 *
 * @copyright Copyright (c) 2022 Sen Morgan
 *
 */

#ifndef _CREDENTIALS_H_
#define _CREDENTIALS_H_

// Fill in your actual WiFi settings as Strings
//#define WIFI_SSID   ""
//#define WIFI_PASSWD ""

// Fill in your actual MQTT settings
//#define MQTT_SERVER      ""
// 1883 - default MQTT port
//#define MQTT_SERVER_PORT 1883
//#define HOSTNAME         ""
//#define MQTT_LOGIN       ""
//#define MQTT_PASSWORD    ""

// Replace HOSTNAME with your actual OTA update hostname or leave it the same as HOSTNAME
#define OTA_HOSTNAME HOSTNAME
// You can leave OTA update password empty if you don't want to secure OTA
#define OTA_PASSWORD ""

// Reminder
#ifndef WIFI_SSID
#error "Please define WIFI_SSID in credentials.h"
#endif
#ifndef WIFI_PASSWD
#error "Please define WIFI_PASSWD in credentials.h"
#endif
#ifndef MQTT_SERVER
#error "Please define MQTT_SERVER in credentials.h"
#endif
#ifndef MQTT_SERVER_PORT
#error "Please define MQTT_SERVER_PORT in credentials.h"
#endif
#ifndef HOSTNAME
#error "Please define HOSTNAME in credentials.h"
#endif
#ifndef MQTT_LOGIN
#error "Please define MQTT_LOGIN in credentials.h"
#endif
#ifndef MQTT_PASSWORD
#error "Please define MQTT_PASSWORD in credentials.h"
#endif

#endif // _CREDENTIALS_H_