#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <INA226.h>
#include <EEPROM.h>

#include "credentials.h"
#include "def.h"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

INA226 ina226(Wire);

void read_ina226_values(void);
void callback(String topic, byte *payload, unsigned int length);
void reconnect(void);
void publish_data(void);

// EEPROM variables
float inverter_wh;

// Global variables
float inverter_V, inverter_A, inverter_P;

void setup()
{
    EEPROM.begin(4);
    // EEPROM.put(0, energy);   // erase EEPROM
    EEPROM.get(0, inverter_wh);

    Wire.begin(SDA_PIN, SCL_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWD);
    WiFi.hostname(HOSTNAME);

    // Arduino OTA initializing
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    // Blink with built-in LED to indicate OTA update is in progress
    ArduinoOTA.onStart([]()
                       { pinMode(STATUS_LED, OUTPUT); });
    ArduinoOTA.onProgress([](uint16_t progress, uint16_t total)
                          { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); });
    ArduinoOTA.onEnd([]()
                     { Wire.begin(SDA_PIN, SCL_PIN); });

    // MQTT initializing
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);
    reconnect();

    ina226.begin(INA226_ADDRESS);
    ina226.configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_8244US,
                     INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
    ina226.calibrate(0.00075, 40);
}

void loop()
{
    ArduinoOTA.handle();
    read_ina226_values();

    // If we are connected to the MQTT server, publish the data
    if (mqttClient.loop())
    {
        publish_data();
    }
    // If we are not connected to the MQTT server, try to reconnect every RECONNECT_DELAY_MS
    else
    {
        reconnect();
    }

    yield();
}

void read_ina226_values(void)
{
    static uint32_t last_meas_timestamp;
    static int eeprom_cnt;
    if (millis() - last_meas_timestamp > READ_STEP_MS)
    {
        last_meas_timestamp = millis();

        inverter_V = ina226.readBusVoltage();
        inverter_A = ina226.readShuntCurrent();
        inverter_P = ina226.readBusPower();

        uint32_t time_now = millis();

        if (last_meas_timestamp > 0)
        {
            inverter_wh += float(((time_now - last_meas_timestamp) * inverter_P) / 3600000.0f);
        }
        last_meas_timestamp = time_now;

        // Save values to EEPROM every 10 measurements
        eeprom_cnt++;
        if (eeprom_cnt > 10)
        {
            eeprom_cnt = 0;
            EEPROM.put(0, inverter_wh);
            EEPROM.commit();
        }
    }
}

/**
 * @brief Callback function for MQTT client
 */
void callback(String topic, byte *payload, unsigned int length)
{
    // Reset board when received "reset" message
    if (topic == (DEFAULT_TOPIC "reset"))
    {
        if ((char)payload[0] == '1')
            ESP.restart();
    }
}

/**
 * @brief Establishing connection or reconnecting to MQTT server
 */
void reconnect(void)
{
    static uint32_t reconnectTimer;

    if (!reconnectTimer || millis() - reconnectTimer > RECONNECT_DELAY_MS)
    {
        if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                               MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
        {
            mqttClient.subscribe(DEFAULT_TOPIC "#");
        }
        reconnectTimer = millis();
    }
}

/**
 * @brief Publish data to broker
 */
void publish_data(void)
{
    static uint32_t last_pub_timestamp;
    static char buff[20];

    if (millis() - last_pub_timestamp > PUBLISH_DELAY_MS)
    {
        last_pub_timestamp = millis();

        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_MESSAGE);
        sprintf(buff, "%ld", millis() / 1000);
        mqttClient.publish(MQTT_UPTIME_TOPIC, buff);
        sprintf(buff, "%0.3f", inverter_V);
        mqttClient.publish(DEFAULT_TOPIC "volt", buff);
        sprintf(buff, "%0.3f", inverter_A);
        mqttClient.publish(DEFAULT_TOPIC "amp", buff);
        sprintf(buff, "%0.3f", inverter_P);
        mqttClient.publish(DEFAULT_TOPIC "watt", buff);
        sprintf(buff, "%f", inverter_wh);
        mqttClient.publish(DEFAULT_TOPIC "wh", buff);
    }
}