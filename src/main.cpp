#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <INA226.h>
#include <EEPROM.h>

#include "credentials.h"
#include "def.h"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

INA226 ina226(Wire);

// Forward declarations
uint8_t read_ina226_values(void);
void callback(String topic, byte *payload, unsigned int length);
void reconnect(void);
void publish_data(void);
void led_fade_on(uint8_t led_pin, int brightness, uint32_t period);
void led_fade_off(uint8_t led_pin, int brightness, uint32_t period);
uint8_t reconnect_and_publish();
void go_to_light_sleep();

// EEPROM variables
float calculated_wh;
// Global variables
float measured_V, measured_A, measured_P;
uint8_t light_sleep_enabled = 1, wifi_sleep_enabled = 1, light_enabled;
uint32_t reconn_time, timestamp_last_published;

void setup()
{
    EEPROM.begin(4);
    // EEPROM.put(0, calculated_wh); // erase EEPROM
    EEPROM.get(0, calculated_wh);

    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, 1);

    WiFi.mode(WIFI_OFF);
    WiFi.hostname(HOSTNAME);

    // Arduino OTA initializing
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    ArduinoOTA.onStart([]()
                       {
                        wifi_sleep_enabled = 0;
                        digitalWrite(STATUS_LED, 1); });

    // MQTT initializing
    mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
    mqttClient.setCallback(callback);
    // reconnect();

    ina226.begin(INA226_ADDRESS);
    ina226.configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_8244US,
                     INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
    ina226.calibrate(0.00075, 40);
    ina226.enableConversionReadyAlert();

    led_fade_off(STATUS_LED, 255, 2);
}

void loop()
{
    read_ina226_values();

    if (millis() - timestamp_last_published > PUBLISH_INTERVAL_SLOW_MS ||
        !timestamp_last_published ||
        !wifi_sleep_enabled)
    {
        if (reconnect_and_publish())
        {
            timestamp_last_published = millis();
        }
    }

    if (light_sleep_enabled && wifi_sleep_enabled)
        go_to_light_sleep();

    yield();
}

uint8_t reconnect_and_publish()
{
    static uint8_t stage;
    static uint32_t timestamp_on_begin, timestamp_after_connection, timestamp_last_published, disconnected_time_stamp;

    switch (stage)
    {
        case 0:
            timestamp_on_begin = millis();
            analogWrite(STATUS_LED, 10);
            // Setup WiFi connection
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWD);
            WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 3); // Automatic Light Sleep, DTIM listen interval = 3
            stage++;
            light_sleep_enabled = 0;
            break;
        case 1:
            // Wait for WiFi connection
            if (WiFi.status() == WL_CONNECTED)
            {
                stage++;
            }
            else
            {
                if (millis() - timestamp_on_begin > MAX_WIFI_RECONNECT_TIME_MS)
                {
                    stage = 4;
                }
                // Delay for Light Sleep to be enabled
                delay(350);
            }
            break;
        case 2:
            if (mqttClient.loop())
            {
                reconn_time = millis() - timestamp_on_begin;
                timestamp_after_connection = millis();
                stage++;
            }
            // If we are not connected to the MQTT broker, try to reconnect every RECONNECT_DELAY_MS
            else
            {
                if (millis() - timestamp_on_begin > MAX_MQTT_RECONNECT_TIME_MS)
                {
                    stage = 4;
                }
                else
                {
                    reconnect();
                }
                // Delay for Light Sleep to be enabled
                delay(350);
            }
            break;
        case 3:
            if (wifi_sleep_enabled && millis() - timestamp_after_connection > SLEEP_AFTER_MS)
            {
                stage++;
            }
            // Main second loop when sleep mode disabled
            else
            {
                ArduinoOTA.handle();
                uint8_t connected_to_broker = mqttClient.loop();

                if (millis() - timestamp_last_published > PUBLISH_INTERVAL_FAST_MS)
                {
                    timestamp_last_published = millis();
                    // Publish data
                    publish_data();
                }
                // Save timestamp when loose connection to the broker
                if (!connected_to_broker && !disconnected_time_stamp)
                    disconnected_time_stamp = millis();
                else if (connected_to_broker && disconnected_time_stamp)
                    disconnected_time_stamp = 0;

                // Fall asleep after some time if can't connect to the broker
                if (disconnected_time_stamp && millis() - disconnected_time_stamp > SLEEP_AFTER_DISCONNECT_MS)
                {
                    stage++;
                }
            }
            break;
        case 4:
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_ON, true);
            mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_OFF, true);
            mqttClient.publish(MQTT_CMD_TOPIC_LIGHT, MQTT_CMD_OFF, true);
            delay(DELAY_AFTER_PUBLISH_MS);
            // Wait the data to be published
            espClient.flush();
            // Go back to the powersave mode
            WiFi.mode(WIFI_OFF);
            stage = 0;
            light_sleep_enabled = 1;
            analogWrite(STATUS_LED, 0);
            return 1;
            break;
    }

    return 0;
}

void go_to_light_sleep()
{
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    // only LOLEVEL or HILEVEL interrupts work, no edge, that's an SDK or CPU limitation
    gpio_pin_wakeup_enable(GPIO_ID_PIN(INA226_ALERT_PIN), GPIO_PIN_INTR_LOLEVEL);
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF); // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);                    // it goes to sleep during this delay() and waits for an interrupt
}

uint8_t read_ina226_values(void)
{
    static uint32_t last_meas_timestamp;
    static int eeprom_cnt;

    if (!digitalRead(INA226_ALERT_PIN))
    {
        // Erase alert
        ina226.isAlert();
        last_meas_timestamp = millis();

        measured_V = ina226.readBusVoltage();
        measured_A = ina226.readShuntCurrent();
        measured_P = ina226.readBusPower();

        uint32_t time_now = millis();

        if (last_meas_timestamp > 0)
        {
            calculated_wh += float(((time_now - last_meas_timestamp) * measured_P) / 3600000.0f);
        }
        last_meas_timestamp = time_now;

        // Save values to EEPROM every 10 measurements
        eeprom_cnt++;
        if (eeprom_cnt > 10)
        {
            eeprom_cnt = 0;
            EEPROM.put(0, calculated_wh);
            EEPROM.commit();
        }

        return 1;
    }

    return 0;
}

/**
 * @brief Callback function for MQTT client
 */
void callback(String topic, byte *payload, unsigned int length)
{
    String msgString = "";
    for (uint16_t i = 0; i < length; i++)
        msgString += (char)payload[i];

    // Reset board when received "reset" message
    if (topic == (MQTT_CMD_TOPIC_RESET))
    {
        if (msgString == MQTT_CMD_ON)
            ESP.restart();
    }
    else if (topic == (MQTT_CMD_TOPIC_WH_RESET))
    {
        if (msgString == MQTT_CMD_ON)
        {
            mqttClient.publish(MQTT_CMD_TOPIC_WH_RESET, MQTT_CMD_OFF, true);
            calculated_wh = 0;
            EEPROM.put(0, calculated_wh);
            EEPROM.commit();
        }
    }
    // Enable/disable WiFi sleep mode
    else if (topic == (MQTT_CMD_TOPIC_SLEEP))
    {
        if (msgString == MQTT_CMD_ON && !wifi_sleep_enabled)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_ON, true);
            wifi_sleep_enabled = 1;
        }
        else if (msgString == MQTT_CMD_OFF && wifi_sleep_enabled)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_OFF, true);
            wifi_sleep_enabled = 0;
        }
    }
    else if (topic == (MQTT_CMD_TOPIC_LIGHT))
    {
        if (msgString == MQTT_CMD_ON && !light_enabled)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_ON, true);
            analogWrite(STATUS_LED, 255);
            light_enabled = 1;
        }
        else if (msgString == MQTT_CMD_OFF && light_enabled)
        {
            mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_OFF, true);
            analogWrite(STATUS_LED, 0);
            light_enabled = 0;
        }
    }
}

/**
 * @brief Establishing connection or reconnecting to MQTT server
 */
void reconnect(void)
{
    static uint32_t reconnectTimer;

    if (!reconnectTimer || millis() - reconnectTimer >= RECONNECT_DELAY_MS)
    {
        if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                               MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
        {
            mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
        }
        reconnectTimer = millis();
    }
}

/**
 * @brief Publish data to broker
 */
void publish_data(void)
{
    static char buff[20];

    mqttClient.publish(MQTT_AVAILABILITY_TOPIC, MQTT_AVAILABILITY_MESSAGE);
    sprintf(buff, "%0.3f", measured_V);
    mqttClient.publish(MQTT_STATE_TOPIC_VOLT, buff);
    sprintf(buff, "%0.3f", measured_A);
    mqttClient.publish(MQTT_STATE_TOPIC_AMP, buff);
    sprintf(buff, "%0.3f", measured_P);
    mqttClient.publish(MQTT_STATE_TOPIC_WATT, buff);
    sprintf(buff, "%f", calculated_wh);
    mqttClient.publish(MQTT_STATE_TOPIC_WH, buff);
    sprintf(buff, "%ld", millis() / 1000);
    mqttClient.publish(MQTT_STATE_TOPIC_UPTIME, buff);
}

void led_fade_on(uint8_t led_pin, int brightness, uint32_t period)
{
    for (int i = 0; i <= brightness; i++)
    {
        analogWrite(led_pin, i);
        delay(period);
    }
}

void led_fade_off(uint8_t led_pin, int brightness, uint32_t period)
{
    for (; brightness >= 0; brightness--)
    {
        analogWrite(led_pin, brightness);
        delay(period);
    }
}