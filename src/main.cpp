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

enum
{
    WIFI_DISABLED,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    CONNECTED_TO_BROKER,
} stage;

// EEPROM variables
float calculated_wh;
// Global variables
float measured_V, measured_A, measured_P;
uint8_t wifi_sleep_enabled = 1, light_enabled = 1, mqtt_conn;
uint32_t timestamp_on_wifi_begin, timestamp_last_published, timestamp_last_mqtt_reconn;
uint32_t timestamp_on_mqtt_begin, timestamp_conn_failed, timestamp_pub_started, timestamp_sleep_mode_started;

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
            // Need to sleep immediately
            timestamp_pub_started = 0;
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
    if (mqttClient.connect(HOSTNAME, MQTT_LOGIN, MQTT_PASSWORD,
                           MQTT_WILL_TOPIC, MQTT_QOS, MQTT_RETAIN, MQTT_WILL_MESSAGE))
    {
        mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC);
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

/**
 * @brief Light sleep mode
 *
 * @note Call this function only if you are sure that the ESP8266 will be woken up by the interrupt
 * @note This function must be called whem WiFi disabled
 */
void go_to_light_sleep()
{
    wifi_set_opmode(NULL_MODE);
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    // only LOLEVEL or HILEVEL interrupts work, no edge, that's an SDK or CPU limitation
    gpio_pin_wakeup_enable(GPIO_ID_PIN(INA226_ALERT_PIN), GPIO_PIN_INTR_LOLEVEL);
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF); // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(10);                    // it goes to sleep during this delay() and waits for an interrupt
    wifi_fpm_close();
}

void state_machine(uint8_t sleep_mode)
{
    switch (stage)
    {
        case WIFI_DISABLED:
            go_to_light_sleep();
            uint8_t begin_wifi;
            begin_wifi = 0;

            if (sleep_mode)
            {
                if (!timestamp_sleep_mode_started || millis() - timestamp_sleep_mode_started > PUBLISH_INTERVAL_SLOW_MS)
                {
                    begin_wifi = 1;
                }
            }
            else
            {
                if (!timestamp_conn_failed || millis() - timestamp_conn_failed > CONN_FAILED_TIMEOUT_MS)
                {
                    begin_wifi = 1;
                }
            }

            if (begin_wifi)
            {
                // Setup WiFi connection
                WiFi.mode(WIFI_STA);
                WiFi.begin(WIFI_SSID, WIFI_PASSWD);
                WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 3); // Automatic Light Sleep, DTIM listen interval = 3
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            break;

        case WIFI_CONNECTING:
            // Blink with LED while connecting
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(50); // Delay for yield

            // If lost WIFi connection
            if (WiFi.status() == WL_CONNECTED)
            {
                stage = WIFI_CONNECTED;
            }
            else if (millis() - timestamp_on_wifi_begin > MAX_WIFI_RECONN_TIME_MS)
            {
                WiFi.mode(WIFI_OFF);
                digitalWrite(STATUS_LED, 0);
                timestamp_conn_failed = millis();
                timestamp_on_mqtt_begin = millis();
                stage = WIFI_DISABLED;
            }
            break;

        case WIFI_CONNECTED:
            // Blink with LED while connecting
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(50); // Delay for yield

            ArduinoOTA.handle();

            // If lost WIFi connection
            if (WiFi.status() != WL_CONNECTED)
            {
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            else
            {
                // If connected to MQTT broker
                if (mqttClient.loop())
                {
                    timestamp_on_mqtt_begin = 0;
                    timestamp_pub_started = millis();
                    analogWrite(STATUS_LED, 10);
                    stage = CONNECTED_TO_BROKER;
                }
                else if (!timestamp_last_mqtt_reconn || millis() - timestamp_last_mqtt_reconn > MQTT_RECONN_PERIOD_MS)
                {
                    reconnect();
                    timestamp_last_mqtt_reconn = millis();
                }
                else if (millis() - timestamp_on_mqtt_begin > SLEEP_AFTER_DISCONN_FROM_BROKER_MS)
                {
                    // Go back to the power save mode
                    WiFi.mode(WIFI_OFF);
                    digitalWrite(STATUS_LED, 0);
                    timestamp_conn_failed = millis();
                    stage = WIFI_DISABLED;
                }
            }
            break;

        case CONNECTED_TO_BROKER:
            ArduinoOTA.handle();
            mqtt_conn = mqttClient.loop();

            // If lost WIFi connection
            if (WiFi.status() != WL_CONNECTED)
            {
                timestamp_on_wifi_begin = millis();
                stage = WIFI_CONNECTING;
            }
            else
            {
                if (!mqtt_conn)
                {
                    timestamp_on_mqtt_begin = millis();
                    stage = WIFI_CONNECTED;
                }
                else if (millis() - timestamp_last_published > PUBLISH_INTERVAL_FAST_MS)
                {
                    publish_data();
                    timestamp_last_published = millis();
                }
            }

            if (sleep_mode)
            {
                if (!timestamp_pub_started || millis() - timestamp_pub_started > SLEEP_AFTER_MS)
                {
                    mqttClient.publish(MQTT_STATE_TOPIC_SLEEP, MQTT_CMD_ON, true);
                    mqttClient.publish(MQTT_STATE_TOPIC_LIGHT, MQTT_CMD_OFF, true);
                    mqttClient.publish(MQTT_CMD_TOPIC_LIGHT, MQTT_CMD_OFF, true);
                    delay(DELAY_AFTER_PUBLISH_MS);
                    // Wait the data to be published
                    espClient.flush();
                    // Go back to the power save mode
                    WiFi.mode(WIFI_OFF);
                    digitalWrite(STATUS_LED, 0);
                    timestamp_conn_failed = 0;
                    timestamp_sleep_mode_started = millis();
                    stage = WIFI_DISABLED;
                }
            }
            break;
    }
}

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

    ina226.begin(INA226_ADDRESS);
    // When set INA226_AVERAGES_64 and INA226_SHUNT_CONV_TIME_8244US, the period is about 1 second
    ina226.configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_8244US,
                     INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_CONT);
    ina226.calibrate(SHUNT_VALUE, MAX_CURRENT_EXCEPTED_A);
    ina226.enableConversionReadyAlert();

    led_fade_off(STATUS_LED, 255, 2);
}

void loop()
{
    read_ina226_values();

    state_machine(wifi_sleep_enabled);

    yield();
}