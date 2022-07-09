#include <Arduino.h>
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
float inverter_wh;

// Global variables
float inverter_V, inverter_A, inverter_P;

uint8_t sleep_enabled = 1;
uint32_t reconn_time;

void setup()
{
    EEPROM.begin(4);
    // EEPROM.put(0, inverter_wh); // erase EEPROM
    EEPROM.get(0, inverter_wh);

    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(INA226_ALERT_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, 1);

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
    static uint8_t ina226_read_counter = 0;

    if (read_ina226_values())
    {
        ina226_read_counter++;
    }

    if (ina226_read_counter >= SEND_DATA_AFTER_N_READS)
    {
        // If finished publishing data, reset counter
        if (reconnect_and_publish())
            ina226_read_counter = 0;
    }

    if (sleep_enabled)
        go_to_light_sleep();

    yield();
}

uint8_t reconnect_and_publish()
{
    static uint8_t stage;
    static uint32_t timestamp_on_begin;

    switch (stage)
    {
        case 0:
            timestamp_on_begin = millis();
            // Setup WiFi connection
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASSWD);
            WiFi.hostname(HOSTNAME);
            WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 3); // Automatic Light Sleep, DTIM listen interval = 3
            stage++;
            sleep_enabled = 0;
            led_fade_on(STATUS_LED, 40, 3);
            break;
        case 1:
            // Wait for WiFi connection
            if (WiFi.status() == WL_CONNECTED)
            {
                stage++;
            }
            else
            {
                // Delay for Light Sleep to be enabled
                delay(350);
            }
            break;
        case 2:
            if (mqttClient.loop())
            {
                stage++;
            }
            // If we are not connected to the MQTT broker, try to reconnect every RECONNECT_DELAY_MS
            else
            {
                reconnect();
                // Delay for Light Sleep to be enabled
                delay(350);
            }
            break;
        case 3:
            reconn_time = millis() - timestamp_on_begin;
            // Publish data
            publish_data();
            stage++;
            break;
        case 4:
            // Wait the data to be published
            espClient.flush();
            // Go back to the powersave mode
            WiFi.mode(WIFI_OFF);
            led_fade_off(STATUS_LED, 40, 3);
            stage = 0;
            sleep_enabled = 1;
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

        return 1;
    }

    return 0;
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

    if (!reconnectTimer || millis() - reconnectTimer >= RECONNECT_DELAY_MS)
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
    static char buff[20];

    // analogWrite(STATUS_LED, 10);

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
    // sprintf(buff, "%ld", millis() / 1000);
    sprintf(buff, "%d", reconn_time);
    mqttClient.publish(DEFAULT_TOPIC "timestamp", buff);

    // analogWrite(STATUS_LED, 0);
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