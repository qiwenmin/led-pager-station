#include <Arduino.h>
#include <ESP8266WiFiMulti.h>
#include <Ticker.h>

#include "mqtt.h"

#define SSID_MAX_LEN 32
#define PASS_MAX_LEN 64

typedef struct {
    char ssid[SSID_MAX_LEN + 1];
    char pass[PASS_MAX_LEN + 1];
} WiFi_Credentials;

#include "config.inc"

// WiFi
static const uint32_t wifi_connect_timeout_ms = 5000;
ESP8266WiFiMulti wifi_multi;

// Mqtt
PagerMqtt pager_mqtt(mqtt_host, mqtt_port, mqtt_username, mqtt_password, mqtt_client_id);
static PagerMqtt::MqttEventListener mqtt_event_listener;
static unsigned char flag = 0;

// indicators
class Indicator {
public:
    Indicator(uint8_t pin, uint8_t interval, uint8_t off_value = HIGH)
        : _pin(pin), _interval(interval), _off_value(off_value), _is_on(false), _cnt(0) {
    };

    void setup() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, _off_value);
    };

    void loop() {
        _cnt++;
        if (_cnt == _interval) {
            _cnt = 0;
                if (_is_on) {
                    digitalWrite(_pin, !_off_value);
                }

        } else {
            digitalWrite(_pin, _off_value);
        }
    };

    void setOn(bool is_on = true) {
        _is_on = is_on;
    };

    bool isOn() {
        return _is_on;
    };
private:
    uint8_t _pin;
    uint8_t _interval;
    uint8_t _off_value;
    bool _is_on;
    uint8_t _cnt;
};

#define MQTT_IND_PIN (LED_BUILTIN)
#define N_FLAG_PIN (13)
#define E_FLAG_PIN (12)

Indicator mqtt_indicator(MQTT_IND_PIN, 200);
Indicator nflag_indicator(N_FLAG_PIN, 150);
Indicator eflag_indicator(E_FLAG_PIN, 50);

void setup() {
    mqtt_indicator.setup();
    eflag_indicator.setup();
    nflag_indicator.setup();

    WiFi.persistent(false);

    Serial.begin(9600);
    Serial.println("\n\n<<<<< LED Pager Sender >>>>>");

    WiFi.mode(WIFI_STA);

    for (auto cred: wifi_creds) {
        wifi_multi.addAP(cred.ssid, cred.pass);
    }

    mqtt_event_listener.onMessage = [](AsyncMqttClient& client, const String &topic, const String &msg) {
        Serial.print("Received mqtt message in topic [");
        Serial.print(topic);
        Serial.print("] with content [");
        Serial.print(msg);
        Serial.println("].");

        flag = msg[0] - '0';
    };

    pager_mqtt.onEvent(mqtt_topic, mqtt_event_listener);

    pager_mqtt.begin();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Connecting to WiFi...");

        if (wifi_multi.run(wifi_connect_timeout_ms) == WL_CONNECTED) {
            Serial.print("Connected to ");
            Serial.print(WiFi.SSID());
            Serial.print(" with IP address ");
            Serial.print(WiFi.localIP());
            Serial.print(" [");
            Serial.print(WiFi.RSSI());
            Serial.println("dBm].");
        } else {
            Serial.println("Failed!");
        }
    }

    mqtt_indicator.setOn(flag == 0 && pager_mqtt.getMqttClient().connected());
    nflag_indicator.setOn(flag == 2);
    eflag_indicator.setOn(flag & 0x01);

    mqtt_indicator.loop();
    nflag_indicator.loop();
    eflag_indicator.loop();

    delay(100); // indicator tick interval and power saving
}
