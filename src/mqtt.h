#pragma once

#include <Arduino.h>
#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <map>
#include <list>
#include <functional>

class PagerMqtt {
public:
    typedef std::function<void(AsyncMqttClient& client, const String& topic, const String& msg)> MessageListener;
    typedef std::function<void(AsyncMqttClient& client, const String& topic, uint8_t qos)> SubscribeListener;
    typedef std::function<void(AsyncMqttClient& client, bool sessionPresent)> ConnectListener;

    typedef struct {
        ConnectListener onConnect;
        MessageListener onMessage;
        SubscribeListener onSubscribe;
    } MqttEventListener;

    PagerMqtt(const String& host, int port, const String& username, const String& password, const String& client_id)
        : _host(host), _port(port), _username(username), _password(password), _client_id(client_id) {
    };

    void onEvent(const String& topic, MqttEventListener listener) {
        _mlsm[topic].push_back(listener);
    };

    void begin() {
        _mqtt_client.setServer(_host.c_str(), _port);
        _mqtt_client.setCredentials(_username.c_str(), _password.c_str());
        _mqtt_client.setClientId(_client_id.c_str());

        // Mqtt connection events
        _mqtt_client.onConnect([this](bool sessionPresent) {
            Serial.println("Mqtt: Connected.");

            for (auto it = _mlsm.begin(); it != _mlsm.end(); ++it) {
                auto pid = _mqtt_client.subscribe(it->first.c_str(), 1);
                _pitm[pid] = it->first;
            }

            for (auto it = _mlsm.begin(); it != _mlsm.end(); ++it) {
                for (auto lsnr = it->second.begin(); lsnr != it->second.end(); ++lsnr) {
                    if (lsnr->onConnect) {
                        lsnr->onConnect(_mqtt_client, sessionPresent);
                    }
                }
            }
        });

        _mqtt_client.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
            Serial.println("Mqtt: Disconnected.");

            if (WiFi.isConnected()) {
                Serial.println("Mqtt: Reconnecting...");
                _reconnect_ticker.once(2, [this]() {
                    _connect();
                });
            }
        });

        // Mqtt subscribe events
        _mqtt_client.onSubscribe([this](uint16_t packetId, uint8_t qos) {
            Serial.print("Mqtt: Subscribed. PacketId: ");
            Serial.println(packetId);

            if (_pitm.count(packetId) == 1) {
                auto topic = _pitm[packetId];
                auto lst = _mlsm.at(topic);
                for (auto it = lst.begin(); it != lst.end(); ++it) {
                    SubscribeListener lsnr = it->onSubscribe;
                    if (lsnr) {
                        lsnr(_mqtt_client, topic, qos);
                    }
                }
            }
        });

        _mqtt_client.onUnsubscribe([this](uint16_t packetId) {
            // Nothing to do
            Serial.print("Mqtt: Unsubscribed. PacketId: ");
            Serial.println(packetId);
        });

        // Mqtt pub/msg events
        _mqtt_client.onPublish([this](uint16_t packetId) {
            // Nothing to do
        });

        _mqtt_client.onMessage([this](char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
            Serial.println("Mqtt: Message received.");

            auto msg = String("");
            for (size_t i = 0; i < len; i ++) {
                msg += payload[i];
            }
            if (_mlsm.count(topic) == 1) {
                auto lst = _mlsm.at(topic);
                for (auto it = lst.begin(); it != lst.end(); ++it) {
                    MessageListener lsnr = it->onMessage;
                    if (lsnr) {
                        lsnr(_mqtt_client, topic, msg);
                    }
                }
            }
        });

        // WiFi events
        _got_ip_handler = WiFi.onStationModeGotIP([this](const WiFiEventStationModeGotIP &) {
            Serial.println("WiFi is connected. Connecting to mqtt server...");
            _connect();
        });

        _disconnected_handler = WiFi.onStationModeDisconnected([this](const WiFiEventStationModeDisconnected &) {
            Serial.println("WiFi is disconnected. Turn off mqtt...");
            _reconnect_ticker.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        });
    };

    void loop() {
    };

    AsyncMqttClient &getMqttClient() {
        return _mqtt_client;
    };
private:
    void _connect() {
        _mqtt_client.connect();
    };

private:
    String _host;
    int _port;
    String _username;
    String _password;
    String _client_id;

    typedef std::list<MqttEventListener> MqttEventListenerList;
    typedef std::map<String, MqttEventListenerList> MqttEventListenersMap;
    typedef std::map<uint16_t, String> PacketIdTopicMap;

    MqttEventListenersMap _mlsm;
    PacketIdTopicMap _pitm;

    AsyncMqttClient _mqtt_client;

    WiFiEventHandler _got_ip_handler;
    WiFiEventHandler _disconnected_handler;

    Ticker _reconnect_ticker;
};
