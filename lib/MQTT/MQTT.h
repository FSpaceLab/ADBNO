/*
 * MQTT Controller
 *
 * Created by @Korzhak (GitHub), 21.12.2021
 */

#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MQTTController
{
private:
    WiFiClient espClient;
    static void callback(char *topic, byte *payload, unsigned int length);

public:
    PubSubClient client;

    void initialize(
        const char *ssid,
        const char *password,
        const char *mqttServer,
        const int mqttPort,
        const char *mqttUser,
        const char *mqttPassword,
        const char *clientID); // passing callback function

    void subscribe(const char *feedback_topic);
    void send(const char *topic, char * data);
    void reconect_to_wifi();
};

#endif