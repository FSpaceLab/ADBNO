/*
 * MQTT Controller
 *
 * Created by @Korzhak (GitHub), 21.12.2021
 */

#include "MQTT.h"


void MQTTController::send(const char *topic, char *data)
{
    // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
    if (client.publish(topic, data))
    {
        // Serial.print(topic);
        // Serial.println(" send!");
    }
    // Again, client.publish will return a boolean value depending on whether it succeded or not.
    // If the message failed to send, we will try again, as the connection may have broken.
    else
    {
        Serial.print(topic);
        Serial.println(" failed to send. Reconnecting to MQTT Broker and trying again");
        client.publish(topic, data);
    }
}

void MQTTController::subscribe(const char *feedback_topic)
{
    client.subscribe(feedback_topic);
}

void MQTTController::reconect_to_wifi() {
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    // Serial.println("Connected to the WiFi network");
}

void MQTTController::initialize(
    const char *ssid,
    const char *password,
    const char *mqttServer,
    const int mqttPort,
    const char *mqttUser,
    const char *mqttPassword,
    const char *clientID) // getting callback function as "callback" (define placing in PubSubClient.h)
{
    /*
     * @brief initializing MQTT connection. 
     *
     * 
     */
    client.setClient(espClient);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the WiFi network");

    client.setServer(mqttServer, mqttPort);
    // client.setCallback(callback); // setting callback function getting via signature

    while (!client.connected())
    {
        Serial.println("Connecting to MQTT...");

        if (client.connect(clientID, mqttUser, mqttPassword))
        {

            Serial.println("connected");
        }
        else
        {

            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}


