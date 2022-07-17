#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG  // TODO comment it

#ifdef DEBUG
#define LOG(str) Serial.println(str);
#else
#define LOG(str)
#endif

// #define SIDE "LEFT"
#define SIDE "RIGHT"

// #define MQTT_WIFI_SSID "FSpace"
#define MQTT_WIFI_SSID "HomeWiFi"
// #define MQTT_WIFI_PASSWORD "freakspace#main"
#define MQTT_WIFI_PASSWORD "57907433"
// #define MQTT_SERVER "teleoperate.annaone.com"
// #define MQTT_PORT 1883
// #define MQTT_USER "magic"
// #define MQTT_PASSWORD "zEbl042_Chug"

#define MQTT_SERVER "192.168.0.107"
#define MQTT_PORT 1883
#define MQTT_USER "anna_1"
#define MQTT_PASSWORD "anna_1"
#define MQTT_CLIENT_ID "smart_glove"
#define MQTT_ARMS_TOPIC "anna_2/arms/right"
// #define MQTT_ARMS_TOPIC "anna_1/arms/left"

#define DELAY_MS 70

 #define CALIBRATING_IMU_ENABLE

//#define SHOULDER_SENSOR_ADDR 0X29   // chock
//#define ROOT_BACK_SENSOR_ADDR 0x28 // chock
#define SHOULDER_SENSOR_ADDR 0X28 // LOW
#define ROOT_BACK_SENSOR_ADDR 0x29 // HIGH
#define ELBOW_SENSOR_ADDR 0x48
#define WRIST_X_SENSOR_ADDR 0x50
#define WRIST_Y_SENSOR_ADDR 0x51
#define THUMB_FINGER_SENSOR_ADDR 0x52
#define INDEX_FINGER_SENSOR_ADDR 0x53
#define MIDDLE_FINGER_SENSOR_ADDR 0x54
#define RING_FINGER_SENSOR_ADDR 0x55
#define LITTLE_FINGER_SENSOR_ADDR 0x56

#define DIRECTION_X 1
#define DIRECTION_Y 1
#define DIRECTION_Z 1

#endif