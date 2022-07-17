#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MQTT.h"
#include "config.h"
#include "ArduinoJson.h"
#include "helper.h"

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 root_imu = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 shoulder_imu = Adafruit_BNO055(56, 0x29);

MQTTController mqtt;

bool is_root_sensor_available = false;
bool is_shoulder_sensor_available = false;

// IMU SENSORS STATUS, ERRORS and CALIBRATING DATA
uint8_t root_system_status, root_self_test_results, root_system_error;
root_system_status = root_self_test_results = root_system_error = 0;
uint8_t root_system, root_gyro, root_accel, root_mag;
root_system = root_gyro = root_accel = root_mag = 0;

uint8_t system_system_status, system_self_test_results, system_system_error;
system_system_status = system_self_test_results = system_system_error = 0;
uint8_t system_system, system_gyro, system_accel, system_mag;
system_system = system_gyro = system_accel = system_mag = 0;

uint8_t root_system, root_gyro, root_accel, root_mag;
root_system = root_gyro = root_accel = root_mag = 0; 
uint8_t shoulder_system, shoulder_gyro, shoulder_accel, shoulder_mag;
shoulder_system = shoulder_gyro = shoulder_accel = shoulder_mag = 0;

// JSON OBJECT FOR TRANSFERING DATA
StaticJsonDocument<256> json_obj;
char json_arr[256];

// OBJECTS FOR PROCESSING ANGLES
imu::Quaternion root_q, shoulder_q, finaly_q;

void setup(void)
{
  Serial.begin(115200);
  LOG("Orientation Sensor Test");

  /* Initialise the sensor */
  if (!root_imu.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    LOG("Ooops, no BNO055 ROOT detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the sensor */
  if (!shoulder_imu.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    LOG("Ooops, no BNO055 SHOULDER detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  is_root_sensor_available = true;
  is_shoulder_sensor_available = true;
  
   mqtt.initialize(
    MQTT_WIFI_SSID,
    MQTT_WIFI_PASSWORD,
    MQTT_SERVER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASSWORD,
    MQTT_CLIENT_ID);
    

  delay(1000);
}

void loop(void)
{ 
  root_q = root_imu.getQuat();
  shoulder_q = shoulder_imu.getQuat();
  // printQ(root_q, shoulder_q);

  finaly_q = root_q * shoulder_q.conjugate();

  Angles final_angles;
  final_angles = get_angles_from_quat(finaly_q, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  // iss - IMU SYSTEM STATUS;  ist - IMU SELFT TEST; ise - IMU SYSTEM ERROR
  // ics - IMU CALIBRATING SYSTEM; icg - IMU CALIBRATING GYRO
  // ica - IMU CALIBRATING ACCELEROMETER; icm - IMU CALIBRATING MAGNETOMETER
  if (is_root_sensor_available) {
    root_imu.getSystemStatus(&root_system_status, &root_self_test_results, &root_system_error);
    root_imu.getCalibration(&root_system, &root_gyro, &root_accel, &root_mag);
    json_obj["iss1"] = String(root_system_status);
    json_obj["ist1"] = String(root_self_test_result);
    json_obj["ise1"] = String(root_system_error);
    json_obj["ics1"] = String(root_system);
    json_obj["icg1"] = String(root_gyro);
    json_obj["ica1"] = String(root_accel);
    json_obj["icm1"] = String(root_mag);
  } else {
    json_obj["iss1"] = "-1";
    json_obj["ist1"] = "-1";
    json_obj["ise1"] = "-1";
    json_obj["ics1"] = "-1";
    json_obj["icg1"] = "-1";
    json_obj["ica1"] = "-1";
    json_obj["icm1"] = "-1";
  }
  
  if (is_shoulder_sensor_available) {
    shoulder_imu.getSystemStatus(&shoulder_system_status, &shoulder_self_test_results, &shoulder_system_error);
    shoulder_imu.getCalibration(&shoulder_system, &shoulder_gyro, &shoulder_accel, &shoulder_mag);
    json_obj["iss2"] = String(shoulder_system_status);
    json_obj["ist2"] = String(shoulder_self_test_result);
    json_obj["ise2"] = String(shoulder_system_error);
    json_obj["ics2"] = String(shoulder_system);
    json_obj["icg2"] = String(shoulder_gyro);
    json_obj["ica2"] = String(shoulder_accel);
    json_obj["icm2"] = String(shoulder_mag);
  } else {
    json_obj["iss2"] = "-1";
    json_obj["ist2"] = "-1";
    json_obj["ise2"] = "-1";
    json_obj["ics2"] = "-1";
    json_obj["icg2"] = "-1";
    json_obj["ica2"] = "-1";
    json_obj["icm2"] = "-1";
  }

  if (is_root_sensor_available && is_shoulder_sensor_available) {
    json_obj["sx"] = String(final_angles.from_z);
    json_obj["sy"] = String(final_angles.from_x);
    json_obj["sz"] = String(final_angles.from_y);
  } else {
    json_obj["sx"] = "-1";
    json_obj["sy"] = "-1";
    json_obj["sz"] = "-1";
  }
  
  serializeJson(json_obj, json_arr);
  
  mqtt.send(MQTT_ARMS_TOPIC, json_arr);

  LOG(json_arr);

  delay(DELAY_MS);
}