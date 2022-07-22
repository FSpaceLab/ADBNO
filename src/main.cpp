#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MQTT.h"
#include "config.h"
#include "ArduinoJson.h"
#include "helper.h"
#include "ADS1X15.h"
#include "SparkFunFlexes.h" 

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 root_imu = Adafruit_BNO055(ROOT_SENSOR_ID, ROOT_SENSOR_ADDR);
Adafruit_BNO055 shoulder_imu = Adafruit_BNO055(SHOULDER_SENSOR_ID, SHOULDER_SENSOR_ADDR);
Adafruit_ADS1115 elbow_sensor;
ADS wrist_x_sensor;
ADS wrist_y_sensor;
ADS thumb_finger_sensor;
ADS index_finger_sensor;
ADS middle_finger_sensor;
ADS ring_finger_sensor;
ADS little_finger_sensor;

MQTTController mqtt;

bool is_root_sensor_available = false;
bool is_shoulder_sensor_available = false;

// IMU SENSORS STATUS, ERRORS and CALIBRATING DATA
uint8_t root_system_status = 0, 
        root_self_test_results = 0, 
        root_system_error = 0;

uint8_t root_system = 0, 
        root_preview_system = 0, 
        root_gyro = 0, 
        root_accel = 0, 
        root_mag = 0;

uint8_t shoulder_system_status = 0,
        shoulder_self_test_results = 0,
        shoulder_system_error = 0;
       
uint8_t shoulder_system = 0, 
        shoulder_preview_system = 0, 
        shoulder_gyro = 0, 
        shoulder_accel = 0, 
        shoulder_mag = 0;


// OBJECTS FOR PROCESSING ANGLES
imu::Quaternion root_q, shoulder_q, finaly_q;

void printQ(imu::Quaternion &q, imu::Quaternion &q2);

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

  while (!elbow_sensor.begin(ELBOW_SENSOR_ADDR))
  {
    LOG("Failed to initialize elbow_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("elbow_sensor initialized successfully");
  
  while(!wrist_x_sensor.begin(WRIST_X_SENSOR_ADDR))
  {
    LOG("Failed to initialize wrist_x_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("wrist_x_sensor initialized successfully");

  while(!wrist_y_sensor.begin(WRIST_Y_SENSOR_ADDR))
  {
    LOG("Failed to initialize wrist_y_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("wrist_y_sensor initialized successfully");
  
  while(!thumb_finger_sensor.begin(THUMB_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize thumb_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("thumb_finger_sensor initialized successfully");

  while(!index_finger_sensor.begin(INDEX_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize index_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("index_finger_sensor initialized successfully");

  while(!middle_finger_sensor.begin(MIDDLE_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize middle_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("middle_finger_sensor initialized successfully");

  while(!ring_finger_sensor.begin(RING_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize ring_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("ring_finger_sensor initialized successfully");

  while(!little_finger_sensor.begin(LITTLE_FINGER_SENSOR_ADDR))
  {
    LOG("Failed to initialize little_finger_sensor. Retrying...");
    delay(DELAY_MS);
    break; // TODO remove it
  }
  LOG("little_finger_sensor initialized successfully");


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
  
  // JSON OBJECT FOR TRANSFERING DATA
  StaticJsonDocument<256> json_obj;
  char json_arr[256];

  if (is_root_sensor_available) {
    root_imu.getSystemStatus(&root_system_status, &root_self_test_results, &root_system_error);
    json_obj["iss1"] = String(root_system_status);
    json_obj["ist1"] = String(root_self_test_results);
    json_obj["ise1"] = String(root_system_error);
  } else {
    json_obj["iss1"] = "-1";
    json_obj["ist1"] = "-1";
    json_obj["ise1"] = "-1";
  }

  if (is_shoulder_sensor_available) {
    shoulder_imu.getSystemStatus(&shoulder_system_status, &shoulder_self_test_results, &shoulder_system_error);
    json_obj["iss2"] = String(shoulder_system_status);
    json_obj["ist2"] = String(shoulder_self_test_results);
    json_obj["ise2"] = String(shoulder_system_error);
  } else {
    json_obj["iss2"] = "-1";
    json_obj["ist2"] = "-1";
    json_obj["ise2"] = "-1";
  }
  serializeJson(json_obj, json_arr);
  
  mqtt.send(MQTT_ARMS_TOPIC, json_arr);
  delay(1000);
}

void loop(void)
{ 
  // JSON OBJECT FOR TRANSFERING DATA
  StaticJsonDocument<512> json_obj;
  char json_arr[512];
  root_q = root_imu.getQuat();
  shoulder_q = shoulder_imu.getQuat();
  // printQ(root_q, shoulder_q);

  finaly_q = root_q / shoulder_q;

  Angles final_angles;
  final_angles = get_angles_from_quat(finaly_q, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  // iss - IMU SYSTEM STATUS;  ist - IMU SELFT TEST; ise - IMU SYSTEM ERROR
  // ics - IMU CALIBRATING SYSTEM; icg - IMU CALIBRATING GYRO
  // ica - IMU CALIBRATING ACCELEROMETER; icm - IMU CALIBRATING MAGNETOMETER
  if (is_root_sensor_available) {
    root_imu.getCalibration(&root_system, &root_gyro, &root_accel, &root_mag);
    json_obj["ics1"] = String(root_system);
    json_obj["icg1"] = String(root_gyro);
    json_obj["ica1"] = String(root_accel);
    json_obj["icm1"] = String(root_mag);
  } else {
    json_obj["ics1"] = "-1";
    json_obj["icg1"] = "-1";
    json_obj["ica1"] = "-1";
    json_obj["icm1"] = "-1";
  }
  
  if (is_shoulder_sensor_available) {
    shoulder_imu.getCalibration(&shoulder_system, &shoulder_gyro, &shoulder_accel, &shoulder_mag);
    json_obj["ics2"] = String(shoulder_system);
    json_obj["icg2"] = String(shoulder_gyro);
    json_obj["ica2"] = String(shoulder_accel);
    json_obj["icm2"] = String(shoulder_mag);
  } else {
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

  json_obj["el"] = elbow_sensor.isConnected() ? String(elbow_sensor.readADC_SingleEnded(0)) : "-1";
  json_obj["wx"] = wrist_x_sensor.isConnected() && wrist_x_sensor.available() ? String(wrist_x_sensor.getX()) : "-1";
  json_obj["wy"] = wrist_y_sensor.isConnected() && wrist_y_sensor.available() ? String(wrist_y_sensor.getX()) : "-1";
  json_obj["f0"] = thumb_finger_sensor.isConnected() && thumb_finger_sensor.available() ? String(thumb_finger_sensor.getX()) : "-1";
  json_obj["f1"] = index_finger_sensor.isConnected() && index_finger_sensor.available() ? String(index_finger_sensor.getX()) : "-1";
  json_obj["f2"] = middle_finger_sensor.isConnected() && middle_finger_sensor.available() ? String(middle_finger_sensor.getX()) : "-1";
  json_obj["f3"] = ring_finger_sensor.isConnected() && ring_finger_sensor.available() ? String(ring_finger_sensor.getX()) : "-1";
  json_obj["f4"] = little_finger_sensor.isConnected() && little_finger_sensor.available() ? String(little_finger_sensor.getX()) : "-1";
  
  serializeJson(json_obj, json_arr);
  
  mqtt.send(MQTT_ARMS_TOPIC, json_arr);

  LOG(json_arr);

  delay(DELAY_MS);
}

// void printQ(imu::Quaternion &q, imu::Quaternion &q2) {
//   imu::Quaternion q3;

//   q3 = q * q2.conjugate();

//   uint8_t system, gyro, accel, mag;
//   system = gyro = accel = mag = 0;
//   root_imu.getCalibration(&system, &gyro, &accel, &mag);

//   /* The data should be ignored until the system calibration is > 0 */
//   if (!system)
//   {
//       Serial.print("! ");
//   }

//   /* Display the individual values */
//   Serial.print("Sys:");
//   Serial.print(system, DEC);
//   Serial.print(" G:");
//   Serial.print(gyro, DEC);
//   Serial.print(" A:");
//   Serial.print(accel, DEC);
//   Serial.print(" M:");
//   Serial.print(mag, DEC);

//   Serial.print("\tQ: ");
//   Serial.print(q.x());
//   Serial.print("  ");
//   Serial.print(q.y());
//   Serial.print("  ");
//   Serial.print(q.z());
//   Serial.print("  ");
//   Serial.print(q.w());

//   Serial.print("\tQ2: ");
//   Serial.print(q2.x());
//   Serial.print("  ");
//   Serial.print(q2.y());
//   Serial.print("  ");
//   Serial.print(q2.z());
//   Serial.print("  ");
//   Serial.print(q2.w());

//   Serial.print("\tQ3: ");
//   Serial.print(q3.x());
//   Serial.print("  ");
//   Serial.print(q3.y());
//   Serial.print("  ");
//   Serial.print(q3.z());
//   Serial.print("  ");
//   Serial.println(q3.w());
// }