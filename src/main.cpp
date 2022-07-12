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

#define DIRECTION_X 1
#define DIRECTION_Y 1
#define DIRECTION_Z 1

bool is_root_sensor_available = false;
bool is_shoulder_sensor_available = false;

// void printQ(imu::Quaternion &q, imu::Quaternion &q2);

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
  imu::Quaternion q1, q2, q3;

  q1 = root_imu.getQuat();
  q2 = shoulder_imu.getQuat();
  // printQ(q1, q2);

  q3 = q1 * q2.conjugate();

  Angles final_angles;
  final_angles = get_angles_from_quat(q3, DIRECTION_X, DIRECTION_Y, DIRECTION_Z);

  uint8_t system1, gyro1, accel1, mag1, system2, gyro2, accel2, mag2;
  system1 = gyro1 = accel1 = mag1 = system2 = gyro2 = accel2 = mag2 = 0;
  
  StaticJsonDocument<256> json_obj;
  char json_arr[256];

  root_imu.getCalibration(&system1, &gyro1, &accel1, &mag1);
  shoulder_imu.getCalibration(&system2, &gyro2, &accel2, &mag2);

  json_obj["cs1"] = is_root_sensor_available ? String(system1) : "-1";
  json_obj["cg1"] = is_root_sensor_available ? String(gyro1) : "-1";
  json_obj["ca1"] = is_root_sensor_available ? String(accel1) : "-1";
  json_obj["cm1"] = is_root_sensor_available ? String(mag1) : "-1";

  json_obj["cs2"] = is_shoulder_sensor_available ? String(system2) : "-1";
  json_obj["cg2"] = is_shoulder_sensor_available ? String(gyro2) : "-1";
  json_obj["ca2"] = is_shoulder_sensor_available ? String(accel2) : "-1";
  json_obj["cm2"] = is_shoulder_sensor_available ? String(mag2) : "-1";

  json_obj["sx"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_z) : "-1";
  json_obj["sx"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_z) : "-1";
  json_obj["sx"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_z) : "-1";
  json_obj["sy"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_x) : "-1";
  json_obj["sz"] = is_root_sensor_available && is_shoulder_sensor_available ? String(final_angles.from_y) : "-1";

  serializeJson(json_obj, json_arr);
  
  mqtt.send(MQTT_ARMS_TOPIC, json_arr);

  LOG(json_arr);

  delay(BNO055_SAMPLERATE_DELAY_MS);
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
  // Serial.print(mag, DEC);

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

