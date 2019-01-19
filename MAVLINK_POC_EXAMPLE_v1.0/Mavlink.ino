/**
  MinimOSD standard configuration
  https://code.google.com/p/minimosd-extra/wiki/APM

  - 2hz for waypoints, GPS raw, fence data, current waypoint, etc
  - 5hz for attitude and simulation state
  - 2hz for VFR_Hud data
  - 3hz for AHRS, Hardware Status, Wind
  - 2hz for location data
  - 2hz for raw imu sensor data
  - 5hz for radio input or radio output data

**/

void sendMavlinkMessages() {
  unsigned long currtime = millis();

  static unsigned long heartbeat_1hz = 0;
  static unsigned long heartbeat_2hz = 0;
  static unsigned long heartbeat_3hz = 0;
  static unsigned long heartbeat_5hz = 0;
  static unsigned long dimming = 0;



  int sysid = 100; // System ID, 1-255
  int compid = 50; // Component/Subsystem ID, 1-255

  //1hz for mavlink heart beat
  if (currtime >= heartbeat_1hz ) {
    sendHeartBeat();
    heartbeat_1hz = currtime + 1000;
    dimming = currtime;
  }

  //2hz for waypoints, GPS raw, fence data, current waypoint, etc
  //2hz for VFR_Hud data
  if (currtime >= heartbeat_2hz ) {
    SendIMUData();
    SendTempPress();
    //sendVfrHud();
    //sendGlobalPosition();
    heartbeat_2hz = currtime + 500;
    dimming = currtime;
  }

  //3hz for AHRS, Hardware Status, Wind
  if (currtime >= heartbeat_3hz) {
    SendSystemStatus();
    heartbeat_3hz = currtime + 333;
    dimming = currtime;
  }

  //5hz for attitude and simulation state
  //5hz for radio input or radio output data
  if (currtime >= heartbeat_5hz ) {
    SendAttitude();
    //sendRawChannels();
    heartbeat_5hz = currtime + 200;
    dimming = currtime;
  }
}
/*if( currtime - dimming < 15 ) { //LED is on for 30ms when a message is sent
  LEDPIN_ON
  } else {
  LEDPIN_OFF
  }
  }*/


void sendHeartBeat() {

  //< ID 1 for this system
  int sysid = 100;
  //< The component sending the message.
  int compid = 120;

  // Define the system type, in this case ground control station
  uint8_t system_type = MAV_TYPE_QUADROTOR;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = 1;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  //delay(1);
  minimosd.write(buf, len);
  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print("sysid     ");
    Serial.print(sysid);
    Serial.print("compid    ");
    Serial.print(compid);
    //Serial.print("message    ");
    //Serial.print(msg);
    Serial.print("system_type     ");
    Serial.print(system_type);
    Serial.print("autopilot_type     ");
    Serial.print(autopilot_type);
    Serial.print("system_mode     ");
    Serial.print(system_mode);
    Serial.print("custom_mode     ");
    Serial.print(custom_mode);
    Serial.print("system_state     ");
    Serial.println(system_state);
  }
}


void SendIMUData() {

  int sysid = 100;
  //< The component sending the message.
  int compid = 200;

  extern int16_t ax;
  extern int16_t ay;
  extern int16_t az;
  extern int16_t gx;
  extern int16_t gy;
  extern int16_t gz;
  extern int16_t mx;
  extern int16_t my;
  extern int16_t mz;

  uint32_t time_boot_ms = millis(); //< Timestamp (milliseconds since system boot)
  int16_t xacc = ax; //< X acceleration (mg)//

  int16_t yacc = ay; //< Y acceleration (mg)//

  int16_t zacc = az; //< Z acceleration (mg)//

  int16_t xgyro = gx; //< Angular speed around X axis (millirad /sec)//

  int16_t ygyro = gy; //< Angular speed around Y axis (millirad /sec)//

  int16_t zgyro = gz; //< Angular speed around Z axis (millirad /sec)//

  int16_t xmag = mx; //< X Magnetic field (milli tesla)//

  int16_t ymag = my; //< Y Magnetic field (milli tesla)//

  int16_t zmag = mz; //< Z Magnetic field (milli tesla)//



  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_scaled_imu_pack(sysid, compid, &msg, time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);

  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print("xacc    ");
    Serial.print(xacc);
    Serial.print("    ");
    Serial.print("yacc    ");
    Serial.print(yacc);
    Serial.print("    ");
    Serial.print("zacc    ");
    Serial.print(zacc);
    Serial.print("    ");
    Serial.print("xgyro    ");
    Serial.print(xgyro);
    Serial.print("    ");
    Serial.print("ygyro    ");
    Serial.print(ygyro);
    Serial.print("    ");
    Serial.print("zgyrp    ");
    Serial.print(zgyro);
    Serial.print("    ");
    Serial.print("xmag    ");
    Serial.print(xmag);
    Serial.print("    ");
    Serial.print("ymag    ");
    Serial.print(ymag);
    Serial.print("    ");
    Serial.print("zmag    ");
    Serial.print(zmag);
    Serial.println("    ");
  }
}


void SendTempPress()
{
  //temperature_c = Tempsensor.getTemperature(CELSIUS, ADC_512);
  temperature = Tempsensor.getTemperature(FAHRENHEIT, ADC_512);
  pressure_abs = Tempsensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, base_altitude);
  altitude_delta = altitude(pressure_abs, pressure_baseline);


  //MAVLINK Pressure & Temperature MESSAGE
  int sysid = 100;
  //< The component sending the message.
  int compid = 175;

  uint32_t time_boot_ms = millis(); //< Time since system boot

  //float press_abs = pressure_abs; //< Absolute pressure (hectopascal)
  //float press_diff = pressure_relative; //< Differential pressure 1 (hectopascal)
  //int16_t temperature = temperature_f; //< Temperature measurement (0.01 degrees celsius)

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_scaled_pressure_pack(sysid, compid, &msg, time_boot_ms, pressure_abs, pressure_relative, temperature);


  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);


  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print(" Pressure Abs:  ");
    Serial.println(pressure_abs);
    Serial.print("    Pressure Delta: ");
    Serial.println(pressure_relative);
    Serial.print("Temperature F:   ");
    Serial.println(temperature);
  }
}

float sealevel(float P, float A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return (P / pow(1 - (A / 44330.0), 5.255));
}


float altitude(float P, float P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}



void SendSystemStatus() {

  int sysid = 100;
  //< The component sending the message.
  int compid = 123;

  uint16_t volts = ((analogRead(A7) / 319.68 * 5.0) * 100);

  uint32_t onboard_control_sensors_present = 1; //< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
  uint32_t onboard_control_sensors_enabled = 1; //< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
  uint32_t onboard_control_sensors_health = 1; //< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
  uint16_t load; //< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
  uint16_t voltage_battery = (volts); //< Battery voltage, in millivolts (1 = 1 millivolt)
  int16_t current_battery; //< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
  uint16_t drop_rate_comm; //< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
  uint16_t errors_comm; //< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
  uint16_t errors_count1; //< Autopilot-specific errors
  uint16_t errors_count2; //< Autopilot-specific errors
  uint16_t errors_count3; //< Autopilot-specific errors
  uint16_t errors_count4; //< Autopilot-specific errors
  int8_t battery_remaining; //< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_sys_status_pack(sysid, compid, &msg, onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);

  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print("Volts:   ");
    Serial.println(volts);
  }
}



void SendAttitude() {

  int sysid = 100;
  //< The component sending the message.
  int compid = 108;

  //extern float q[4];
  extern float roll;
  extern float pitch;
  extern float yaw;


  uint32_t time_boot_ms = millis();  //< Timestamp (milliseconds since system boot)
  //float q1 = q[0];  //< Quaternion component 1, w (1 in null-rotation)
  //float q2 = q[1];  //< Quaternion component 2, x (0 in null-rotation)
  //float q3 = q[2];  //< Quaternion component 3, y (0 in null-rotation)
  //float q4 = q[3];  //< Quaternion component 4, z (0 in null-rotation)
  float rollrads = roll;  //< Roll angular speed (rad/s)
  float pitchrads = pitch;  //< Pitch angular speed (rad/s)
  float yawrads = yaw;  //< Yaw angular speed (rad/s)
  float rollspeed = 0.0;  //< Roll angular speed (rad/s)
  float pitchspeed = 0;  //< Pitch angular speed (rad/s)
  float yawspeed = 0;  //< Yaw angular speed (rad/s)



  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  //mavlink_msg_attitude_quaternion_pack(sysid, compid, &msg, time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed);
  mavlink_msg_attitude_pack(sysid, compid, &msg, time_boot_ms, rollrads, pitchrads, yawrads, rollspeed, pitchspeed, yawspeed);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);
  delta_time = millis() - counttime;
  if (delta_time > 500) {
    //Serial.print("q1:  ");
    //Serial.print(q1);
    //Serial.print("    q2:  ");
    //Serial.print(q2);
    //Serial.print("    q3:  ");
    //Serial.print(q3);
    //Serial.print("    q4:  ");
    //Serial.print(q4);
    Serial.print("    roll:  ");
    Serial.print(roll);
    Serial.print("    pitch:  ");
    Serial.print(pitch);
    Serial.print("    yaw:  ");
    Serial.println(yaw);
    counttime = millis();
  }
}





