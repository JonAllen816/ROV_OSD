
//Arduino MAVLink  http://forum.arduino.cc/index.php?topic=382592.0
// https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/code-overview-object-avoidance.rst

/*
    The system id of the message should match the system id of the vehicle
    (default is "1" but can be changed using the SYSID_THISMAV parameter).
    The component id can be anything but MAV_COMP_ID_PATHPLANNER (195)
    or MAV_COMP_ID_PERIPHERAL (158) are probably good choices.

   # Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
  # http://mavlink.org/messages/common#DISTANCE_SENSOR
  def send_distance_message(dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = 0 MAV_DISTANCE_SENSOR_LASER Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    if args.verbose:
        log.debug("Sending mavlink distance_message:" +str(dist))
*/
uint32_t delta_time = 0; // used to control //display output rate
uint32_t counttime = 0, sumCounttime = 0; // used to control //display output rate

#include "mavlink.h"        // Mavlink interface
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <SendOnlySoftwareSerial.h>



SendOnlySoftwareSerial minimosd(14);

MS5803 Tempsensor(ADDRESS_HIGH);

float temperature;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
// Create Variable to store altitude in (m) for calculations;
double base_altitude = 70.0; // Altitude in Boston, MA)

const int Voltpin = A7;
const float RefVolts = 5.0; // 5-Volt board reference voltage on Nano
const float ResistFactor = 319.68; //Calculated from 1023.0*(R2/(R1 + R2) - where R1 = 2200 ohms and R2 = 1000 ohms for a 15V max voltage.

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
//MPU9250 IMU(Wire, 0x69);

void setup()
{

  Wire.begin();
  Serial.begin(115200);

  minimosd.begin(57600);
  Tempsensor.reset();
  Tempsensor.begin();
  pressure_baseline = Tempsensor.getPressure(ADC_4096);
  IMUsetup();

}


void loop() {
  
  //command_heartbeat();
  //command_TempPress();
  //command_ROVVOLT();
  //command_SystemStatus();

  IMUloop();
  sendMavlinkMessages();
  //command_IMUSENSORRAW();
  //command_Attitude();

}






/*
void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 100;
  //< The component sending the message.
  int compid = 120;

  // Define the system type, in this case ground control station
  uint8_t system_type = MAV_TYPE_SUBMARINE;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = 0;

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
  /*delta_time = millis() - counttime;
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

void command_SystemStatus() {
  uint16_t volts = ((analogRead(Voltpin) / ResistFactor * RefVolts) * 100);
  int sysid = 1;
  //< The component sending the message.
  int compid = 123;

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





void command_TempPress()
{
  temperature_c = Tempsensor.getTemperature(CELSIUS, ADC_512);
  temperature_f = Tempsensor.getTemperature(FAHRENHEIT, ADC_512);
  pressure_abs = Tempsensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, base_altitude);
  altitude_delta = altitude(pressure_abs, pressure_baseline);

  float press_abs = pressure_abs; //< Absolute pressure (hectopascal)
  float press_diff = pressure_relative; //< Differential pressure 1 (hectopascal)
  int16_t temperature = temperature_f; //< Temperature measurement (0.01 degrees celsius)

  //MAVLINK Pressure & Temperature MESSAGE
  int sysid = 1;
  //< The component sending the message.
  int compid = 175;

  uint32_t time_boot_ms = 0; //< Time since system boot

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_scaled_pressure_pack(sysid, compid, &msg, time_boot_ms, press_abs, press_diff, temperature);


  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);

  
  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print(" Pressure Abs:  ");
Serial.println(press_abs);
Serial.print("    Pressure Delta: ");
Serial.println(press_diff);
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




void command_ROVVOLT() {

  uint16_t volts = ((analogRead(Voltpin) / ResistFactor * RefVolts) * 10);

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;
  //< The component sending the message.
  int compid = 110;

  int32_t current_consumed; //< Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
  int32_t energy_consumed; //< Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
  int16_t battery_temperature; //< Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
  uint16_t voltages[10] = {volts}; //< Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
  int16_t current_battery = 50000; //< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
  uint8_t id = 7; //< Battery ID
  uint8_t battery_function = "main"; //< Function of the battery
  uint8_t battery_type = "PBA"; //< Type (chemistry) of the battery
  int8_t battery_remaining = 100; //< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_battery_status_pack(sysid, compid, &msg, id, battery_function, battery_type, battery_temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining);
  // Copy the message to the send buffer

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);

  command_SystemStatus();

}


void command_IMUSENSORRAW() {
  int sysid = 1;
  //< The component sending the message.
  int compid = 102;

  extern int16_t ax;
  extern int16_t ay;
  extern int16_t az;
  extern int16_t gx;
  extern int16_t gy;
  extern int16_t gz;
  extern int16_t mx;
  extern int16_t my;
  extern int16_t mz;

  uint32_t time_boot_ms = 0; //< Timestamp (milliseconds since system boot)
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




void command_Altitude()
{
  Tempsensor.begin();

  pressure_abs = Tempsensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, base_altitude);
  altitude_delta = altitude(pressure_abs, pressure_baseline);

  int sysid = 1;
  //< The component sending the message.
  int compid = 145;

  uint64_t time_usec; //< Timestamp (micros since boot or Unix epoch)
  float altitude_monotonic = pressure_baseline; //< This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change).
  //The only guarantee on this field is that it will never be reset and is consistent within a flight.
  //The recommended value for this field is the uncorrected barometric altitude at boot time.
  //This altitude will also drift and vary between flights.
  float altitude_amsl = 14; //< This altitude measure is strictly above mean sea level and might be non - monotonic (it might reset on events like GPS lock or when a new QNH value is set).
  //It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
  float altitude_local = pressure_abs; //< This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up - positive.
  float altitude_relative = pressure_relative; //< This is the altitude above the home position. It resets on each change of the current home position.
  float altitude_terrain; //< This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than - 1000 should be interpreted as unknown.
  float bottom_clearance; //< This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter.
  //It is generally a moving target. A negative value indicates no measurement available.

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_altitude_pack(sysid, compid, &msg, time_usec, altitude_monotonic, altitude_amsl, altitude_local, altitude_relative, altitude_terrain, bottom_clearance);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);

  Serial.write(buf, len);

  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print("Altitude Monotonic  ");
    Serial.print(altitude_monotonic);
    Serial.print("   Altitude AMSL    ");
    Serial.println(altitude_amsl);
    Serial.print("   Altitude Local    ");
    Serial.print(altitude_local);
    Serial.print("   Altitude Relative    ");
    Serial.println(altitude_relative);

  }
}







void command_Attitude() {

  int sysid = 1;
  //< The component sending the message.
  int compid = 108;

  extern float q[4];
  extern float roll;
  extern float pitch;
  extern float yaw;


  uint32_t time_boot_ms = 0;  //< Timestamp (milliseconds since system boot)
  float q1 = q[0];  //< Quaternion component 1, w (1 in null-rotation)

  float q2 = q[1];  //< Quaternion component 2, x (0 in null-rotation)

  float q3 = q[2];  //< Quaternion component 3, y (0 in null-rotation)

  float q4 = q[3];  //< Quaternion component 4, z (0 in null-rotation)

  float rollspeed = roll;  //< Roll angular speed (rad/s)

  float pitchspeed = pitch;  //< Pitch angular speed (rad/s)

  float yawspeed = yaw;  //< Yaw angular speed (rad/s)



  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  //mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, current_distance, type, id, orientation, covariance);
  mavlink_msg_attitude_quaternion_pack(sysid, compid, &msg, time_boot_ms, q1, q2, q3, q4, rollspeed, pitchspeed, yawspeed);
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  //delay(1);
  minimosd.write(buf, len);

  delta_time = millis() - counttime;
  if (delta_time > 500) {
    Serial.print("q1:  ");
    Serial.print(q1);
    Serial.print("    q2:  ");
    Serial.print(q2);
    Serial.print("    q3:  ");
    Serial.print(q3);
    Serial.print("    q4:  ");
    Serial.print(q4);
    Serial.print("    roll:  ");
    Serial.print(rollspeed);
    Serial.print("    pitch:  ");
    Serial.print(pitchspeed);
    Serial.print("    yaw:  ");
    Serial.println(yawspeed);
    counttime = millis();
  }
  }
*/
