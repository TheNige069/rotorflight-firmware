/*
 * JR DMSS telemetry
 */

 /*
  * Authors:
  * Nigel Higgs
  * 
  * RCGroups users: DanielGA and garfield2412
  * Without their skills in reverse engineering the JR telelmetry procol, this would not be possible
  */

#pragma once

#include "common/time.h"
//Sensor Ids
// The value for the Constant in the message
#define JR_TELEMTRY_SENSOR_MSG_CONSTANT 0x03
//#define JR_TELEMETRY_NO_SENSOR_ID     0x80
// Temperature
#define JR_TELEMETRY_TEMPERATURE_SENSOR_ID  0xE1
// Motor RPM
#define JR_TELEMETRY_RPM_SENSOR_ID  0xE2
// Pressure/Vario/Height
#define JR_TELEMETRY_ALTIMETER_SENSOR_ID  0xE3
//#define JR_TELEMETRY_0XE4_SENSOR_ID  0xE4     // Not used
// Airspeed
#define JR_TELEMETRY_AIRSPEED_SENSOR_ID  0xE5
//#define JR_TELEMETRY_0XE6_SENSOR_ID  0xE6     // Not used
//#define JR_TELEMETRY_0XE7_SENSOR_ID  0xE7     // Not used
// ESC Telemetry
#define JR_TELEMETRY_ESC_SENSOR_ID  0xE8
//#define JR_TELEMETRY_0XE9_SENSOR_ID  0xE9     // Not used
//#define JR_TELEMETRY_0XEA_SENSOR_ID  0xEA     // Not used
//#define JR_TELEMETRY_0XEB_SENSOR_ID  0xEB     // Not used
//#define JR_TELEMETRY_0XEC_SENSOR_ID  0xEC     // Not used
//#define JR_TELEMETRY_0XED_SENSOR_ID  0xED     // Not used
//#define JR_TELEMETRY_0XEE_SENSOR_ID  0xEE     // Not used
//#define JR_TELEMETRY_0XEF_SENSOR_ID  0xEF     // Not used

// These are the Sub-ID's for the relevant sensors
// Vario sensor sub-functions
#define JR_TELEMETRY_ALTIMETER_PRESSURE_SENSOR_ID  0xE1
#define JR_TELEMETRY_ALTIMETER_VARIO_SENSOR_ID  0xE2
#define JR_TELEMETRY_ALTIMETER_HEIGHT_SENSOR_ID  0xE3
// ESC sensor sub-functions
#define JR_TELEMETRY_ESC_FBATVOLTAGE_SENSOR_ID  0x00
#define JR_TELEMETRY_ESC_IMOT_SENSOR_ID  0x80
#define JR_TELEMETRY_ESC_FBATCAPACITY_SENSOR_ID  0x8
#define JR_TELEMETRY_ESC_POWER_SENSOR_ID  0x90

/*
Temperature
    rpmMotor
Pressure
Vario
    Height
Airspeed
F-Battery Voltage (ESC Voltage)
Imot (ESC, Shunt)
F-Battery Capacity (ESC Flight Pack Used mAh)
Power (ESC Power)
*/

// one message for each sensor packet
typedef struct jrdmssTelemetryMessage_s {
    uint8_t sensor;           // Sensor ID - what sensor are we sending data for
    uint8_t constant;         // Unused, set to 0x03
    uint8_t sensorindex;      // Sensor sub id. if no sub id, then 0x00
    uint8_t sensorvauelhigh;  // high byte of 16 bit value
    uint8_t sensorvaluelow;   // low byte of 16 bit value
    uint8_t crc;              // CRC value of above bytes
} jrdmssTelemetryMessage_t;

// Holds the actual values of the sensor
typedef struct jrdmssTelemetry_s {
    jrdmssTelemetryMessage_t temperature;    // 0xE1 temperature 1. offset of 20. a value of 20 = 0?C
    jrdmssTelemetryMessage_t rpm;            // 0xE2 RPM in 10 RPM steps. 300 = 3000rpm
    jrdmssTelemetryMessage_t pressure;       // 0xE3, 0x01 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    jrdmssTelemetryMessage_t climbrate;      // 0xE3, 0x02 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    jrdmssTelemetryMessage_t altitude;       // 0xE3, 0x03 altitude in meters. 
    jrdmssTelemetryMessage_t batvoltage;     // in 0.01V steps
    jrdmssTelemetryMessage_t batamperage;    // in 0.1A steps
    jrdmssTelemetryMessage_t batmah;         // in 1mah steps
    jrdmssTelemetryMessage_t batwatt;        // in 0.1W steps
} jrdmssTelemetry_t;

void handleJRDMSSTelemetry(timeUs_t currentTimeUs);
void checkJRDMSSTelemetryState(void);

void initJRDMSSTelemetry(void);
void configureJRDMSSTelemetryPort(void);
void freeJRDMSSTelemetryPort(void);
