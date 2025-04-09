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
  /*
  #define HOTTV4_RXTX 4

  #define HOTTV4_TEXT_MODE_REQUEST_ID       0x7f
  #define HOTTV4_BINARY_MODE_REQUEST_ID     0x80

  #define HOTTV4_BUTTON_DEC    0xB
  #define HOTTV4_BUTTON_INC    0xD
  #define HOTTV4_BUTTON_SET    0x9
  #define HOTTV4_BUTTON_NIL    0xF
  #define HOTTV4_BUTTON_NEXT   0xE
  #define HOTTV4_BUTTON_PREV   0x7

  #define HOTT_EAM_OFFSET_HEIGHT       500
  #define HOTT_EAM_OFFSET_M2S           72
  #define HOTT_EAM_OFFSET_M3S          120
  #define HOTT_EAM_OFFSET_TEMPERATURE   20

  #define HOTT_GPS_ALTITUDE_OFFSET 500
  */
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
#define JR_TELEMETRY_ESC_POWER_SENSOR_ID  0x009

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

// Holds the actual values of the sensor
typedef struct jrdmssTelemetry_s {
    uint8_t temperature;    // 0xE1 temperature 1. offset of 20. a value of 20 = 0?C
    uint8_t rpm;            // 0xE2 RPM in 10 RPM steps. 300 = 3000rpm
    uint8_t pressure;       // 0xE3, 0x01 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    uint8_t climbrate;      // 0xE3, 0x02 climb rate in 0.01m/s. Value of 30000 = 0.00 m/s
    uint8_t altitude;       // 0xE3, 0x03 altitude in meters. 
    uint8_t current;        // current in 0.1A steps

} jrdmssTelemetry_t;

// one message for each sensor packet
typedef struct jrdmssTelemetryMessage_s {
    uint8_t sensor;           // Sensor ID - what sensor are we sending data for
    uint8_t constant;         // Unused, set to 0x03
    uint8_t sensorindex;      // Sensor sub id. if no sub id, then 0x00
    uint8_t sensorvauelhigh;  // high byte of 16 bit value
    uint8_t sensorvaluelow;   // low byte of 16 bit value
    uint8_t crc;              // CRC value of above bytes
} jrdmssTelemetryMessage_t;
/*
typedef struct HOTT_AIRESC_MSG_s {
    uint8_t start_byte;      //#01 constant value 0x7c
    uint8_t gps_sensor_id;   //#02 constant value 0x8c
    uint8_t warning_beeps;   //#03 1=A 2=B ...
    // A
    // L
    // O
    // Z
    // C
    // B
    // N
    // M
    // D
    //

    uint8_t sensor_id;      //#04 constant value 0xc0
    uint8_t alarm_invers1;  //#05 TODO: more info
    uint8_t alarm_invers2;  //#06 TODO: more info
    uint8_t input_v_L;      //#07 Input voltage low byte
    uint8_t input_v_H;      //#08
    uint8_t input_v_min_L;  //#09 Input min. voltage low byte
    uint8_t input_v_min_H;  //#10
    uint8_t batt_cap_L;     //#11 battery capacity in 10mAh steps
    uint8_t batt_cap_H;     //#12
    uint8_t esc_temp;       //#13 ESC temperature
    uint8_t esc_max_temp;   //#14 ESC max. temperature
    uint8_t current_L;      //#15 Current in 0.1 steps
    uint8_t current_H;      //#16
    uint8_t current_max_L;  //#17 Current max. in 0.1 steps
    uint8_t current_max_H;  //#18
    uint8_t rpm_L;          //#19 RPM in 10U/min steps
    uint8_t rpm_H;          //#20
    uint8_t rpm_max_L;      //#21 RPM max
    uint8_t rpm_max_H;      //#22
    uint8_t throttle;       //#23 throttle in %
    uint8_t speed_L;        //#24 Speed
    uint8_t speed_H;        //#25
    uint8_t speed_max_L;    //#26 Speed max
    uint8_t speed_max_H;    //#27
    uint8_t bec_v;          //#28 BEC voltage
    uint8_t bec_min_v;      //#29 BEC min. voltage
    uint8_t bec_current;    //#30 BEC current
    uint8_t bec_current_max_L;  //#31 BEC max. current
    uint8_t bec_current_max_H;  //#32 TODO: not really clear why 2 bytes...
    uint8_t pwm;            //#33 PWM
    uint8_t bec_temp;       //#34 BEC temperature
    uint8_t bec_temp_max;   //#35 BEC highest temperature
    uint8_t motor_temp;     //#36 Motor or external sensor temperature
    uint8_t motor_temp_max; //#37 Highest motor or external sensor temperature
    uint8_t motor_rpm_L;    //#38 Motor or external RPM sensor (without gear)
    uint8_t motor_rpm_H;    //#39
    uint8_t motor_timing;   //#40 Motor timing
    uint8_t motor_timing_adv; //#41 Motor advanced timing
    uint8_t motor_highest_current;  //#42 Motor number (1-x) with highest current
    uint8_t version;        //#43   Version number (highest current motor 1-x)
    uint8_t stop_byte;      //#44 constant value 0x7d
} HOTT_AIRESC_MSG_t;
*/
void handleJRdmssTelemetry(timeUs_t currentTimeUs);
void checkJRdmssTelemetryState(void);

void initJRdmssTelemetry(void);
void configureJRdmssTelemetryPort(void);
void freeJRdmssTelemetryPort(void);

//uint32_t getHoTTTelemetryProviderBaudRate(void);

//void hottPrepareGPSResponse(HOTT_GPS_MSG_t* hottGPSMessage);
