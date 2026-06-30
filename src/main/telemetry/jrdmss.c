/*
 * JR DMSS telemetry
 */

 /*
  * jrdmss.c
  *
  * Authors:
  * Nigel Higgs
  *
  * RCGroups users: DanielGA and garfield2412
  * Without their skills in reverse engineering the JR telelmetry procol, the addition of JR telemetry to Rotorflight would not be possible
  *
  * JR DMSS telemetry is implemented in JR equipment using a bi-directional protocol over a single wire.
  *
  * Generally the receiver sends a single request byte out using normal uart signals, then waits a short period for a
  * multiple byte response and checksum byte before it sends out the next request byte. Each request from the receiver
  * is the ID of the sensor that it wants the information for
  * Each response byte must be sent with a protocol specific delay between them.
  */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_JRDMSS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/crc.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "telemetry/jrdmss.h"
//#include "rx/xbus.h"            // Need this for the CRC8 calculation
#include "telemetry/telemetry.h"

#define JRDMSS_DEBUG

#ifdef JRDMSS_DEBUG
#include "common/printf.h"
#endif

#define JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ ((1000 * 1000) / 5)
#define JRDMSS_RX_SCHEDULE 4000
#define JRDMSS_TX_DELAY_US 3000
//#define MILLISECONDS_IN_A_SECOND 1000

//static uint32_t rxSchedule = JRDMSS_RX_SCHEDULE;
//static uint32_t txDelayUs = JRDMSS_TX_DELAY_US;

static uint32_t lastJRDMSSRequestCheckAt = 0;
static uint32_t lastMessagesPreparedAt = 0;

static bool jrdmssIsSending = false;

static uint8_t* jrdmssMsg = NULL;
static uint8_t jrdmssMsgRemainingBytesToSendCount;
static uint8_t jrdmssMsgCrc;

#define JRDMSS_CRC_SIZE (sizeof(jrdmssMsgCrc))

#define JRDMSS_BAUDRATE 250000
#define JRDMSS_PORT_MODE MODE_RXTX // must be opened in RXTX so that TX and RX pins are allocated.

static serialPort_t* jrdmssPort = NULL;
static const serialPortConfig_t* portConfig;

static bool jrdmssTelemetryEnabled = false;
static portSharing_e jrdmssPortSharing;

//static jrdmssTelemetry_t jrdmssTelemetryMessage;
static jrdmssTelemetryMessage_t jrdmssTelemetryMessage;
static jrdmssTelemetry_t jrdmssTelemetryValues;

/*
 * TODO: Finish update for JR
 *
 * typedef struct jrdmssTelemetryMessage_s {
 *   uint8_t sensor;           // Sensor ID - what sensor are we sending data for
 *   uint8_t constant;         // Unused, set to 0x03
 *   uint8_t sensorindex;      // Sensor sub id. if no sub id, then 0x00
 *   uint8_t sensorvauelhigh;  // high byte of 16 bit value
 *   uint8_t sensorvaluelow;   // low byte of 16 bit value
 *   uint8_t crc;              // CRC value of above bytes
 * } jrdmssTelemetryMessage_t;
 */
static void initialiseJRDMSSMessage(jrdmssTelemetryMessage_t *msg, size_t size, uint8_t sensor, uint8_t sensorindex)
{
    memset(msg, 0, size);
    msg->sensor = sensor;
    msg->constant = JR_TELEMTRY_SENSOR_MSG_CONSTANT;
    msg->sensorindex = sensorindex;
    msg->sensorvauelhigh = 0x00;
    msg->sensorvaluelow = 0x0;
    msg->crc = 0x0;
}

static void initialiseMessages(void)
{
    initialiseJRDMSSMessage(&jrdmssTelemetryMessage,            sizeof(jrdmssTelemetryMessage), 0x0, 0x0);

    initialiseJRDMSSMessage(&jrdmssTelemetryValues.temperature, sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_TEMPERATURE_SENSOR_ID, 0x0);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.rpm,         sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_RPM_SENSOR_ID, 0x0);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.pressure,    sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ALTIMETER_SENSOR_ID, JR_TELEMETRY_ALTIMETER_PRESSURE_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.climbrate,   sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ALTIMETER_SENSOR_ID, JR_TELEMETRY_ALTIMETER_VARIO_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.altitude,    sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ALTIMETER_SENSOR_ID, JR_TELEMETRY_ALTIMETER_HEIGHT_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.batvoltage,  sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ESC_SENSOR_ID, JR_TELEMETRY_ESC_FBATVOLTAGE_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.batamperage, sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ESC_SENSOR_ID, JR_TELEMETRY_ESC_IMOT_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.batmah,      sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ESC_SENSOR_ID, JR_TELEMETRY_ESC_FBATCAPACITY_SENSOR_ID);
    initialiseJRDMSSMessage(&jrdmssTelemetryValues.batwatt,     sizeof(jrdmssTelemetryMessage), JR_TELEMETRY_ESC_SENSOR_ID, JR_TELEMETRY_ESC_POWER_SENSOR_ID);
}

static inline void jrdmssUpdateBattery(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t volt = getLegacyBatteryVoltage();

    msg->sensorvaluelow = volt & 0xFF;
    msg->sensorvauelhigh = volt >> 8;

    //uint8_t crc = crc8((uint8_t*)msg, 5);
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}

static inline void jrdmssUpdateCurrentMeter(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t amp = 123; // getLegacyBatteryCurrent();

    msg->sensorvaluelow = amp & 0xFF;
    msg->sensorvauelhigh = amp >> 8;
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}

static inline void jrdmssUpdateBatteryDrawnCapacity(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t mAh = getBatteryCapacityUsed() / 10;

    msg->sensorvaluelow = mAh & 0xFF;
    msg->sensorvauelhigh = mAh >> 8;
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}

static inline void jrdmssUpdateAltitude(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t altitude = (getEstimatedAltitudeCm() / 100);

    msg->sensorvaluelow = altitude & 0xFF;
    msg->sensorvauelhigh = altitude >> 8;
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}

#ifdef USE_VARIO
static inline void jrdmssUpdateClimbrate(jrdmssTelemetryMessage_t* msg)
{
    const int32_t vario = getEstimatedVarioCms();

    msg->sensorvaluelow = (30000 + vario) & 0x00FF;
    msg->sensorvauelhigh = (30000 + vario) >> 8;
    //msg->climbrate3s = 120 + (vario / 100);
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}
#endif

static inline void jrdmssUpdateRPM(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t rpm = 2124;  // telemetrySensorValue(TELEM_HEADSPEED);

    msg->sensorvaluelow = rpm & 0xFF;
    msg->sensorvauelhigh = rpm >> 8;
    msg->crc = crc8_dallas((uint8_t*)msg, 5);
}

static void updateJRDMSSTelemetryValues(jrdmssTelemetryMessage_t* msg)
{
    jrdmssUpdateBattery(msg);
    jrdmssUpdateCurrentMeter(msg);
    jrdmssUpdateBatteryDrawnCapacity(msg);
    jrdmssUpdateAltitude(msg);
#ifdef USE_VARIO
    jrdmssUpdateClimbrate(msg);
#endif
    jrdmssUpdateRPM(msg);
}

static void jrdmssSerialWrite(uint8_t c)
{
    static uint8_t serialWrites = 0;
    serialWrites++;
    serialWrite(jrdmssPort, c);
}

void freeJRDMSSTelemetryPort(void)
{
    closeSerialPort(jrdmssPort);
    jrdmssPort = NULL;
    jrdmssTelemetryEnabled = false;
}

void initJRDMSSTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_JRDMSS);

    if (portConfig) {

        jrdmssPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_JRDMSS);

        // Initialise the debug serial port for dprintf messages.
        // Remove once testing complete
#ifdef JRDMSS_DEBUG
        //initDebugSerial(SERIAL_PORT_USART3, 250000);
        //printfSerialInit(PRINTF_SERIAL_PORT, PRINTF_SERIAL_SPEED, PRINTF_SERIAL_OPTIONS);
        //printfSerialInit(SERIAL_PORT_USART3, 250000, 0);
        tfp_printf("JR Telemetry enabled\n");
#endif

        initialiseMessages();
    }
}

static void flushJRDMSSRxBuffer(void)
{
    while (serialRxBytesWaiting(jrdmssPort) > 0) {
        serialRead(jrdmssPort);
    }
#ifdef JRDMSS_DEBUG
    tfp_printf("flushJRDMSSRxBuffer...\n");
#endif
}

static void JRDMSSTelemetryOnUsart(serialPort_t* instance, portMode_e mode)
{
#ifdef JRDMSS_DEBUG
    tfp_printf("JRDMSSTelemetryOnUsart\n");
#endif
    closeSerialPort(jrdmssPort);

    portOptions_e portOptions =
        (telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (telemetryConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

    jrdmssPort = openSerialPort(instance->identifier, FUNCTION_TELEMETRY_JRDMSS, NULL, NULL, JRDMSS_BAUDRATE, mode, portOptions);
}

static bool jrdmssIsUsingHardwareUART(void)
{
    return !(portConfig->identifier == SERIAL_PORT_SOFTSERIAL1 || portConfig->identifier == SERIAL_PORT_SOFTSERIAL2);
}

static void jrdmssConfigurePortForTX(void)
{
#ifdef JRDMSS_DEBUG
    tfp_printf("jrdmssConfigurePortForTX\n");
#endif
    if (jrdmssIsUsingHardwareUART()) {
        JRDMSSTelemetryOnUsart(jrdmssPort, MODE_TX);
    }
    else {
        serialSetMode(jrdmssPort, MODE_TX);
    }
    jrdmssIsSending = true;
    jrdmssMsgCrc = 0;
}

static void jrdmssConfigurePortForRX(void)
{
#ifdef JRDMSS_DEBUG
    tfp_printf("jrdmssConfigurePortForRX\n");
#endif
    if (jrdmssIsUsingHardwareUART()) {
        JRDMSSTelemetryOnUsart(jrdmssPort, MODE_RX);
    }
    else {
        serialSetMode(jrdmssPort, MODE_RX);
    }
    jrdmssMsg = NULL;
    jrdmssIsSending = false;
    flushJRDMSSRxBuffer();
}

void configureJRDMSSTelemetryPort(void)
{
#ifdef JRDMSS_DEBUG
    tfp_printf("configureJRDMSSTelemetryPort\n");
#endif
    if (!portConfig) {
#ifdef JRDMSS_DEBUG
        tfp_printf("configureJRDMSSTelemetryPort -> Port not configured\n");
#endif
        return;
    }

    portOptions_e portOptions = SERIAL_NOT_INVERTED |
        (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (telemetryConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

    jrdmssPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_JRDMSS, NULL, NULL, JRDMSS_BAUDRATE, JRDMSS_PORT_MODE, portOptions);

    if (!jrdmssPort) {
#ifdef JRDMSS_DEBUG
        tfp_printf("configureJRDMSSTelemetryPort - Open port failed\n");
#endif
        return;
    }

    jrdmssConfigurePortForRX();

    jrdmssTelemetryEnabled = true;
}

/*
static void jrdmssSendResponse(uint8_t* buffer, int length)
{
    if (jrdmssIsSending) {
        return;
    }

    jrdmssMsg = buffer;
    jrdmssMsgRemainingBytesToSendCount = length + JRDMSS_CRC_SIZE;
}
*/
//static inline void jrdmssSendTelemetryResponse(void)
//{
//    jrdmssSendResponse((uint8_t*)&jrdmssTelemetryMessage, sizeof(jrdmssTelemetryMessage));
//}

//static void jrdmssPrepareMessages(void) {
//    updateJRDMSSTelemetryValues(&jrdmssTelemetryMessage);
//}

static void processJRDMSSTelemetryRequest(uint8_t sensorID)
{
#ifdef JRDMSS_DEBUG
    static uint8_t jrdmssBinaryRequests = 0;
    static uint8_t jrdmssRequests = 0;
    tfp_printf("ID requested: %d\n", sensorID);
#endif

    switch (sensorID) {
    case JR_TELEMETRY_TEMPERATURE_SENSOR_ID:
        jrdmssTelemetryMessage = jrdmssTelemetryValues.temperature;
        break;
    case JR_TELEMETRY_RPM_SENSOR_ID:
        jrdmssTelemetryMessage = jrdmssTelemetryValues.rpm;
        break;
    case JR_TELEMETRY_ALTIMETER_SENSOR_ID:
        break;
    //case JR_TELEMETRY_AIRSPEED_SENSOR_ID:
    //    break;
    case JR_TELEMETRY_ESC_SENSOR_ID:
#ifdef JRDMSS_DEBUG
        jrdmssRequests++;
#endif
        //jrdmssTelemetryMessage = jrdmssTelemetryValues.

        break;
    }

    //jrdmssSendTelemetryResponse();
    //jrdmssSendResponse((uint8_t*)&jrdmssTelemetryMessage, sizeof(jrdmssTelemetryMessage));
    if (!jrdmssIsSending) {
        jrdmssMsg = (uint8_t*)&jrdmssTelemetryMessage;
        jrdmssMsgRemainingBytesToSendCount = sizeof(jrdmssTelemetryMessage) + JRDMSS_CRC_SIZE;
    }

#ifdef JRDMSS_DEBUG
    jrdmssBinaryRequests++;
    debug[0] = jrdmssBinaryRequests;
    debug[2] = jrdmssRequests;
#endif
}

static void jrdmssCheckSerialData(uint32_t currentMicros)
{
    static bool lookingForRequest = true;
    bool bExitflag = false;

    const uint8_t bytesWaiting = serialRxBytesWaiting(jrdmssPort);

    if (bytesWaiting > 0) {
#ifdef JRDMSS_DEBUG
        tfp_printf("JR Telemetry - Bytes waiting... [ %d ]\n", bytesWaiting);
#endif

        if (bytesWaiting != 1) {
            flushJRDMSSRxBuffer();
            lookingForRequest = true;
            //return;
        }
        else {
            if (lookingForRequest) {
                lastJRDMSSRequestCheckAt = currentMicros;
                lookingForRequest = false;
                bExitflag = true;
                //return;
            }
            else {
                //bool enoughTimePassed = currentMicros - lastJRDMSSRequestCheckAt >= JRDMSS_RX_SCHEDULE; // rxSchedule;

                //if (!enoughTimePassed) {
                if (currentMicros - lastJRDMSSRequestCheckAt < JRDMSS_RX_SCHEDULE) {
                    bExitflag = true;
#ifdef JRDMSS_DEBUG
                    tfp_printf("JR Telemetry - jrdmssCheckSerialData... not enough time passed: [ %ld < 4000]\n", currentMicros - lastJRDMSSRequestCheckAt);
#endif
                    //return;
                }
                lookingForRequest = true;
            }
            if (bExitflag == false) {
                const uint8_t requestId = serialRead(jrdmssPort);   // This is the sensor ID that we need to send data for
                //const uint8_t address = serialRead(jrdmssPort);
#ifdef JRDMSS_DEBUG
                tfp_printf("JR Telemetry - jrdmssCheckSerialData... [ %d ]\n", requestId);
#endif

                //if ((requestId == 0) || (requestId == JR_TELEMTRY_SENSOR_MSG_CONSTANT) || (address == JRDMSS_TELEMETRY_NO_SENSOR_ID)) {
                if (requestId > 0) {
                    processJRDMSSTelemetryRequest(requestId);
                }
            }
        }
    }
}

/*
 * TODO: Complete for JR telemetry
 */
static void sendJRDMSSTelemetryData(void) {

    if (!jrdmssIsSending) {
        jrdmssConfigurePortForTX();
        return;
    }

    if (jrdmssMsgRemainingBytesToSendCount == 0) {
        jrdmssConfigurePortForRX();
        return;
    }

    --jrdmssMsgRemainingBytesToSendCount;
    if (jrdmssMsgRemainingBytesToSendCount == 0) {
        jrdmssSerialWrite(jrdmssMsgCrc++);
        return;
    }

    jrdmssMsgCrc += *jrdmssMsg;
    jrdmssSerialWrite(*jrdmssMsg++);
}

//static inline bool shouldPrepareJRDMSSMessages(uint32_t currentMicros)
//{
//    return currentMicros - lastMessagesPreparedAt >= JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ;
//}

static inline bool shouldCheckForJRDMSSRequest(void)
{
    if (jrdmssIsSending) {
        return false;
    }
    return true;
}

void checkJRDMSSTelemetryState(void)
{
    const bool newTelemetryEnabledValue = telemetryDetermineEnabledState(jrdmssPortSharing);

    if (newTelemetryEnabledValue == jrdmssTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue) {
        configureJRDMSSTelemetryPort();
    }
    else {
        freeJRDMSSTelemetryPort();
    }
}
/*
void handleJRDMSSTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t serialTimer;

    if (!jrdmssTelemetryEnabled) {
        return;
    }

    // Do we need this??
    if (currentTimeUs - lastMessagesPreparedAt >= JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ) {
        updateJRDMSSTelemetryValues(&jrdmssTelemetryMessage); // jrdmssPrepareMessages();
        lastMessagesPreparedAt = currentTimeUs;
    }

    if (shouldCheckForJRDMSSRequest()) {
        jrdmssCheckSerialData(currentTimeUs);
    }

    if (!jrdmssMsg)
        return;

    if (jrdmssIsSending) {
        if (currentTimeUs - serialTimer < txDelayUs) {
            return;
        }
    }
    sendJRDMSSTelemetryData();
    serialTimer = currentTimeUs;
}
*/
void handleJRDMSSTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t serialTimer;

    if (jrdmssTelemetryEnabled) {

        // Do we need this??
        if (currentTimeUs - lastMessagesPreparedAt >= JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ) {
            updateJRDMSSTelemetryValues(&jrdmssTelemetryMessage); // jrdmssPrepareMessages();
            lastMessagesPreparedAt = currentTimeUs;
        }

        if (shouldCheckForJRDMSSRequest()) {
            jrdmssCheckSerialData(currentTimeUs);
        }

        if (jrdmssMsg) {

            if (jrdmssIsSending) {
                if (currentTimeUs - serialTimer < JRDMSS_TX_DELAY_US) {
                    return;
                }
            }
            sendJRDMSSTelemetryData();
            serialTimer = currentTimeUs;
        }
    }
}

#endif
