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
  * Without their skills in reverse engineering the JR telelmetry procol, this would not be possible
  *
--  * https://github.com/obayer/MultiWii-HoTT
--  * https://github.com/oBayer/MultiHoTT-Module
--  * https://code.google.com/p/hott-for-ardupilot
  *
  * JR DMSS telemetry is implemented in JR equipment using a bi-directional protocol over a single wire.
  *
  * Generally the receiver sends a single request byte out using normal uart signals, then waits a short period for a
  * multiple byte response and checksum byte before it sends out the next request byte. Each request from the receiver
  * is the ID of the sensor that it wan't the information for
  * Each response byte must be send with a protocol specific delay between them.
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
#include "rx/xbus.h"            // Need this for the CRC8 calculation
#include "telemetry/telemetry.h"

  //#define JRDMSS_DEBUG

#define JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ ((1000 * 1000) / 5)
#define JRDMSS_RX_SCHEDULE 4000
#define JRDMSS_TX_DELAY_US 3000
#define MILLISECONDS_IN_A_SECOND 1000

static uint32_t rxSchedule = JRDMSS_RX_SCHEDULE;
static uint32_t txDelayUs = JRDMSS_TX_DELAY_US;

static uint32_t lastJRDMSSRequestCheckAt = 0;
static uint32_t lastMessagesPreparedAt = 0;

static bool jrdmssIsSending = false;

static uint8_t* jrdmssMsg = NULL;
static uint8_t jrdmssMsgRemainingBytesToSendCount;
static uint8_t jrdmssMsgCrc;

#define JRDMSS_BAUDRATE 250000
#define JRDMSS_PORT_MODE MODE_RXTX // must be opened in RXTX so that TX and RX pins are allocated.

static serialPort_t* jrdmssPort = NULL;
static const serialPortConfig_t* portConfig;

static bool jrdmssTelemetryEnabled = false;
static portSharing_e jrdmssPortSharing;

static jrdmssTelemetry_t jrdmssTelemetryMessage;

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
static void initialiseJRDMSSMessage(jrdmssTelemetryMessage_t* msg, size_t size)
{
    memset(msg, 0, size);
    msg->sensor = 0x00;
    msg->constant = JR_TELEMTRY_SENSOR_MSG_CONSTANT;
    msg->sensorindex = 0x00;
    msg->sensorvauelhigh = 0x00;
    msg->sensorvaluelow = 0x0;
    msg->crc = 0x0;
}

static void initialiseMessages(void)
{
    initialiseJRDMSSMessage(&jrdmssTelemetryMessage, sizeof(jrdmssTelemetryMessage));
}

static inline void jrdmssUpdateBattery(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t volt = getLegacyBatteryVoltage();

    msg->sensor = JR_TELEMETRY_ESC_SENSOR_ID;
    msg->constant = JR_TELEMTRY_SENSOR_MSG_CONSTANT;
    msg->sensorindex = JR_TELEMETRY_ESC_FBATVOLTAGE_SENSOR_ID;
    msg->sensorvauelhigh = volt >> 8;
    msg->sensorvaluelow = volt & 0xFF;

    //uint8_t crc = crc8((uint8_t*)msg, 5);
    msg->crc = crc8((uint8_t*)msg, 5);

    updateAlarmBatteryStatus(hottEAMMessage);
}

static inline void jrdmssUpdateCurrentMeter(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t amp = getLegacyBatteryCurrent();
    msg->current_L = amp & 0xFF;
    msg->current_H = amp >> 8;
}

static inline void jrdmssUpdateBatteryDrawnCapacity(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t mAh = getBatteryCapacityUsed() / 10;
    msg->batt_cap_L = mAh & 0xFF;
    msg->batt_cap_H = mAh >> 8;
}

static inline void jrdmssUpdateAltitude(jrdmssTelemetryMessage_t* msg)
{
    const uint16_t hottEamAltitude = (getEstimatedAltitudeCm() / 100) + JRDMSS_EAM_OFFSET_HEIGHT;

    msg->altitude_L = hottEamAltitude & 0x00FF;
    msg->altitude_H = hottEamAltitude >> 8;
}

#ifdef USE_VARIO
static inline void jrdmssUpdateClimbrate(jrdmssTelemetryMessage_t* msg)
{
    const int32_t vario = getEstimatedVarioCms();
    msg->climbrate_L = (30000 + vario) & 0x00FF;
    msg->climbrate_H = (30000 + vario) >> 8;
    msg->climbrate3s = 120 + (vario / 100);
}
#endif

void PrepareJRDMSSResponse(jrdmssTelemetryMessage_t* msg)
{
    jrdmssUpdateBattery(msg);
    jrdmssUpdateCurrentMeter(msg);
    jrdmssUpdateBatteryDrawnCapacity(msg);
    jrdmssUpdateAltitude(msg);
#ifdef USE_VARIO
    jrdmssUpdateClimbrate(msg);
#endif
}

static void jrdmssSerialWrite(uint8_t c)
{
    static uint8_t serialWrites = 0;
    serialWrites++;
    serialWrite(jrdmssPort, c);
}

void freeJRdmssTelemetryPort(void)
{
    closeSerialPort(jrdmssPort);
    jrdmssPort = NULL;
    jrdmssTelemetryEnabled = false;
}

void initJRdmssTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_JRDMSS);

    if (!portConfig) {
        return;
    }

    jrdmssPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_JRDMSS);

    initialiseMessages();
}

static void flushJRdmssRxBuffer(void)
{
    while (serialRxBytesWaiting(jrdmssPort) > 0) {
        serialRead(jrdmssPort);
    }
}

static void workAroundForJRDMSSTelemetryOnUsart(serialPort_t* instance, portMode_e mode)
{
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
    if (jrdmssIsUsingHardwareUART()) {
        workAroundForJRDMSSTelemetryOnUsart(jrdmssPort, MODE_TX);
    }
    else {
        serialSetMode(jrdmssPort, MODE_TX);
    }
    jrdmssIsSending = true;
    jrdmssMsgCrc = 0;
}

static void jrdmssConfigurePortForRX(void)
{
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if (jrdmssIsUsingHardwareUART()) {
        workAroundForJRDMSSTelemetryOnUsart(jrdmssPort, MODE_RX);
    }
    else {
        serialSetMode(jrdmssPort, MODE_RX);
    }
    jrdmssMsg = NULL;
    jrdmssIsSending = false;
    flushJRdmssRxBuffer();
}

void configureJRdmssTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    portOptions_e portOptions = SERIAL_NOT_INVERTED |
        (telemetryConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR) |
        (telemetryConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

    jrdmssPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_JRDMSS, NULL, NULL, JRDMSS_BAUDRATE, JRDMSS_PORT_MODE, portOptions);

    if (!jrdmssPort) {
        return;
    }

    jrdmssConfigurePortForRX();

    jrdmssTelemetryEnabled = true;
}

static void jrdmssSendResponse(uint8_t* buffer, int length)
{
    if (jrdmssIsSending) {
        return;
    }

    jrdmssMsg = buffer;
    jrdmssMsgRemainingBytesToSendCount = length + JRDMSS_CRC_SIZE;
}

static inline void jrdmssSendResponse(void)
{
    jrdmssSendResponse((uint8_t*)&jrdmssTelemetryMessage, sizeof(jrdmssTelemetryMessage));
}

static void jrdmssPrepareMessages(void) {
    PrepareJRDMSSResponse(&jrdmssTelemetryMessage);
}

static void processBinaryModeRequest(uint8_t address)
{
#ifdef JRDMSS_DEBUG
    static uint8_t jrdmssBinaryRequests = 0;
    static uint8_t jrdmssRequests = 0;
#endif

    switch (address) {
    case 0x8E:
#ifdef JRDMSS_DEBUG
        jrdmssRequests++;
#endif
        jrdmssSendResponse();
        break;
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

    const uint8_t bytesWaiting = serialRxBytesWaiting(jrdmssPort);

    if (bytesWaiting <= 1) {
        return;
    }

    if (bytesWaiting != 2) {
        flushJRdmssRxBuffer();
        lookingForRequest = true;
        return;
    }

    if (lookingForRequest) {
        lastJRDMSSRequestCheckAt = currentMicros;
        lookingForRequest = false;
        return;
    }
    else {
        bool enoughTimePassed = currentMicros - lastJRDMSSRequestCheckAt >= rxSchedule;

        if (!enoughTimePassed) {
            return;
        }
        lookingForRequest = true;
    }

    const uint8_t requestId = serialRead(jrdmssPort);
    const uint8_t address = serialRead(jrdmssPort);

    if ((requestId == 0) || (requestId == JRDMSS_BINARY_MODE_REQUEST_ID) || (address == JRDMSS_TELEMETRY_NO_SENSOR_ID)) {
        /*
         * FIXME the first byte of the HoTT request frame is ONLY either 0x80 (binary mode) or 0x7F (text mode).
         * The binary mode is read as 0x00 (error reading the upper bit) while the text mode is correctly decoded.
         * The (requestId == 0) test is a workaround for detecting the binary mode with no ambiguity as there is only
         * one other valid value (0x7F) for text mode.
         * The error reading for the upper bit should nevertheless be fixed
         */
        processBinaryModeRequest(address);
    }
}

/*
 * TODO: Complete for JR telemetry
 */
static void sendJRdmssTelemetryData(void) {

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

static inline bool shouldPrepareJRdmssMessages(uint32_t currentMicros)
{
    return currentMicros - lastMessagesPreparedAt >= JRDMSS_MESSAGE_PREPARATION_FREQUENCY_5_HZ;
}

static inline bool shouldCheckForJRdmssRequest(void)
{
    if (jrdmssIsSending) {
        return false;
    }
    return true;
}

void checkJRdmssTelemetryState(void)
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

void handleJRdmssTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t serialTimer;

    if (!jrdmssTelemetryEnabled) {
        return;
    }

    if (shouldPrepareJRdmssMessages(currentTimeUs)) {
        jrdmssPrepareMessages();
        lastMessagesPreparedAt = currentTimeUs;
    }

    if (shouldCheckForJRdmssRequest()) {
        jrdmssCheckSerialData(currentTimeUs);
    }

    if (!jrdmssMsg)
        return;

    if (jrdmssIsSending) {
        if (currentTimeUs - serialTimer < txDelayUs) {
            return;
        }
    }
    sendJRdmssTelemetryData();
    serialTimer = currentTimeUs;
}

#endif
