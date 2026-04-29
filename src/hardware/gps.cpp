#include <Arduino.h>
#include "gps.h"
#include "utils/log.h"
#include "utils/settings.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <core/defines.h>

SFE_UBLOX_GNSS myGNSS;

constexpr int GPS_RX_PIN = 0;
constexpr int GPS_TX_PIN = 1;
constexpr size_t test_bauds_len = GPS_BAUD_TEST_COUNT;
constexpr int test_bauds[test_bauds_len] = {460800, 38400, 115200, 230400};
constexpr int selected_baud = GPS_SELECTED_BAUD;

bool gpsConnected = false;
unsigned long gpsInitTime;

GPSStatusStruct currentGPSStatus;
static GPSStatusStruct gpsStatusOwned;

enum class GPSCommandType : uint8_t {
    START_SURVEY,
    STOP_SURVEY,
};

struct GPSCommand {
    GPSCommandType type;
    uint16_t observationTime;
    float requiredAccuracy;
};

static QueueHandle_t gps_command_queue = NULL;
static portMUX_TYPE gps_status_mux = portMUX_INITIALIZER_UNLOCKED;

GPSMode readGpsMode();
const char *gpsStatusString(const GPSStatusStruct &currentGPSStatus_);
void publishGPSStatus();
bool updateGPSStatus();
void processGPSCommands();
bool processGPSUart();
bool executeStartSurveyMode(uint16_t observationTime, float requiredAccuracy);
void executeStopSurveyMode();
bool saveSurveyPosition();

[[noreturn]] void gpsServiceTask(void *pvParameters);

bool prev_survey_in_active = false;
bool survey_save_armed = false;

// GPS service ownership:
//
// After startup configuration, gpsServiceTask is the sole runtime owner of
// myGNSS and Serial1. It continuously parses UART data, periodically publishes
// cached status, and executes queued control commands from the web/API layer.
//
// Other tasks read currentGPSStatus snapshots or enqueue commands. They do not
// call SparkFun GNSS methods directly, avoiding concurrent access to the
// non-thread-safe parser and UART.
bool configureGPS();

bool initializeGPS() {
    if (gps_command_queue == NULL) {
        gps_command_queue = xQueueCreate(GPS_COMMAND_QUEUE_LENGTH, sizeof(GPSCommand));
    }

    if (gps_command_queue == NULL) {
        error("GPS - Failed to create command queue.");
        return false;
    }

    bool resp = false;
    debug("Initializing GPS...");
    for (const int test_baud : test_bauds) {
        debugf("Testing baud rate: %d", test_baud);
        Serial1.end();
        Serial1.setRxBufferSize(1024 * 8);  // 8KB buffer to handle high baud rates
        Serial1.begin(test_baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        delay(1000);
        if ((resp = myGNSS.begin(Serial1, defaultMaxWait, false))) {
            break;
        }
    }

    if (!resp) {
        error("GPS - Not detected");
        gpsConnected = false;
        return false;
    }
    // Configure the GPS module
    gpsConnected = configureGPS();

    if (gpsConnected) {
        gpsInitTime = millis();
    }

    gpsStatusOwned.gpsConnected = gpsConnected;
    gpsStatusOwned.status_message = gpsConnected ? "Connected" : "Disconnected";
    gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);
    publishGPSStatus();

    // Start the single runtime owner for UART parsing, status, and GPS commands.
    xTaskCreate(gpsServiceTask, "gpsServiceTask", GPS_SERVICE_TASK_STACK, nullptr, GPS_SERVICE_TASK_PRIORITY, nullptr);
    return true;
}

bool configureGPS() {
    //myGNSS.enableDebugging(USBSerial);
    bool result = false;
    // UBX+RTCM3 is not a valid option so we enable all three.
    // myGNSS.setPortOutput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Set the UART port to
    // output UBX only (turn off NMEA and RTCM)

    myGNSS.setNavigationFrequency(1);  // Set output in Hz. RTCM rarely benefits from >1Hz.

    bool response = myGNSS.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA |
                                          COM_TYPE_RTCM3);  // Set the UART port to output RTCM3 and NMEA
    if (response == false) {
        error("GPS - Failed to set UART1 output.");
    }
    response = myGNSS.setUSBOutput(COM_TYPE_UBX | COM_TYPE_NMEA |
                                   COM_TYPE_RTCM3);  // Set the UART port to output UBX only (turn off NMEA and RTCM)
    if (response == false) {
        error("GPS - Failed to set USB output.");
    }

    // update uart1 baud rate
    debugf("Setting UART1 baud rate to %d", selected_baud);
    myGNSS.setSerialRate(selected_baud, COM_PORT_UART1);  // Set the UART port to fast baud rate
    Serial1.end();
    Serial1.setRxBufferSize(1024 * 8);  // 8KB buffer to handle high baud rates
    Serial1.begin(selected_baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Disable all NMEA sentences
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
    response &= myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
    if (response == false) {
        error("GPS - Failed to disable NMEA.");
    }

    // Enable necessary RTCM sentences
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_UART1, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_UART1, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1097, COM_PORT_UART1, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1127, COM_PORT_UART1, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 10);

    // Enable RTCM messages on USB
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_USB, 10);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_USB, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_USB, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1097, COM_PORT_USB, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1127, COM_PORT_USB, 1);
    response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_USB, 10);

    if (response == false) {
        error("GPS - Failed to enable RTCM.");
    }

    // automatic message reporting for gps status:
    response = true;

    response &= myGNSS.setAutoHPPOSLLH(true, false);
    response &= myGNSS.setAutoPVT(true, false);
    response &= myGNSS.setAutoNAVHPPOSECEF(true, false);
    response &= myGNSS.setAutoNAVSVIN(true, false);

    if (response == false) {
        error("GPS - Failed to set automatic messages.");
    }

    int64_t ecefX = settings["ecefX"].as<int64_t>();
    int64_t ecefY = settings["ecefY"].as<int64_t>();
    int64_t ecefZ = settings["ecefZ"].as<int64_t>();

    int32_t ecefX_cm = (ecefX / 100);
    int32_t ecefY_cm = (ecefY / 100);
    int32_t ecefZ_cm = (ecefZ / 100);
    int8_t ecefX_0_1mm = (ecefX % 100);
    int8_t ecefY_0_1mm = (ecefY % 100);
    int8_t ecefZ_0_1mm = (ecefZ % 100);

    debugf("ecefX: %d.%02dcm", ecefX_cm, ecefX_0_1mm);
    debugf("ecefY: %d.%02dcm", ecefY_cm, ecefY_0_1mm);
    debugf("ecefZ: %d.%02dcm", ecefZ_cm, ecefZ_0_1mm);

    if (ecefX == 0 && ecefY == 0 && ecefZ == 0) {
        info("GPS - Static position not set. Using Rover mode.");
        response = myGNSS.setSurveyMode(0, 0, 0);  // Disable survey mode
    } else {
        debugf("Setting static position to %d.%02d, %d.%02d, %d.%02d", 
               ecefX_cm, ecefX_0_1mm, ecefY_cm, ecefY_0_1mm, ecefZ_cm, ecefZ_0_1mm);
        response = myGNSS.setStaticPosition(ecefX_cm, ecefX_0_1mm, ecefY_cm, ecefY_0_1mm, ecefZ_cm, ecefZ_0_1mm, false);
    }

    if (response == false) {
        error("GPS - Failed to set GPS mode.");
    } else {
        info("GPS - Module configuration complete");
        result = true;
    }
    gpsStatusOwned.gpsMode = readGpsMode();

    return result;
}

String appendLeadingZero(int input) {
    if (input < 10) {
        return "0" + String(input);
    }
    return String(input);
}

bool executeStartSurveyMode(uint16_t observationTime, float requiredAccuracy) {
    if (!gpsConnected) {
        error("GPS - Not connected.");
        return false;
    }

    bool response = true;
    // response = myGNSS.setSurveyMode(0, 0, 0);  // Disable survey mode
    if (response == false) {
        error("GPS - Failed to stop Survey-in mode.");
        gpsStatusOwned.gpsMode = readGpsMode();
        return false;
    }

    infof("GPS - Starting Survey-in mode for %d seconds with accuracy %.2f meters...",
          observationTime, requiredAccuracy);

    // Set ZED-F9P to Survey-in mode
    response = myGNSS.setSurveyMode(1, observationTime, requiredAccuracy);
    // accuracy
    if (response == false) {
        error("GPS - Failed to set Survey-in mode.");
        gpsStatusOwned.gpsMode = readGpsMode();
        gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);
        publishGPSStatus();
        return false;
    }
    info("GPS - Survey-in mode started.");
    survey_save_armed = true;
    gpsStatusOwned.gpsMode = readGpsMode();
    gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);
    publishGPSStatus();
    return true;
}

void executeStopSurveyMode() {
    info("GPS - Stopping Survey-in mode...");

    // Set ZED-F9P to Survey-in mode
    bool resp = myGNSS.setSurveyMode(0, 0, 0);  // Minimum 600s (10 min) and 2.0m
    // accuracy

    if (resp == false) {
        error("GPS - Failed to stop Survey-in mode.");
    } else {
        info("GPS - Survey-in mode stopped.");
    }
    survey_save_armed = false;
    gpsStatusOwned.gpsMode = readGpsMode();
    gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);
    publishGPSStatus();
}

bool saveSurveyPosition() {
    if (myGNSS.getSurveyInValid()) {
        const int64_t x = static_cast<int64_t>(myGNSS.getHighResECEFX()) * 100 + myGNSS.getHighResECEFXHp();
        const int64_t y = static_cast<int64_t>(myGNSS.getHighResECEFY()) * 100 + myGNSS.getHighResECEFYHp();
        const int64_t z = static_cast<int64_t>(myGNSS.getHighResECEFZ()) * 100 + myGNSS.getHighResECEFZHp();
        writeSettings("ecefX", x);
        writeSettings("ecefY", y);
        writeSettings("ecefZ", z);
        settings["ecefX"] = x;
        settings["ecefY"] = y;
        settings["ecefZ"] = z;

        const int32_t x_cm = (x / 100);
        const int32_t y_cm = (y / 100);
        const int32_t z_cm = (z / 100);
        const int8_t x_0_1mm = (x % 100);
        const int8_t y_0_1mm = (y % 100);
        const int8_t z_0_1mm = (z % 100);

        bool response = myGNSS.setStaticPosition(x_cm, x_0_1mm, y_cm, y_0_1mm, z_cm, z_0_1mm, false);
        if (response == false) {
            error("GPS - Failed to set static position.");
        } else {
            info("GPS - Static position set.");
        }
        gpsStatusOwned.gpsMode = readGpsMode();
        gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);
        publishGPSStatus();
        return true;
    }
    return false;
}

const char *gpsStatusString(const GPSStatusStruct &currentGPSStatus_) {
    switch (currentGPSStatus_.gpsMode) {
        case GPSMode::ROVER:
            return "Rover mode";
        case GPSMode::SURVEY_IN:
            return "Survey-in mode";
        case GPSMode::FIXED:
            return "Fixed mode";
        case GPSMode::UNKNOWN:
        default:
            return "Unknown mode";
    }
}

void publishGPSStatus() {
    portENTER_CRITICAL(&gps_status_mux);
    currentGPSStatus = gpsStatusOwned;
    portEXIT_CRITICAL(&gps_status_mux);
}

GPSStatusStruct getGPSStatusSnapshot() {
    GPSStatusStruct snapshot;
    portENTER_CRITICAL(&gps_status_mux);
    snapshot = currentGPSStatus;
    portEXIT_CRITICAL(&gps_status_mux);
    return snapshot;
}

bool isSurveyInActive() {
    return getGPSStatusSnapshot().surveyInActive;
}

bool requestStartSurveyMode(uint16_t observationTime, float requiredAccuracy) {
    if (gps_command_queue == NULL) {
        error("GPS - Command queue not ready.");
        return false;
    }

    GPSCommand command = {GPSCommandType::START_SURVEY, observationTime, requiredAccuracy};
    if (xQueueSend(gps_command_queue, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
        error("GPS - Failed to queue survey start command.");
        return false;
    }

    infof("GPS - Queued Survey-in start for %d seconds with accuracy %.2f meters",
          observationTime, requiredAccuracy);
    return true;
}

bool requestStopSurveyMode() {
    if (gps_command_queue == NULL) {
        error("GPS - Command queue not ready.");
        return false;
    }

    GPSCommand command = {GPSCommandType::STOP_SURVEY, 0, 0.0f};
    if (xQueueSend(gps_command_queue, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
        error("GPS - Failed to queue survey stop command.");
        return false;
    }

    info("GPS - Queued Survey-in stop");
    return true;
}

bool updateGPSStatus() {
    // Read cached GNSS status. Automatic messages are configured with
    // implicitUpdate=false so these getters do not drain Serial1 here.
    gpsStatusOwned.gpsConnected = gpsConnected;
    gpsStatusOwned.status_message = gpsStatusOwned.gpsConnected ? "Connected" : "Disconnected";
    gpsStatusOwned.latitude = myGNSS.getHighResLatitude() / 10000000.0 + myGNSS.getHighResLatitudeHp() / 1000000000.0;
    gpsStatusOwned.longitude =
        myGNSS.getHighResLongitude() / 10000000.0 + myGNSS.getHighResLongitudeHp() / 1000000000.0;
    gpsStatusOwned.altitude = myGNSS.getAltitude() / 1000.0;
    gpsStatusOwned.x = myGNSS.getHighResECEFX() / 10.0 + myGNSS.getHighResECEFXHp() / 100.0;
    gpsStatusOwned.y = myGNSS.getHighResECEFY() / 10.0 + myGNSS.getHighResECEFYHp() / 100.0;
    gpsStatusOwned.z = myGNSS.getHighResECEFZ() / 10.0 + myGNSS.getHighResECEFZHp() / 100.0;

    gpsStatusOwned.surveyInActive = myGNSS.getSurveyInActive();
    gpsStatusOwned.surveyInValid = myGNSS.getSurveyInValid();
    gpsStatusOwned.surveyInObservationTime = myGNSS.getSurveyInObservationTime();
    gpsStatusOwned.surveyInMeanAccuracy = myGNSS.getSurveyInMeanAccuracy();
    gpsStatusOwned.satellites = myGNSS.getSIV();

    if (gpsStatusOwned.gpsMode == GPSMode::UNKNOWN) {
        gpsStatusOwned.gpsMode = readGpsMode();
    }
    gpsStatusOwned.gpsModeString = gpsStatusString(gpsStatusOwned);

    // If survey-in mode has just completed, save the position
    if (survey_save_armed && prev_survey_in_active && !gpsStatusOwned.surveyInActive) {
        info("GPS - Survey-in completed. Saving position...");
        saveSurveyPosition();
        survey_save_armed = false;
    }
    prev_survey_in_active = gpsStatusOwned.surveyInActive;
    publishGPSStatus();
    return true;
}

void processGPSCommands() {
    if (gps_command_queue == NULL) {
        return;
    }

    GPSCommand command;
    while (xQueueReceive(gps_command_queue, &command, 0) == pdTRUE) {
        switch (command.type) {
            case GPSCommandType::START_SURVEY:
                if (!executeStartSurveyMode(command.observationTime, command.requiredAccuracy)) {
                    error("GPS - Failed to start survey mode");
                }
                break;
            case GPSCommandType::STOP_SURVEY:
                executeStopSurveyMode();
                break;
        }
    }
}

bool processGPSUart() {
    static unsigned long lastBufferWarning = 0;
    static int maxBufferUsage = 0;
    const int BUFFER_SIZE = 1024 * 8;  // 8KB
    const int WARNING_THRESHOLD = (BUFFER_SIZE * 75) / 100;  // 75% full

    // Check buffer usage before processing
    int available = Serial1.available();

    // Track maximum buffer usage
    if (available > maxBufferUsage) {
        maxBufferUsage = available;
        debugf("GPS UART buffer peak usage: %d/%d bytes (%.1f%%)",
               maxBufferUsage, BUFFER_SIZE, (maxBufferUsage * 100.0f) / BUFFER_SIZE);
    }

    // Warn if buffer is getting full (rate-limited to once per 5 seconds)
    if (available > WARNING_THRESHOLD) {
        unsigned long now = millis();
        if ((unsigned long)(now - lastBufferWarning) > 5000) {
            lastBufferWarning = now;
            warningf("GPS UART buffer near overflow: %d/%d bytes (%.1f%% full)",
                    available, BUFFER_SIZE, (available * 100.0f) / BUFFER_SIZE);
        }
    }

    myGNSS.checkUblox();
    return Serial1.available() > 0;
}

[[noreturn]] void gpsServiceTask(void *pvParameters){
    unsigned long lastStatusUpdate = 0;

    for (;;) {
        processGPSCommands();
        bool hasDataAfterCheck = processGPSUart();

        const unsigned long now = millis();
        if ((unsigned long)(now - lastStatusUpdate) >= 1000) {
            lastStatusUpdate = now;
            updateGPSStatus();
        }

        // Always use a minimal delay to allow lower-priority tasks (like loopTask) to run
        // and feed the watchdog. Even 1 tick (~1ms) is enough to prevent starvation.
        // At 460800 baud, ~57 bytes arrive per 1ms, but 8KB buffer provides plenty of margin.
        if (!hasDataAfterCheck) {
            vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay when buffer empty
        } else {
            // Buffer has data: process multiple iterations quickly, then brief delay
            // This batches processing while still allowing watchdog to be fed
            vTaskDelay(1);  // Minimum possible delay (1 FreeRTOS tick)
        }
    }
}

GPSMode readGpsMode() {
    bool response = false;
    UBX_CFG_TMODE3_data_t tmode3_data;
    //auto start_time = millis();
    const int maxRetries = 5;
    const int retryTimeout = 1000; // 1 second timeout

    for (int retry = 0; retry < maxRetries; retry++) {
        response = myGNSS.getSurveyMode(&tmode3_data, retryTimeout);
        if (response) {
            break; // Success, exit the retry loop
        }
    }

    if (!response) {
        error("GPS - Failed to get Survey-in mode.");
        return GPSMode::UNKNOWN;
    }
    //debugf("Getting Survey-in mode took %d ms", millis() - start_time);
    return static_cast<GPSMode>(tmode3_data.flags.bits.mode);
}
