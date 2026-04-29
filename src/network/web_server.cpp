#include <Arduino.h>
#include "core/defines.h"

#include "hardware/gps.h"
#include "utils/settings.h"
#include <http_parser.h>
#include "ntrip.h"
#include "ethernet.h"
#include "web_server.h"
#include <Update.h>
#include "utils/system_status.h"
#include "utils/log.h"

// HTTP Related
WebServer server(80);

static void notFound()
{
    debugf("Not found: %s", server.uri().c_str());
    server.send(404, "text/plain", "Not found");
}

// Add these handler functions before initializeWebServer()
void handleUpdateRequest() {
    server.send(200, "text/plain", "Ready for OTA update");
}

// Track OTA upload state to prevent processing after error
static bool otaUploadFailed = false;

void handleFileUpload() {
    HTTPUpload& upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        // Start update
        debugf("Update Start: %s", upload.filename.c_str());
        info("OTA: Starting update");
        otaUploadFailed = false;

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            error("OTA: Failed to begin update");
            Update.printError(USBSerial);
            otaUploadFailed = true;
            return;
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        // Skip write if previous operation failed
        if (otaUploadFailed) {
            return;
        }

        // Write update chunk
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            error("OTA: Write failed");
            Update.printError(USBSerial);
            Update.abort();  // Abort the update to prevent partial writes
            otaUploadFailed = true;
            return;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        // Check if upload had errors
        if (otaUploadFailed) {
            error("OTA: Upload failed - aborting");
            Update.abort();
            return;
        }

        if (!Update.end(true)) {
            error("OTA: Failed to finalize update");
            Update.printError(USBSerial);
            Update.abort();
            return;
        }
        info("OTA: Update successful");
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        error("OTA: Upload aborted");
        Update.abort();
        otaUploadFailed = true;
    }
}

// Update the POST handler to check for errors
void handleUpdateComplete() {
    if (otaUploadFailed) {
        server.send(500, "text/plain", "Update failed. Check logs for details.");
    } else {
        server.send(200, "text/plain", "Update successful. Rebooting...");
        delay(500);
        ESP.restart();
    }
}

// Add this helper function to calculate uptime string from milliseconds
String calculateUptime(unsigned long milliseconds) {
    unsigned long seconds = milliseconds / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;

    hours %= 24;
    minutes %= 60;
    seconds %= 60;

    char uptimeStr[50];
    if (days > 0) {
        snprintf(uptimeStr, sizeof(uptimeStr), "%lud %02luh %02lum", days, hours, minutes);
    } else if (hours > 0) {
        snprintf(uptimeStr, sizeof(uptimeStr), "%luh %02lum", hours, minutes);
    } else {
        snprintf(uptimeStr, sizeof(uptimeStr), "%lum %02lus", minutes, seconds);
    }
    return String(uptimeStr);
}

[[noreturn]] void WebServerTask(void *pvParameters)
{
    for (;;)
    {
        server.handleClient();
        delay(1);
    }
}

// Start web server
void initializeWebServer()
{
    debug("Starting webserver");
    server.begin();

    for (const auto & webpage : webpages)
    {
        server.on(webpage.filename, HTTP_GET, [&webpage]()
                  { server.send(200, webpage.content_type, (const char *)webpage.data); });
    }

    server.on("/restart", HTTP_GET, []()
              {
                server.send(200, "text/html", "message");
                info("Rebooting...");
                ESP.restart(); });

    server.on("/getSettings", HTTP_GET, []()
              {
                  String message;
                  serializeJson(settings, message);
                  server.send(200, "text/plain", message); });
    server.on("/log", HTTP_GET, []()
              {
                  // Get log string that already includes timestamp
                  String logJson = getLog();
                  server.send(200, "application/json", logJson);
              });
    // Start Survey-in mode
    server.on("/startSurvey", HTTP_GET, []() {
        uint16_t observationTime = 0;
        float requiredAccuracy = 0.0f;

        // Check if time parameter is provided
        if (server.hasArg("time")) {
            auto time = server.arg("time");
            observationTime = time.toInt();
        }

        // Check if accuracy parameter is provided
        if (server.hasArg("accuracy")) {
            auto accuracy = server.arg("accuracy");
            requiredAccuracy = accuracy.toFloat();
        }

        infof("Survey-in parameters set: %d seconds, %.2f meters",
              observationTime,
              requiredAccuracy);
        if (requestStartSurveyMode(observationTime, requiredAccuracy)) {
            server.send(200, "text/plain", "Survey start queued");
        } else {
            server.send(503, "text/plain", "Failed to queue survey start");
        }
    });

    server.on("/status", HTTP_GET, []()
              {
                  String message;
                  StaticJsonDocument<512> status;

                  // Add version information
                  status["firmwareVersion"] = FIRMWARE_VERSION;
                  status["buildDate"]       = BUILD_DATE;
                  status["uptime"]          = getUptimeString();
                  status["timestamp"]       = millis();

                  // Add NTRIP connection status
                  status["enableCaster1"] = settings["enableCaster1"].as<bool>();
                  status["enableCaster2"] = settings["enableCaster2"].as<bool>();
                  status["ntripVersion1"] = settings["ntripVersion1"].as<int>();
                  status["ntripVersion2"] = settings["ntripVersion2"].as<int>();

                  // Use connection opened times for status
                  status["ntripConnected1"] = NtripPrimaryStatus.connected;
                  status["ntripConnected2"] = NtripSecondaryStatus.connected;

                  // Calculate uptimes if connected
                  unsigned long currentMillis = millis();

                  if (NtripPrimaryStatus.connectionOpenedAt > 0) {
                      status["ntripUptime1"] = calculateUptime(currentMillis - NtripPrimaryStatus.connectionOpenedAt);
                  }

                  if (NtripSecondaryStatus.connectionOpenedAt > 0) {
                      status["ntripUptime2"] = calculateUptime(currentMillis - NtripSecondaryStatus.connectionOpenedAt);
                  }

                  GPSStatusStruct gpsStatus = getGPSStatusSnapshot();

                  // Rest of the status fields...
                  status["gpsStatusString"] = gpsStatus.status_message;
                  status["gpsLatitude"] = serialized(String(gpsStatus.latitude, 9));
                  status["gpsLongitude"] = serialized(String(gpsStatus.longitude, 9));
                  status["gpsAltitude"] = gpsStatus.altitude;
                  status["gpsSiv"] = gpsStatus.satellites;
                  status["gpsConnected"] = gpsStatus.gpsConnected;
                  status["surveyInActive"] = gpsStatus.surveyInActive;
                  status["surveyInObservationTime"] = gpsStatus.surveyInObservationTime;
                  status["surveyInValid"] = gpsStatus.surveyInValid;
                  status["surveyInMeanAccuracy"] = gpsStatus.surveyInMeanAccuracy;
                  status["gpsCurrentTime"] = gpsStatus.gpsCurrentTime;
                  status["x"] = gpsStatus.x;
                  status["y"] = gpsStatus.y;
                  status["z"] = gpsStatus.z;
                  status["gpsMode"] = gpsStatus.gpsModeString;

                  serializeJson(status, message);
                  server.send(200, "application/json", message);
              });

    // SECURITY: Removed GET handler for applySettings to prevent credentials in query parameters
    // Only POST is allowed to protect sensitive data (passwords) from being logged

    // POST handler for applySettings
    server.on("/applySettings", HTTP_POST, []() {
      info("Applying settings via POST");
      String inputMessage;
      
      // Handle POST data
      if (server.hasArg("plain")) {
        // Handle raw POST data
        inputMessage = server.arg("plain");
        debugf("Raw POST data: %s", inputMessage.c_str());
      } else {
        // Handle form data
        size_t param_count = server.args();
        for (int i = 0; i < param_count; i++) {
          auto name = server.argName(i);
          auto value = server.arg(i);
          debugf("Param %s: %s", name.c_str(), value.c_str());
          writeSettings(name, value);
        }
      }

      delay(100);
      server.send(200, "text/plain", "Settings applied");
      ESP.restart();
    });

    server.on("/stopSurvey", HTTP_GET, []() {
        if (requestStopSurveyMode()) {
            server.send(200, "text/plain", "Survey stop queued");
        } else {
            server.send(503, "text/plain", "Failed to queue survey stop");
        }
    });

    server.on("/update", HTTP_GET, handleUpdateRequest);
    server.on("/update", HTTP_POST, handleUpdateComplete, handleFileUpload);

    server.onNotFound(notFound);

    xTaskCreate(WebServerTask, "WebServerTask", WEB_SERVER_TASK_STACK, NULL, WEB_SERVER_TASK_PRIORITY, NULL);
}
