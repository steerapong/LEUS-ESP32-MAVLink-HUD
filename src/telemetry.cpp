#include "telemetry.h"
// =====================================================
// LEUS HUD - v0.19 Step 3 / Per-Message Timing Policy
// ESP32-S3 / JC8048W550 / 800x480
// =====================================================

// WiFi - ใช้ค่าจาก Test 15 เดิม
const char* WIFI_SSID = "steevapong 2.4G";
const char* WIFI_PASS = "Teera2512";

// Trial thresholds: adjust after observing actual message intervals.
// Trial thresholds per MAVLink message; tune from measured arrival rates.

// LCD
#define LCD_BL 2

Arduino_ESP32RGBPanel* rgbpanel =
    new Arduino_ESP32RGBPanel(
        40, 41, 39, 42,
        45, 48, 47, 21, 14,
        5, 6, 7, 15, 16, 4,
        8, 3, 46, 9, 1,
        0, 8, 4, 8,
        0, 8, 4, 8,
        1, 16000000
    );

Arduino_RGB_Display* gfx =
    new Arduino_RGB_Display(
        800, 480, rgbpanel, 0, true
    );

WiFiUDP udp;
mavlink_message_t mavMsg;
mavlink_status_t mavStatus = {};

// Colors RGB565
// Status background colors (RGB565)
// Demonstration thresholds based on reported remaining percentage;
// configure for each aircraft, not a flight-readiness determination.



// Layout coordinates ประมาณในเบื้องต้น

// HUD rectangle

// Telemetry
float rollDeg = 0;
float pitchDeg = 0;
float yawDeg = 0;

uint32_t flightMode = 0;
bool armed = false;

float batteryVoltage = -1;
float batteryCurrent = -1;

uint32_t heartbeatCount = 0;
uint32_t attitudeCount = 0;
uint32_t sysStatusCount = 0;
uint32_t totalMessages = 0;
uint32_t totalPackets = 0;

uint32_t lastHudUpdate = 0;
uint32_t lastStatusUpdate = 0;
uint32_t lastAttitudeTime = 0;
uint32_t lastVfrTime = 0;
uint32_t lastPositionTime = 0;
uint32_t statusEvaluationCount = 0;
uint32_t hudFrameCount = 0;
uint32_t lastSerialReport = 0;
uint32_t lastHeartbeatTime = 0;

bool hudDrawn = false;
float oldRoll = 0;
float oldPitch = 0;
float oldYaw = 0;
bool oldArmed = false;
uint32_t oldMode = UINT32_MAX;
bool oldConnected = false;

float airSpeed = -1.0f;
uint16_t missionCurrentSeq = 0;
bool missionCurrentValid = false;
float targetDistanceM = NAN;
uint32_t lastNavControllerTime = 0;
uint16_t gpsHdopCenti = UINT16_MAX;
uint8_t gpsSatellites = 255;
// Flight time prefers EXTENDED_SYS_STATE; fallback estimates takeoff from armed + relative altitude.
uint32_t airborneStartMs = 0;
uint32_t lastAirborneDurationMs = 0;
bool airborne = false;
bool airborneTimeStarted = false;
uint32_t lastExtendedStateTime = 0;
uint32_t takeoffCandidateMs = 0;
uint32_t landingCandidateMs = 0;
bool flightTimeEstimated = false;
float groundSpeed = -1.0f;
float verticalSpeed = NAN; // VFR_HUD.climb: positive = climb, negative = descent
float relativeAltitude = NAN;
uint8_t gpsFixType = 0;
int8_t batteryRemaining = -1;
uint32_t lastGpsTime = 0;
uint32_t lastBatteryTime = 0;
// Each stream tracks actual received rate and inter-arrival jitter.
StreamStats streams[S_COUNT] = {
    {"ATT", 0, 0, 0}, {"POS", 0, 0, 0}, {"VFR", 0, 0, 0},
    {"GPS", 0, 0, 0}, {"BAT", 0, 0, 0}, {"HB", 0, 0, 0}
};
void noteStream(StreamId id, uint32_t now) {
    StreamStats& s = streams[id];
    if (s.lastMs != 0) {
        uint32_t delta = now - s.lastMs;
        if (delta > s.intervalMaxMs) s.intervalMaxMs = delta;
    }
    s.lastMs = now;
    ++s.received;
}
uint32_t ageMs(uint32_t last, uint32_t now) {
    return last ? (uint32_t)(now - last) : UINT32_MAX;
}

// EKF is not evaluated yet; show NO DATA rather than an invented healthy state.


