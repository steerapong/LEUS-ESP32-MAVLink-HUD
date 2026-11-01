#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_GFX_Library.h>
#include <common/mavlink.h>
#include <math.h>
#define LCD_BL 2
constexpr uint16_t UDP_PORT = 14550;
constexpr uint32_t HUD_INTERVAL_MS = 100;
constexpr uint32_t STATUS_INTERVAL_MS = 200;
constexpr uint32_t ATT_CAUTION_MS = 300, ATT_STALE_MS = 1000;
constexpr uint32_t POS_CAUTION_MS = 1000, POS_STALE_MS = 3000;
constexpr uint32_t VFR_CAUTION_MS = 1000, VFR_STALE_MS = 3000;
constexpr uint32_t GPS_CAUTION_MS = 2000, GPS_STALE_MS = 5000;
constexpr uint32_t BAT_CAUTION_MS = 3000, BAT_STALE_MS = 6000;
constexpr uint32_t LINK_CAUTION_MS = 3000, LINK_LOST_MS = 6000;
constexpr uint32_t RECOVERY_HOLD_MS = 600;
constexpr uint16_t BG       = 0x1083;
constexpr uint16_t PANEL    = 0x18C6;
constexpr uint16_t BAR      = 0x2169;
constexpr uint16_t BORDER   = 0x426D;
constexpr uint16_t WHITE_C  = 0xFFFF;
constexpr uint16_t CYAN_C   = 0x5E9F;
constexpr uint16_t YELLOW_C = 0xFD86;
constexpr uint16_t GREEN_C  = 0x4F36;
constexpr uint16_t RED_C    = 0xF986;
constexpr uint16_t MUTED    = 0xA534;
constexpr uint16_t STATUS_GREEN = 0x2468;
constexpr uint16_t STATUS_AMBER = 0x9A80;
constexpr uint16_t STATUS_RED   = 0xA145;
constexpr uint16_t STATUS_GRAY  = 0x4A69;
constexpr int BAT_CAUTION_PCT = 30;
constexpr int BAT_WARNING_PCT = 15;
constexpr int CX = 400;
constexpr int CY = 265;
constexpr int HORIZON_HALF = 135;
constexpr int ROLL_RADIUS = 145;
constexpr int HUD_LEFT   = 250;
constexpr int HUD_RIGHT  = 550;
constexpr int HUD_TOP    = 110;
constexpr int HUD_BOTTOM = 395;
constexpr float TAKEOFF_REL_ALT_M = 2.0f;
constexpr float LANDING_REL_ALT_M = 0.8f;
constexpr uint32_t FLIGHT_CONFIRM_MS = 1200;
constexpr uint32_t EXT_STATE_FRESH_MS = 3000;
enum StreamId : uint8_t { S_ATT, S_POS, S_VFR, S_GPS, S_BAT, S_HB, S_COUNT };
struct StreamStats {
    const char* name;
    uint32_t lastMs;
    uint32_t received;
    uint32_t intervalMaxMs;
};

extern Arduino_ESP32RGBPanel* rgbpanel;
extern Arduino_RGB_Display* gfx;
extern WiFiUDP udp;
extern mavlink_message_t mavMsg;
extern mavlink_status_t mavStatus;
extern float rollDeg, pitchDeg, yawDeg;
extern uint32_t flightMode;
extern bool armed;
extern float batteryVoltage, batteryCurrent;
extern uint32_t heartbeatCount, attitudeCount, sysStatusCount, totalMessages, totalPackets;
extern uint32_t lastHudUpdate, lastStatusUpdate, lastAttitudeTime, lastVfrTime, lastPositionTime;
extern uint32_t statusEvaluationCount, hudFrameCount, lastSerialReport, lastHeartbeatTime;
extern bool hudDrawn;
extern float oldRoll, oldPitch, oldYaw;
extern bool oldArmed;
extern uint32_t oldMode;
extern bool oldConnected;
extern float airSpeed;
extern uint16_t missionCurrentSeq;
extern bool missionCurrentValid;
extern float targetDistanceM;
extern uint32_t lastNavControllerTime;
extern uint16_t gpsHdopCenti;
extern uint8_t gpsSatellites;
extern uint32_t airborneStartMs, lastAirborneDurationMs;
extern bool airborne, airborneTimeStarted;
extern uint32_t lastExtendedStateTime, takeoffCandidateMs, landingCandidateMs;
extern bool flightTimeEstimated;
extern float groundSpeed, verticalSpeed, relativeAltitude;
extern uint8_t gpsFixType;
extern int8_t batteryRemaining;
extern uint32_t lastGpsTime, lastBatteryTime;
extern const char* WIFI_SSID;
extern const char* WIFI_PASS;
extern StreamStats streams[S_COUNT];
void noteStream(StreamId id, uint32_t now);
uint32_t ageMs(uint32_t last, uint32_t now);
