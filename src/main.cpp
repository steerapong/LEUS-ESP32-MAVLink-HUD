
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_GFX_Library.h>
#include <common/mavlink.h>
#include <math.h>

// =====================================================
// LEUS HUD - v0.19 Step 3 / Per-Message Timing Policy
// ESP32-S3 / JC8048W550 / 800x480
// =====================================================

// WiFi - ใช้ค่าจาก Test 15 เดิม
const char* WIFI_SSID = "xxxxx";
const char* WIFI_PASS = "xxxxxxx";

constexpr uint16_t UDP_PORT = 14550;
constexpr uint32_t HUD_INTERVAL_MS = 100;
constexpr uint32_t STATUS_INTERVAL_MS = 200;
// Trial thresholds: adjust after observing actual message intervals.
// Trial thresholds per MAVLink message; tune from measured arrival rates.
constexpr uint32_t ATT_CAUTION_MS = 300, ATT_STALE_MS = 1000;
constexpr uint32_t POS_CAUTION_MS = 1000, POS_STALE_MS = 3000;
constexpr uint32_t VFR_CAUTION_MS = 1000, VFR_STALE_MS = 3000;
constexpr uint32_t GPS_CAUTION_MS = 2000, GPS_STALE_MS = 5000;
constexpr uint32_t BAT_CAUTION_MS = 3000, BAT_STALE_MS = 6000;
constexpr uint32_t LINK_CAUTION_MS = 3000, LINK_LOST_MS = 6000;
constexpr uint32_t RECOVERY_HOLD_MS = 600;

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
// Status background colors (RGB565)
constexpr uint16_t STATUS_GREEN = 0x2468;
constexpr uint16_t STATUS_AMBER = 0x9A80;
constexpr uint16_t STATUS_RED   = 0xA145;
constexpr uint16_t STATUS_GRAY  = 0x4A69;
// Demonstration thresholds based on reported remaining percentage;
// configure for each aircraft, not a flight-readiness determination.
constexpr int BAT_CAUTION_PCT = 30;
constexpr int BAT_WARNING_PCT = 15;



// Layout coordinates ประมาณในเบื้องต้น
constexpr int CX = 400;
constexpr int CY = 265;
constexpr int HORIZON_HALF = 135;
constexpr int ROLL_RADIUS = 145;

// HUD rectangle
constexpr int HUD_LEFT   = 250;
constexpr int HUD_RIGHT  = 550;
constexpr int HUD_TOP    = 110;
constexpr int HUD_BOTTOM = 395;

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
constexpr float TAKEOFF_REL_ALT_M = 2.0f;
constexpr float LANDING_REL_ALT_M = 0.8f;
constexpr uint32_t FLIGHT_CONFIRM_MS = 1200;
constexpr uint32_t EXT_STATE_FRESH_MS = 3000;
float groundSpeed = -1.0f;
float verticalSpeed = NAN; // VFR_HUD.climb: positive = climb, negative = descent
float relativeAltitude = NAN;
uint8_t gpsFixType = 0;
int8_t batteryRemaining = -1;
uint32_t lastGpsTime = 0;
uint32_t lastBatteryTime = 0;
// Each stream tracks actual received rate and inter-arrival jitter.
enum StreamId : uint8_t { S_ATT, S_POS, S_VFR, S_GPS, S_BAT, S_HB, S_COUNT };
struct StreamStats {
    const char* name;
    uint32_t lastMs;
    uint32_t received;
    uint32_t intervalMaxMs;
};
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


// =====================================================
// Utility
// =====================================================

float normalizeYaw(float yaw)
{
    float result = fmodf(yaw, 360.0f);
    if (result < 0) result += 360.0f;
    return result;
}

const char* getFlightMode(uint32_t mode)
{
    // ArduCopter โหมดที่ตาดว่ามี ในระบบ
    switch (mode) {
        case 0:  return "STABILIZE";
        case 1:  return "ACRO";
        case 2:  return "ALT HOLD";
        case 3:  return "AUTO";
        case 4:  return "GUIDED";
        case 5:  return "LOITER";
        case 6:  return "RTL";
        case 9:  return "LAND";
        case 16: return "POSHOLD";
        case 17: return "BRAKE";
        default: return "OTHER";
    }
}

// =====================================================
// Static UI
// =====================================================
void drawFlightInfoPanel();
void updateFlightInfo();

void drawStaticUI()
{
    gfx->fillScreen(BG);

    // Top bar
    // LEUS Header - Red Background / White Text เลือกเอาตามใจ
    constexpr uint16_t LEUS_RED = 0xD104;

    gfx->fillRoundRect(12, 8, 185, 42, 6, LEUS_RED);

    gfx->setTextColor(WHITE_C, LEUS_RED);
    gfx->setTextSize(3);
    gfx->setCursor(22, 17);
    gfx->print("LEUS HUD");

    // Main panel
    gfx->fillRoundRect(10, 68, 780, 350, 8, PANEL);
    gfx->drawRoundRect(10, 68, 780, 350, 8, BORDER);

    // Navigation side panels are drawn by drawNavigationPanels().

    // Bottom bar
    gfx->fillRect(0, 424, 800, 56, BAR);

    // Roll scale - fixed
    const int marks[] = {
        -60, -45, -30, -20, -10,
        0, 10, 20, 30, 45, 60
    };

    gfx->setTextSize(1);
    gfx->setTextColor(WHITE_C);

    for (int i = 0; i < 11; i++) {
        int deg = marks[i];
        float a = (deg - 90.0f) * DEG_TO_RAD;

        int r1 = ROLL_RADIUS;
        int r2 = ROLL_RADIUS + ((deg % 30 == 0) ? 11 : 7);

        int x1 = CX + r1 * cosf(a);
        int y1 = CY + r1 * sinf(a);
        int x2 = CX + r2 * cosf(a);
        int y2 = CY + r2 * sinf(a);

        gfx->drawLine(x1, y1, x2, y2, WHITE_C);

        int tx = CX + (ROLL_RADIUS + 23) * cosf(a);
        int ty = CY + (ROLL_RADIUS + 23) * sinf(a);

        char label[8];
        snprintf(label, sizeof(label), "%d", deg);

        int textWidth = strlen(label) * 6;
        gfx->setCursor(tx - textWidth / 2, ty - 4);
        gfx->print(label);
    }

    // Fixed aircraft reference
    gfx->drawLine(CX - 46, CY, CX - 17, CY, YELLOW_C);
    gfx->drawLine(CX + 17, CY, CX + 46, CY, YELLOW_C);
    gfx->drawLine(CX - 17, CY, CX, CY + 12, YELLOW_C);
    gfx->drawLine(CX, CY + 12, CX + 17, CY, YELLOW_C);

}

// Clip code for Cohen-Sutherland line clipping algorithm  ไม่ให้การแสดงผล ของ pitch เกินออกไปทับ แถว roll ด้านบน
uint8_t clipCode(float x, float y)
{
    uint8_t code = 0;

    if (x < HUD_LEFT)         code |= 1;
    else if (x > HUD_RIGHT)   code |= 2;

    if (y < HUD_TOP)          code |= 4;
    else if (y > HUD_BOTTOM)  code |= 8;

    return code;
}

void drawClippedLine(
    float x1, float y1,
    float x2, float y2,
    uint16_t color)
{
    while (true) {
        uint8_t c1 = clipCode(x1, y1);
        uint8_t c2 = clipCode(x2, y2);

        if (!(c1 | c2)) {
            gfx->drawLine(
                (int)roundf(x1), (int)roundf(y1),
                (int)roundf(x2), (int)roundf(y2),
                color
            );
            return;
        }

        if (c1 & c2) return;

        uint8_t out = c1 ? c1 : c2;
        float x = 0, y = 0;

        if (out & 4) {
            if (y2 == y1) return;
            x = x1 + (x2-x1) * (HUD_TOP-y1)/(y2-y1);
            y = HUD_TOP;
        } else if (out & 8) {
            if (y2 == y1) return;
            x = x1 + (x2-x1) * (HUD_BOTTOM-y1)/(y2-y1);
            y = HUD_BOTTOM;
        } else if (out & 2) {
            if (x2 == x1) return;
            y = y1 + (y2-y1) * (HUD_RIGHT-x1)/(x2-x1);
            x = HUD_RIGHT;
        } else {
            if (x2 == x1) return;
            y = y1 + (y2-y1) * (HUD_LEFT-x1)/(x2-x1);
            x = HUD_LEFT;
        }

        if (out == c1) {
            x1 = x;
            y1 = y;
        } else {
            x2 = x;
            y2 = y;
        }
    }
}

bool isPitchLabelVisible(int x, int y, const char* label)
{
    const int charWidth = 6;
    const int charHeight = 8;

    int textWidth = strlen(label) * charWidth;

    return (
        x >= HUD_LEFT &&
        y >= HUD_TOP &&
        x + textWidth <= HUD_RIGHT &&
        y + charHeight <= HUD_BOTTOM
    );
}
// =====================================================
// Dynamic graphics
// =====================================================

void drawHorizon(float roll, float pitch, uint16_t color)
{
    float angle = roll * DEG_TO_RAD;
    float c = cosf(angle);
    float s = sinf(angle);

    float pitchOffset =
        constrain(pitch * 4.0f, -110.0f, 110.0f);

    const int marks[] = {-20, -10, 0, 10, 20};

    for (int i = 0; i < 5; i++) {
        int mark = marks[i];

        float offset = pitchOffset - mark * 4.0f;
        float halfWidth = (mark == 0)
            ? HORIZON_HALF : 40.0f;

        int x1 = CX - halfWidth * c - offset * s;
        int y1 = CY - halfWidth * s + offset * c;
        int x2 = CX + halfWidth * c - offset * s;
        int y2 = CY + halfWidth * s + offset * c;

        //gfx->drawLine(x1, y1, x2, y2, color);
        drawClippedLine(x1, y1, x2, y2, color);

        if (mark != 0) {
            char label[8];
            snprintf(label, sizeof(label), "%+d", mark);

            float lx = -halfWidth - 28.0f;
            float ly = offset - 4.0f;

            int tx = CX + lx * c - ly * s;
            int ty = CY + lx * s + ly * c;

            if (isPitchLabelVisible(tx, ty, label)) {
                gfx->setTextSize(1);
                gfx->setTextColor(color);
                gfx->setCursor(tx, ty);
                gfx->print(label);
            }
        }
    }
}

void drawRollPointer(float roll, uint16_t color)
{
    roll = constrain(roll, -60.0f, 60.0f);
    float a = (roll - 90.0f) * DEG_TO_RAD;

    int r1 = ROLL_RADIUS - 19;
    int r2 = ROLL_RADIUS - 5;

    int x1 = CX + r1 * cosf(a);
    int y1 = CY + r1 * sinf(a);
    int x2 = CX + r2 * cosf(a);
    int y2 = CY + r2 * sinf(a);

    gfx->drawLine(x1, y1, x2, y2, color);
}

// Navigation display: all values are drawn inside dedicated side panels.
// The artificial horizon and top/bottom status bars remain unchanged.
void drawNavCell(int x, int y, const char* title, const char* value,
                 uint16_t valueColor = WHITE_C) {
    constexpr int W = 174;
    constexpr int H = 78;
    gfx->fillRoundRect(x, y, W, H, 6, BAR);
    gfx->setTextSize(1);
    gfx->setTextColor(MUTED, BAR);
    gfx->setCursor(x + 11, y + 10);
    gfx->print(title);
    gfx->setTextSize(2);
    gfx->setTextColor(valueColor, BAR);
    gfx->setCursor(x + 11, y + 34);
    gfx->print(value);
}

// Evaluated on its own 5 Hz schedule. EXTENDED_SYS_STATE is authoritative
// when fresh; otherwise use a conservative altitude-based SITL fallback.
void evaluateFlightTime(uint32_t now) {
    const bool linkFresh = ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS;
    const bool extFresh = ageMs(lastExtendedStateTime, now) <= EXT_STATE_FRESH_MS;
    const bool posFresh = ageMs(lastPositionTime, now) <= POS_STALE_MS;
    if (!linkFresh) return; // do not invent a landing during telemetry loss

    if (!armed) {
        if (airborne) {
            lastAirborneDurationMs = now - airborneStartMs;
            airborne = false;
        }
        takeoffCandidateMs = 0;
        landingCandidateMs = 0;
        return;
    }
    if (extFresh) return; // EXTENDED_SYS_STATE handles transitions directly
    if (!posFresh || !isfinite(relativeAltitude)) return;

    if (!airborne) {
        landingCandidateMs = 0;
        if (relativeAltitude >= TAKEOFF_REL_ALT_M) {
            if (!takeoffCandidateMs) takeoffCandidateMs = now;
            if (now - takeoffCandidateMs >= FLIGHT_CONFIRM_MS) {
                airborne = true;
                airborneTimeStarted = true;
                flightTimeEstimated = true;
                airborneStartMs = takeoffCandidateMs;
                lastAirborneDurationMs = 0;
                takeoffCandidateMs = 0;
            }
        } else takeoffCandidateMs = 0;
    } else {
        takeoffCandidateMs = 0;
        if (relativeAltitude <= LANDING_REL_ALT_M) {
            if (!landingCandidateMs) landingCandidateMs = now;
            if (now - landingCandidateMs >= FLIGHT_CONFIRM_MS) {
                lastAirborneDurationMs = landingCandidateMs - airborneStartMs;
                airborne = false;
                landingCandidateMs = 0;
            }
        } else landingCandidateMs = 0;
    }
}

void drawNavigationPanels() {
    char text[24];
    uint32_t now = millis();
    bool vfrFresh = ageMs(lastVfrTime, now) <= VFR_STALE_MS;
    bool posFresh = ageMs(lastPositionTime, now) <= POS_STALE_MS;
    if (vfrFresh && isfinite(airSpeed) && airSpeed >= 0)
        snprintf(text, sizeof(text), "%.1f m/s", airSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(25, 96, "AIR SPEED", text);

    if (vfrFresh && isfinite(groundSpeed) && groundSpeed >= 0)
        snprintf(text, sizeof(text), "%.1f m/s", groundSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(25, 194, "GROUND SPEED", text);

    // Vertical speed from VFR_HUD.climb (m/s), positive climb / negative descent.
    // Keep the last value only while the VFR telemetry stream is fresh.
    if (vfrFresh && isfinite(verticalSpeed))
        snprintf(text, sizeof(text), "%+.1f m/s", verticalSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(25, 292, "VERT SPEED", text);

    if (posFresh && isfinite(relativeAltitude))
        snprintf(text, sizeof(text), "%.1f m", relativeAltitude);
    else
        snprintf(text, sizeof(text), "-- m");
    drawNavCell(601, 96, "ALT (REL HOME)", text);

    if (ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS)
        snprintf(text, sizeof(text), "%s", getFlightMode(flightMode));
    else
        snprintf(text, sizeof(text), "NO DATA");
    drawNavCell(601, 194, "FLIGHT MODE", text);

    // MISSION_CURRENT.seq is a mission item index, not necessarily a NAV_WAYPOINT.
    // Display it honestly; mission item 0 may be a TAKEOFF or another command.
    // MISSION_CURRENT is a mission item index. NAV_CONTROLLER_OUTPUT.wp_dist
    // is the autopilot-reported distance to the active navigation waypoint.
    // It may not be meaningful for non-navigation mission commands.
    gfx->fillRoundRect(601, 292, 174, 78, 6, BAR);
    gfx->setTextSize(1);
    gfx->setTextColor(MUTED, BAR);
    gfx->setCursor(612, 301);
    gfx->print("TARGET MISSION");
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE_C, BAR);
    gfx->setCursor(612, 319);
    if (missionCurrentValid && ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS)
        gfx->printf("ITEM %u", (unsigned)missionCurrentSeq);
    else
        gfx->print("ITEM --");
    gfx->setCursor(612, 343);
    if (missionCurrentValid && ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS &&
        ageMs(lastNavControllerTime, now) <= POS_STALE_MS &&
        isfinite(targetDistanceM) && targetDistanceM >= 0.0f)
        gfx->printf("%.0f m", targetDistanceM);
    else
        gfx->print("DIST --");
}

enum class LabelState : uint8_t { NORMAL, CAUTION, WARNING, UNKNOWN };

void drawStatusLabel(int x, int w, const char* name, const char* value,
                     LabelState state)
{
    uint16_t fill = STATUS_GRAY;
    if (state == LabelState::NORMAL) fill = STATUS_GREEN;
    else if (state == LabelState::CAUTION) fill = STATUS_AMBER;
    else if (state == LabelState::WARNING) fill = STATUS_RED;

    gfx->fillRoundRect(x, 10, w, 38, 5, fill);
    gfx->setTextColor(WHITE_C, fill);
    gfx->setTextSize(1);
    gfx->setCursor(x + 7, 14);
    gfx->print(name);
    gfx->setTextSize(2);
    gfx->setCursor(x + 7, 26);
    gfx->print(value);
}

// Status state is evaluated on its own clock, never inside LCD drawing.
struct StatusSnapshot {
    LabelState gps = LabelState::UNKNOWN;
    LabelState battery = LabelState::UNKNOWN;
    LabelState link = LabelState::UNKNOWN;
    uint8_t fix = 255;
    int8_t batteryPct = -1;
};

StatusSnapshot statusSnapshot;

// Non-critical recovery is held briefly to avoid color chatter.
// Worsening conditions are shown immediately on the next evaluation.
struct StatusFilter {
    LabelState shown = LabelState::UNKNOWN;
    LabelState candidate = LabelState::UNKNOWN;
    uint32_t candidateSince = 0;
};
StatusFilter gpsFilter, batteryFilter, linkFilter;

int severity(LabelState state) {
    switch (state) {
        case LabelState::NORMAL: return 0;
        case LabelState::CAUTION: return 1;
        case LabelState::WARNING: return 2;
        default: return 3; // Unknown is not treated as healthy
    }
}

LabelState filterStatus(StatusFilter& f, LabelState incoming, uint32_t now) {
    if (incoming != f.candidate) {
        f.candidate = incoming;
        f.candidateSince = now;
    }
    if (incoming == f.shown) return f.shown;
    if (severity(incoming) > severity(f.shown) ||
        (uint32_t)(now - f.candidateSince) >= RECOVERY_HOLD_MS) {
        f.shown = incoming;
    }
    return f.shown;
}

void evaluateStatus(uint32_t now) {
    ++statusEvaluationCount;
    const bool everLinked = lastHeartbeatTime != 0;
    const uint32_t hbAge = everLinked ? now - lastHeartbeatTime : UINT32_MAX;

    LabelState linkRaw = LabelState::UNKNOWN;
    if (everLinked) {
        linkRaw = hbAge > LINK_LOST_MS ? LabelState::WARNING :
                  hbAge > LINK_CAUTION_MS ? LabelState::CAUTION : LabelState::NORMAL;
    }

    LabelState gpsRaw = LabelState::UNKNOWN;
    if (lastGpsTime != 0) {
        uint32_t age = now - lastGpsTime;
        if (age > GPS_STALE_MS) gpsRaw = LabelState::UNKNOWN;
        else if (gpsFixType < 2) gpsRaw = LabelState::WARNING;
        else if (gpsFixType == 2 || age > GPS_CAUTION_MS) gpsRaw = LabelState::CAUTION;
        else gpsRaw = LabelState::NORMAL;
    }

    LabelState batteryRaw = LabelState::UNKNOWN;
    if (lastBatteryTime != 0 && batteryRemaining >= 0 && batteryRemaining <= 100) {
        uint32_t age = now - lastBatteryTime;
        if (age > BAT_STALE_MS) batteryRaw = LabelState::UNKNOWN;
        else if (batteryRemaining <= BAT_WARNING_PCT) batteryRaw = LabelState::WARNING;
        else if (batteryRemaining <= BAT_CAUTION_PCT || age > BAT_CAUTION_MS)
            batteryRaw = LabelState::CAUTION;
        else batteryRaw = LabelState::NORMAL;
    }

    statusSnapshot.link = filterStatus(linkFilter, linkRaw, now);
    statusSnapshot.gps = filterStatus(gpsFilter, gpsRaw, now);
    statusSnapshot.battery = filterStatus(batteryFilter, batteryRaw, now);
    statusSnapshot.fix = gpsFixType;
    statusSnapshot.batteryPct = batteryRemaining;
}

void drawStatus()
{
    static int lastGpsState = -1;
    static int lastBatteryState = -1;
    static int lastLinkState = -1;
    static uint8_t lastFix = 255;
    static int lastBatteryPct = -999;

    const LabelState gpsState = statusSnapshot.gps;
    const LabelState batteryState = statusSnapshot.battery;
    const LabelState linkState = statusSnapshot.link;

    if ((int)gpsState != lastGpsState ||
        (gpsState != LabelState::UNKNOWN && statusSnapshot.fix != lastFix)) {
        const char* gpsText = "NO DATA";
        if (gpsState != LabelState::UNKNOWN) {
            if (statusSnapshot.fix >= 3) gpsText = "3D FIX";
            else if (statusSnapshot.fix == 2) gpsText = "2D FIX";
            else gpsText = "NO FIX";
        }
        drawStatusLabel(205, 104, "GPS", gpsText, gpsState);
        lastGpsState = (int)gpsState;
        lastFix = statusSnapshot.fix;
    }
    if ((int)batteryState != lastBatteryState ||
        (batteryState != LabelState::UNKNOWN && statusSnapshot.batteryPct != lastBatteryPct)) {
        char batteryText[12] = "NO DATA";
        if (batteryState != LabelState::UNKNOWN)
            snprintf(batteryText, sizeof(batteryText), "%d%%", (int)statusSnapshot.batteryPct);
        drawStatusLabel(315, 104, "BATTERY", batteryText, batteryState);
        lastBatteryState = (int)batteryState;
        lastBatteryPct = statusSnapshot.batteryPct;
    }
    // EKF parsing not implemented: do not display a false healthy state.
    if (!hudDrawn) drawStatusLabel(425, 104, "EKF", "NO DATA", LabelState::UNKNOWN);

    if ((int)linkState != lastLinkState) {
        drawStatusLabel(555, 230, "MAVLINK",
                        linkState == LabelState::NORMAL ? "CONNECTED" :
                        linkState == LabelState::CAUTION ? "DELAYED" :
                        linkState == LabelState::UNKNOWN ? "NO DATA" : "LOST",
                        linkState);
        lastLinkState = (int)linkState;
    }

    // Bottom bar: GPS HDOP + satellites | ARM state | time airborne.
    // Always redraw the three dedicated areas to avoid stale characters.
    const uint32_t now = millis();
    const bool gpsFresh = ageMs(lastGpsTime, now) <= GPS_STALE_MS;
    gfx->fillRect(12, 433, 325, 38, BAR);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE_C, BAR);
    gfx->setCursor(18, 443);
    if (gpsFresh && gpsHdopCenti != UINT16_MAX)
        gfx->printf("HDOP %.2f", gpsHdopCenti / 100.0f);
    else
        gfx->print("HDOP --");
    gfx->setCursor(190, 443);
    if (gpsFresh && gpsSatellites != 255)
        gfx->printf("SAT %u", (unsigned)gpsSatellites);
    else
        gfx->print("SAT --");

    gfx->fillRect(350, 433, 215, 38, BAR);
    gfx->setTextSize(2);
    gfx->setTextColor(armed ? RED_C : YELLOW_C, BAR);
    gfx->setCursor(365, 443);
    gfx->print(armed ? "ARMED" : "DISARMED");

    gfx->fillRect(570, 433, 220, 38, BAR);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE_C, BAR);
    gfx->setCursor(579, 443);
    if (airborneTimeStarted) {
        uint32_t elapsed = airborne ? now - airborneStartMs : lastAirborneDurationMs;
        uint32_t secs = elapsed / 1000;
        gfx->printf("AIR %02lu:%02lu:%02lu",
                    (unsigned long)(secs / 3600),
                    (unsigned long)((secs / 60) % 60),
                    (unsigned long)(secs % 60));
    } else {
        gfx->print("AIR 00:00:00");
    }
}

void drawAircraftReference()
{
    // Fixed aircraft reference
    gfx->drawLine(CX - 46, CY, CX - 17, CY, YELLOW_C);
    gfx->drawLine(CX + 17, CY, CX + 46, CY, YELLOW_C);

    gfx->drawLine(CX - 17, CY, CX, CY + 12, YELLOW_C);
    gfx->drawLine(CX, CY + 12, CX + 17, CY, YELLOW_C);

    // Center reference point
    gfx->fillCircle(CX, CY, 3, YELLOW_C);
}

constexpr int INFO_Y = 355;
constexpr int INFO_H = 51;

void drawFlightInfoPanel()
{
    // พื้นหลังแถบข้อมูล
    gfx->fillRoundRect(
        205, INFO_Y, 390, INFO_H,
        5, BAR
    );

    gfx->setTextSize(1);
    gfx->setTextColor(MUTED, BAR);

    gfx->setCursor(215, INFO_Y + 7);
    gfx->print("GS");

    gfx->setCursor(305, INFO_Y + 7);
    gfx->print("ALT");

    gfx->setCursor(400, INFO_Y + 7);
    gfx->print("BATT");

    gfx->setCursor(505, INFO_Y + 7);
    gfx->print("GPS");
}

void updateFlightInfo()
{
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE_C, BAR);

    gfx->setCursor(215, INFO_Y + 23);
    if (groundSpeed >= 0 && ageMs(lastVfrTime, millis()) <= VFR_STALE_MS)
        gfx->printf("%5.1f", groundSpeed);
    else
        gfx->print("  -- ");

    gfx->setCursor(305, INFO_Y + 23);
    if (isfinite(relativeAltitude) && ageMs(lastPositionTime, millis()) <= POS_STALE_MS)
        gfx->printf("%5.1f", relativeAltitude);
    else
        gfx->print("  -- ");

    gfx->setCursor(400, INFO_Y + 23);
    if (batteryVoltage >= 0 && ageMs(lastBatteryTime, millis()) <= BAT_STALE_MS)
        gfx->printf("%5.1f", batteryVoltage);
    else
        gfx->print("  -- ");

    gfx->setCursor(505, INFO_Y + 23);

    if (ageMs(lastGpsTime, millis()) > GPS_STALE_MS) {
        gfx->print("--   ");
        return;
    }
    switch (gpsFixType) {
        case 2: gfx->print("2D   "); break;
        case 3: gfx->print("3D   "); break;
        case 4: gfx->print("DGPS "); break;
        case 5: gfx->print("RTK F"); break;
        case 6: gfx->print("RTK X"); break;
        default: gfx->print("NOFIX"); break;
    }
}

void updateHUD()
{
    if (hudDrawn) {
        drawHorizon(oldRoll, oldPitch, PANEL);
        drawRollPointer(oldRoll, PANEL);
    }

    drawHorizon(rollDeg, pitchDeg, CYAN_C);
    drawRollPointer(rollDeg, YELLOW_C);

        // วาด Reference ทับเป็นลำดับสุดท้ายของกราฟิก
    drawAircraftReference();
    
    drawNavigationPanels();
    drawStatus();

    oldRoll = rollDeg;
    oldPitch = pitchDeg;
    oldYaw = yawDeg;

    hudDrawn = true;
}

// =====================================================
// MAVLink
// =====================================================

void handleMavlink(const mavlink_message_t& msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            // Ignore heartbeats from non-autopilot components
            if (msg.compid != MAV_COMP_ID_AUTOPILOT1)
                break;

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            flightMode = hb.custom_mode;
            bool newArmed =
                (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            armed = newArmed;

            lastHeartbeatTime = millis();
            noteStream(S_HB, lastHeartbeatTime);
            heartbeatCount++;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);

            rollDeg = att.roll * RAD_TO_DEG;
            pitchDeg = att.pitch * RAD_TO_DEG;
            yawDeg = att.yaw * RAD_TO_DEG;

            lastAttitudeTime = millis();
            noteStream(S_ATT, lastAttitudeTime);
            attitudeCount++;
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&msg, &sys);

            batteryVoltage =
                (sys.voltage_battery == UINT16_MAX)
                    ? -1.0f
                    : sys.voltage_battery / 1000.0f;

            batteryCurrent =
                (sys.current_battery == -1)
                    ? -1.0f
                    : sys.current_battery / 100.0f;

            batteryRemaining = sys.battery_remaining;
            lastBatteryTime = millis();
            noteStream(S_BAT, lastBatteryTime);
            sysStatusCount++;
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t vfr;
            mavlink_msg_vfr_hud_decode(&msg, &vfr);

            airSpeed = vfr.airspeed;
            groundSpeed = vfr.groundspeed;
            verticalSpeed = vfr.climb;
            lastVfrTime = millis();
            noteStream(S_VFR, lastVfrTime);
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);

            relativeAltitude = pos.relative_alt / 1000.0f;
            lastPositionTime = millis();
            noteStream(S_POS, lastPositionTime);
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CURRENT: {
            // Mission item sequence; not all items are geographic waypoints.
            mavlink_mission_current_t mission;
            mavlink_msg_mission_current_decode(&msg, &mission);
            missionCurrentSeq = mission.seq;
            missionCurrentValid = true;
            break;
        }

        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
            mavlink_extended_sys_state_t ext;
            mavlink_msg_extended_sys_state_decode(&msg, &ext);
            lastExtendedStateTime = millis();
            if (ext.landed_state == MAV_LANDED_STATE_IN_AIR && !airborne) {
                airborne = true;
                airborneTimeStarted = true;
                flightTimeEstimated = false;
                airborneStartMs = lastExtendedStateTime;
                lastAirborneDurationMs = 0;
                takeoffCandidateMs = 0;
                landingCandidateMs = 0;
            } else if (ext.landed_state == MAV_LANDED_STATE_ON_GROUND && airborne) {
                lastAirborneDurationMs = lastExtendedStateTime - airborneStartMs;
                airborne = false;
                landingCandidateMs = 0;
            }
            break;
        }

        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
            mavlink_nav_controller_output_t nav;
            mavlink_msg_nav_controller_output_decode(&msg, &nav);
            targetDistanceM = (float)nav.wp_dist;
            lastNavControllerTime = millis();
            break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);

            gpsFixType = gps.fix_type;
            gpsHdopCenti = gps.eph;  // GPS_RAW_INT.eph is HDOP * 100, UINT16_MAX means unknown
            gpsSatellites = gps.satellites_visible;  // 255 means unknown
            lastGpsTime = millis();
            noteStream(S_GPS, lastGpsTime);
            break;
        }
    }
}

void receiveMavlink()
{
    int packetSize = udp.parsePacket();

    if (packetSize <= 0)
        return;

    totalPackets++;

    while (udp.available() > 0) {
        int b = udp.read();
        if (b < 0) break;

        if (mavlink_parse_char(
                MAVLINK_COMM_0,
                (uint8_t)b,
                &mavMsg,
                &mavStatus
            )) {
            totalMessages++;
            handleMavlink(mavMsg);
        }
    }
}

// =====================================================
// เริ่มโปรแกรม 
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);

    if (!gfx->begin()) {
        Serial.println("LCD INIT FAILED");
        while (true) delay(1000);
    }

    drawStaticUI();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Connecting WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());

    if (!udp.begin(UDP_PORT)) {
        Serial.println("UDP INIT FAILED");
    } else {
        Serial.println("UDP Port 14550 READY");
    }

    Serial.printf(
        "PSRAM: %s | Free: %u\n",
        psramFound() ? "YES" : "NO",
        (unsigned int)ESP.getFreePsram()
    );

    Serial.println("LEUS HUD v0.20.2 NAVIGATION READY");

    lastHudUpdate = millis();
    lastStatusUpdate = lastHudUpdate;
    evaluateStatus(lastHudUpdate);
    lastSerialReport = millis();
}

// =====================================================
// Main Loop
// =====================================================

void loop()
{
    receiveMavlink();

    uint32_t now = millis();

    if (now - lastStatusUpdate >= STATUS_INTERVAL_MS) {
        lastStatusUpdate = now;
        evaluateStatus(now);
        evaluateFlightTime(now);
    }

    if (now - lastHudUpdate >= HUD_INTERVAL_MS) {
        lastHudUpdate = now;
        updateHUD();
        ++hudFrameCount;
    }

    if (now - lastSerialReport >= 2000) {
        uint32_t elapsed = now - lastSerialReport;
        lastSerialReport = now;
        Serial.printf("TIMING HUD=%.1fHz STATUS=%.1fHz HB_age=%lums GPS_age=%lums BATT_age=%lums ATT_age=%lums\n",
            elapsed ? hudFrameCount * 1000.0f / elapsed : 0.0f,
            elapsed ? statusEvaluationCount * 1000.0f / elapsed : 0.0f,
            (unsigned long)(lastHeartbeatTime ? now - lastHeartbeatTime : UINT32_MAX),
            (unsigned long)(lastGpsTime ? now - lastGpsTime : UINT32_MAX),
            (unsigned long)(lastBatteryTime ? now - lastBatteryTime : UINT32_MAX),
            (unsigned long)(lastAttitudeTime ? now - lastAttitudeTime : UINT32_MAX));
        hudFrameCount = 0;
        statusEvaluationCount = 0;
        for (uint8_t i = 0; i < S_COUNT; ++i) {
            StreamStats& st = streams[i];
            Serial.printf("RATE %-3s=%5.1fHz age=%lums maxGap=%lums\n",
                st.name,
                elapsed ? st.received * 1000.0f / elapsed : 0.0f,
                (unsigned long)ageMs(st.lastMs, now),
                (unsigned long)st.intervalMaxMs);
            st.received = 0;
            st.intervalMaxMs = 0;
        }

        Serial.printf(
            "UDP=%lu Msg=%lu HB=%lu ATT=%lu SYS=%lu "
            "Roll=%.2f Pitch=%.2f Yaw=%.2f "
            "FreePSRAM=%u\n",
            (unsigned long)totalPackets,
            (unsigned long)totalMessages,
            (unsigned long)heartbeatCount,
            (unsigned long)attitudeCount,
            (unsigned long)sysStatusCount,
            rollDeg,
            pitchDeg,
            normalizeYaw(yawDeg),
            (unsigned int)ESP.getFreePsram()
        );
    }
}
