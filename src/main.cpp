
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_GFX_Library.h>
#include <common/mavlink.h>
#include <math.h>

// =====================================================
// LEUS HUD - TEST 16
// ESP32-S3 / JC8048W550 / 800x480
// =====================================================

// WiFi - ใช้ค่าจาก Test 15 เดิม
const char* WIFI_SSID = "steevapong 2.4G";
const char* WIFI_PASS = "Teera2512";

constexpr uint16_t UDP_PORT = 14550;
constexpr uint32_t HUD_INTERVAL_MS = 100;

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

// Layout coordinates
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
uint32_t lastSerialReport = 0;
uint32_t lastHeartbeatTime = 0;

bool hudDrawn = false;
float oldRoll = 0;
float oldPitch = 0;
float oldYaw = 0;
bool oldArmed = false;
uint32_t oldMode = UINT32_MAX;
bool oldConnected = false;

float groundSpeed = -1.0f;
float relativeAltitude = NAN;
uint8_t gpsFixType = 0;

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
    // ArduCopter custom_mode
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
    // LEUS Header - Red Background / White Text
    constexpr uint16_t LEUS_RED = 0xD104;

    gfx->fillRoundRect(12, 8, 185, 42, 6, LEUS_RED);

    gfx->setTextColor(WHITE_C, LEUS_RED);
    gfx->setTextSize(3);
    gfx->setCursor(22, 17);
    gfx->print("LEUS HUD");

    // Main panel
    gfx->fillRoundRect(10, 68, 780, 350, 8, PANEL);
    gfx->drawRoundRect(10, 68, 780, 350, 8, BORDER);

    // Left telemetry labels
    gfx->setTextSize(2);
    gfx->setTextColor(MUTED);
    gfx->setCursor(28, 104);
    gfx->print("ROLL");
    gfx->setCursor(28, 182);
    gfx->print("PITCH");

    // Right telemetry label
    gfx->setCursor(630, 104);
    gfx->print("YAW");

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

    drawFlightInfoPanel();
}

// Clip code for Cohen-Sutherland line clipping algorithm
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

void drawTelemetry()
{
    gfx->setTextSize(3);
    gfx->setTextColor(WHITE_C, PANEL);

    gfx->setCursor(28, 135);
    gfx->printf("%+7.2f  ", rollDeg);

    gfx->setCursor(28, 213);
    gfx->printf("%+7.2f  ", pitchDeg);

    gfx->setCursor(630, 135);
    gfx->printf("%6.1f  ", normalizeYaw(yawDeg));

    //gfx->setCursor(655, 135);
    //gfx->printf("%6.1f  ", normalizeYaw(yawDeg));
}

void drawStatus()
{
    bool connected =
        (lastHeartbeatTime != 0) &&
        (millis() - lastHeartbeatTime < 3000);

    // Top-right connection indicator
    if (connected != oldConnected) {
        gfx->fillRect(555, 12, 230, 36, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(
            connected ? GREEN_C : RED_C, BAR
        );
        gfx->setCursor(560, 20);
        gfx->print(
            connected ? "MAVLINK CONNECTED"
                      : "MAVLINK LOST"
        );
        oldConnected = connected;
    }

    // Bottom flight mode
    if (flightMode != oldMode) {
        gfx->fillRect(15, 434, 325, 35, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE_C, BAR);
        gfx->setCursor(18, 443);
        gfx->printf("MODE: %s", getFlightMode(flightMode));
        oldMode = flightMode;
    }

    // Armed status
    if (armed != oldArmed || !hudDrawn) {
        gfx->fillRect(350, 434, 220, 35, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(
            armed ? RED_C : YELLOW_C, BAR
        );
        gfx->setCursor(365, 443);
        gfx->print(armed ? "ARMED" : "DISARMED");
        oldArmed = armed;
    }

    // Fixed update-rate label
    if (!hudDrawn) {
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE_C, BAR);
        gfx->setCursor(675, 443);
        gfx->print("10 Hz");
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
    if (groundSpeed >= 0)
        gfx->printf("%5.1f", groundSpeed);
    else
        gfx->print("  -- ");

    gfx->setCursor(305, INFO_Y + 23);
    if (isfinite(relativeAltitude))
        gfx->printf("%5.1f", relativeAltitude);
    else
        gfx->print("  -- ");

    gfx->setCursor(400, INFO_Y + 23);
    if (batteryVoltage >= 0)
        gfx->printf("%5.1f", batteryVoltage);
    else
        gfx->print("  -- ");

    gfx->setCursor(505, INFO_Y + 23);

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
    
    drawTelemetry();
    drawStatus();

    // วาดแถบข้อมูลทับส่วนล่างของ Horizon
    drawFlightInfoPanel();
    updateFlightInfo();
    //drawFlightInfoPanel();
    //updateFlightInfo();

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
            armed =
                (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

            lastHeartbeatTime = millis();
            heartbeatCount++;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);

            rollDeg = att.roll * RAD_TO_DEG;
            pitchDeg = att.pitch * RAD_TO_DEG;
            yawDeg = att.yaw * RAD_TO_DEG;

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

            sysStatusCount++;
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {
            mavlink_vfr_hud_t vfr;
            mavlink_msg_vfr_hud_decode(&msg, &vfr);

            groundSpeed = vfr.groundspeed;
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);

            relativeAltitude = pos.relative_alt / 1000.0f;
            break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT: {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);

            gpsFixType = gps.fix_type;
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
// Setup
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

    Serial.println("LEUS HUD TEST 16 READY");

    lastHudUpdate = millis();
    lastSerialReport = millis();
}

// =====================================================
// Main Loop
// =====================================================

void loop()
{
    receiveMavlink();

    uint32_t now = millis();

    if (now - lastHudUpdate >= HUD_INTERVAL_MS) {
        lastHudUpdate = now;
        updateHUD();
    }

    if (now - lastSerialReport >= 2000) {
        lastSerialReport = now;

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
