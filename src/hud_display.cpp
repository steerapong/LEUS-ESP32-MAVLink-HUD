#include "hud_display.h"
#include "flight_status.h"
#include "flight_util.h"
// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
// LEUS-52-W01 Day 2 Step 2: time sections without changing drawing operations.
HudSectionStats hudHorizonStats = {};
HudSectionStats hudNavStats = {};
HudSectionStats hudStatusStats = {};
HudSectionStats hudReferenceStats = {};
static inline void recordHudTime(HudSectionStats &st, uint32_t elapsedUs) {
    st.totalUs += elapsedUs;
    if (elapsedUs > st.maxUs) st.maxUs = elapsedUs;
    ++st.samples;
}
void resetHudSectionStats() {
    hudHorizonStats = {};
    hudNavStats = {};
    hudStatusStats = {};
    hudReferenceStats = {};
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
// Dirty-update cache: compare the actual displayed string, not raw telemetry.
// This also handles transitions to stale/NO DATA without forcing full redraws.
static char navLastText[7][48] = {};
static bool navValid[7] = {};
static uint32_t navCellDraws = 0;
static uint32_t navCellSkips = 0;
static uint32_t statusAreaDraws = 0;
static uint32_t statusAreaSkips = 0;

void reportHudDirtyStats() {
    Serial.printf("DIRTY NAV draws=%lu skips=%lu STATUS draws=%lu skips=%lu\n",
        (unsigned long)navCellDraws, (unsigned long)navCellSkips,
        (unsigned long)statusAreaDraws, (unsigned long)statusAreaSkips);
    navCellDraws = navCellSkips = statusAreaDraws = statusAreaSkips = 0;
}

static bool navChanged(unsigned slot, const char* value) {
    if (navValid[slot] && strcmp(navLastText[slot], value) == 0) {
        ++navCellSkips;
        return false;
    }
    snprintf(navLastText[slot], sizeof(navLastText[slot]), "%s", value);
    navValid[slot] = true;
    ++navCellDraws;
    return true;
}

void drawNavCell(unsigned slot, int x, int y, const char* title, const char* value,
                 uint16_t valueColor = WHITE_C) {
    if (!navChanged(slot, value)) return;
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

// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
void drawNavigationPanels() {
    char text[24];
    uint32_t now = millis();
    bool vfrFresh = ageMs(lastVfrTime, now) <= VFR_STALE_MS;
    bool posFresh = ageMs(lastPositionTime, now) <= POS_STALE_MS;
    if (vfrFresh && isfinite(airSpeed) && airSpeed >= 0)
        snprintf(text, sizeof(text), "%.1f m/s", airSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(0, 25, 96, "AIR SPEED", text);

    if (vfrFresh && isfinite(groundSpeed) && groundSpeed >= 0)
        snprintf(text, sizeof(text), "%.1f m/s", groundSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(1, 25, 194, "GROUND SPEED", text);

    // Vertical speed from VFR_HUD.climb (m/s), positive climb / negative descent.
    // Keep the last value only while the VFR telemetry stream is fresh.
    if (vfrFresh && isfinite(verticalSpeed))
        snprintf(text, sizeof(text), "%+.1f m/s", verticalSpeed);
    else
        snprintf(text, sizeof(text), "-- m/s");
    drawNavCell(2, 25, 292, "VERT SPEED", text);

    if (posFresh && isfinite(relativeAltitude))
        snprintf(text, sizeof(text), "%.1f m", relativeAltitude);
    else
        snprintf(text, sizeof(text), "-- m");
    drawNavCell(3, 601, 96, "ALT (REL HOME)", text);

    if (ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS)
        snprintf(text, sizeof(text), "%s", getFlightMode(flightMode));
    else
        snprintf(text, sizeof(text), "NO DATA");
    drawNavCell(4, 601, 194, "FLIGHT MODE", text);

    // MISSION_CURRENT.seq is a mission item index, not necessarily a NAV_WAYPOINT.
    // Display it honestly; mission item 0 may be a TAKEOFF or another command.
    // MISSION_CURRENT is a mission item index. NAV_CONTROLLER_OUTPUT.wp_dist
    // is the autopilot-reported distance to the active navigation waypoint.
    // It may not be meaningful for non-navigation mission commands.
    char missionItem[24];
    char missionDistance[24];
    if (missionCurrentValid && ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS)
        snprintf(missionItem, sizeof(missionItem), "ITEM %u", (unsigned)missionCurrentSeq);
    else
        snprintf(missionItem, sizeof(missionItem), "ITEM --");
    if (missionCurrentValid && ageMs(lastHeartbeatTime, now) <= LINK_LOST_MS &&
        ageMs(lastNavControllerTime, now) <= POS_STALE_MS &&
        isfinite(targetDistanceM) && targetDistanceM >= 0.0f)
        snprintf(missionDistance, sizeof(missionDistance), "%.0f m", targetDistanceM);
    else
        snprintf(missionDistance, sizeof(missionDistance), "DIST --");
    char missionKey[48];
    snprintf(missionKey, sizeof(missionKey), "%s|%s", missionItem, missionDistance);
    if (navChanged(5, missionKey)) {
        gfx->fillRoundRect(601, 292, 174, 78, 6, BAR);
        gfx->setTextSize(1);
        gfx->setTextColor(MUTED, BAR);
        gfx->setCursor(612, 301);
        gfx->print("TARGET MISSION");
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE_C, BAR);
        gfx->setCursor(612, 319);
        gfx->print(missionItem);
        gfx->setCursor(612, 343);
        gfx->print(missionDistance);
    }
}


// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
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

    // Compare formatted display strings. Flight time naturally changes once/sec.
    // Invalidate when GPS becomes stale, even without incoming messages.
    const uint32_t now = millis();
    const bool gpsFresh = ageMs(lastGpsTime, now) <= GPS_STALE_MS;
    char gpsLine[48];
    char hdop[24];
    char sat[24];
    if (gpsFresh && gpsHdopCenti != UINT16_MAX)
        snprintf(hdop, sizeof(hdop), "HDOP %.2f", gpsHdopCenti / 100.0f);
    else
        snprintf(hdop, sizeof(hdop), "HDOP --");
    if (gpsFresh && gpsSatellites != 255)
        snprintf(sat, sizeof(sat), "SAT %u", (unsigned)gpsSatellites);
    else
        snprintf(sat, sizeof(sat), "SAT --");
    snprintf(gpsLine, sizeof(gpsLine), "%s|%s", hdop, sat);

    static char lastGpsLine[48] = {};
    static char lastArmLine[16] = {};
    static char lastTimeLine[32] = {};
    static bool bottomValid[3] = {};
    if (!bottomValid[0] || strcmp(lastGpsLine, gpsLine) != 0) {
        snprintf(lastGpsLine, sizeof(lastGpsLine), "%s", gpsLine);
        bottomValid[0] = true;
        ++statusAreaDraws;
        gfx->fillRect(12, 433, 325, 38, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE_C, BAR);
        gfx->setCursor(18, 443);
        gfx->print(hdop);
        gfx->setCursor(190, 443);
        gfx->print(sat);
    } else ++statusAreaSkips;

    const char* armLine = armed ? "ARMED" : "DISARMED";
    if (!bottomValid[1] || strcmp(lastArmLine, armLine) != 0) {
        snprintf(lastArmLine, sizeof(lastArmLine), "%s", armLine);
        bottomValid[1] = true;
        ++statusAreaDraws;
        gfx->fillRect(350, 433, 215, 38, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(armed ? RED_C : YELLOW_C, BAR);
        gfx->setCursor(365, 443);
        gfx->print(armLine);
    } else ++statusAreaSkips;

    char timeLine[32];
    if (airborneTimeStarted) {
        uint32_t elapsed = airborne ? now - airborneStartMs : lastAirborneDurationMs;
        uint32_t secs = elapsed / 1000;
        snprintf(timeLine, sizeof(timeLine), "AIR %02lu:%02lu:%02lu",
                 (unsigned long)(secs / 3600),
                 (unsigned long)((secs / 60) % 60),
                 (unsigned long)(secs % 60));
    } else snprintf(timeLine, sizeof(timeLine), "AIR 00:00:00");
    if (!bottomValid[2] || strcmp(lastTimeLine, timeLine) != 0) {
        snprintf(lastTimeLine, sizeof(lastTimeLine), "%s", timeLine);
        bottomValid[2] = true;
        ++statusAreaDraws;
        gfx->fillRect(570, 433, 220, 38, BAR);
        gfx->setTextSize(2);
        gfx->setTextColor(WHITE_C, BAR);
        gfx->setCursor(579, 443);
        gfx->print(timeLine);
    } else ++statusAreaSkips;
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
    uint32_t t = micros();
    if (hudDrawn) {
        drawHorizon(oldRoll, oldPitch, PANEL);
        drawRollPointer(oldRoll, PANEL);
    }

    drawHorizon(rollDeg, pitchDeg, CYAN_C);
    drawRollPointer(rollDeg, YELLOW_C);
    recordHudTime(hudHorizonStats, (uint32_t)(micros() - t));

    t = micros();
    drawAircraftReference();
    recordHudTime(hudReferenceStats, (uint32_t)(micros() - t));

    t = micros();
    drawNavigationPanels();
    recordHudTime(hudNavStats, (uint32_t)(micros() - t));

    t = micros();
    drawStatus();
    recordHudTime(hudStatusStats, (uint32_t)(micros() - t));

    oldRoll = rollDeg;
    oldPitch = pitchDeg;
    oldYaw = yawDeg;

    hudDrawn = true;
}
