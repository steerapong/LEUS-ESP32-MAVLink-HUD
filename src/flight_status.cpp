#include "flight_status.h"
// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
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
StatusSnapshot statusSnapshot;

// Non-critical recovery is held briefly to avoid color chatter.
// Worsening conditions are shown immediately on the next evaluation.
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

