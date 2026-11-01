#pragma once
#include "telemetry.h"
enum class LabelState : uint8_t { NORMAL, CAUTION, WARNING, UNKNOWN };

struct StatusSnapshot {
    LabelState gps = LabelState::UNKNOWN;
    LabelState battery = LabelState::UNKNOWN;
    LabelState link = LabelState::UNKNOWN;
    uint8_t fix = 255;
    int8_t batteryPct = -1;
};

struct StatusFilter {
    LabelState shown = LabelState::UNKNOWN;
    LabelState candidate = LabelState::UNKNOWN;
    uint32_t candidateSince = 0;
};

extern StatusSnapshot statusSnapshot;
extern StatusFilter gpsFilter, batteryFilter, linkFilter;
void evaluateStatus(uint32_t now);
void drawStatusLabel(int x, int w, const char* name, const char* value, LabelState state);
