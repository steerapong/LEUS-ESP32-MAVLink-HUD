#pragma once
#include "telemetry.h"
void drawStaticUI();
void updateHUD();

// Profiling instrumentation only; accumulated in RAM, printed by main.cpp.
struct HudSectionStats {
    uint64_t totalUs;
    uint32_t maxUs;
    uint32_t samples;
};
extern HudSectionStats hudHorizonStats;
extern HudSectionStats hudNavStats;
extern HudSectionStats hudStatusStats;
extern HudSectionStats hudReferenceStats;
void resetHudSectionStats();

// Report dirty-update counts once per 2-second diagnostics window.
void reportHudDirtyStats();
