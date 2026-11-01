#include "flight_time.h"
// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
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

