#include "flight_util.h"
// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
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

