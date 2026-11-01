#include "telemetry.h"
#include "flight_util.h"
#include "flight_time.h"
#include "flight_status.h"
#include "hud_display.h"
#include "mavlink_receiver.h"

// LEUS-52-W01 Day 2: measurement only, no graphics behavior changes.
static uint64_t hudRenderTotalUs = 0;
static uint32_t hudRenderMaxUs = 0;
static uint32_t hudRenderSamples = 0;

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

    // Approximate back-to-back micros() measurement overhead (not LCD cost).
    uint32_t overheadSumUs = 0;
    for (unsigned i = 0; i < 1000; ++i) {
        uint32_t a = micros();
        overheadSumUs += (uint32_t)(micros() - a);
    }
    Serial.printf("PROFILE TIMER pair_avg=%.3fus samples=1000\n",
                  (double)overheadSumUs / 1000.0);

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

    Serial.println("LEUS HUD v0.20.3 NAVIGATION READY");

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
        uint32_t renderStartUs = micros();
        updateHUD();
        uint32_t renderUs = (uint32_t)(micros() - renderStartUs);
        hudRenderTotalUs += renderUs;
        if (renderUs > hudRenderMaxUs) hudRenderMaxUs = renderUs;
        ++hudRenderSamples;
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
        Serial.printf("RENDER HUD avg=%.2fms max=%.2fms samples=%lu\n",
            hudRenderSamples ? (double)hudRenderTotalUs / hudRenderSamples / 1000.0 : 0.0,
            (double)hudRenderMaxUs / 1000.0,
            (unsigned long)hudRenderSamples);
        // Snapshot after the measurement window; no Serial I/O inside draw calls.
        const HudSectionStats sections[] = {
            hudHorizonStats, hudReferenceStats, hudNavStats, hudStatusStats
        };
        const char* names[] = {"HORIZON", "REFERENCE", "NAV", "STATUS"};
        for (unsigned i = 0; i < 4; ++i) {
            const HudSectionStats &st = sections[i];
            Serial.printf("PROFILE %-9s avg=%.2fms max=%.2fms samples=%lu\n",
                names[i],
                st.samples ? (double)st.totalUs / st.samples / 1000.0 : 0.0,
                (double)st.maxUs / 1000.0,
                (unsigned long)st.samples);
        }
        reportHudDirtyStats();
        resetHudSectionStats();
        hudRenderTotalUs = 0;
        hudRenderMaxUs = 0;
        hudRenderSamples = 0;
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
