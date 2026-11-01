#include "telemetry.h"
#include "flight_util.h"
#include "flight_time.h"
#include "flight_status.h"
#include "hud_display.h"
#include "mavlink_receiver.h"

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
