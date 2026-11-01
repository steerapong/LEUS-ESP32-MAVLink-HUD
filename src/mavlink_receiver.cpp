#include "mavlink_receiver.h"
// Extracted verbatim from LEUS HUD v0.20.3; included in original order.
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

