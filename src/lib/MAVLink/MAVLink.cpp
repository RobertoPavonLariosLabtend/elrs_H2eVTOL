#include "MAVLink.h"
#if !defined(PLATFORM_STM32)
    #include "ardupilot_protocol.h"
#endif

#if !defined(PLATFORM_STM32)
static char ascii_tolower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

static bool mavlink_name_equals_ignore_case(const char *name, const char *expected, size_t expectedLen, size_t fieldLen)
{
    if (expectedLen > fieldLen)
    {
        return false;
    }

    for (size_t i = 0; i < expectedLen; ++i)
    {
        if (ascii_tolower(name[i]) != ascii_tolower(expected[i]))
        {
            return false;
        }
    }

    if (expectedLen == fieldLen)
    {
        return true;
    }

    return name[expectedLen] == '\0';
}

static uint16_t crsf_gps_altitude_from_h2(float h2)
{
    int32_t crsf_altitude = (int32_t)(h2 + 1000.0f);

    if (crsf_altitude < 0)
    {
        return 0;
    }
    if (crsf_altitude > 32767)
    {
        return 32767;
    }

    return (uint16_t)crsf_altitude;
}

static void send_h2_vario(int16_t h2_vspd_value, Handset *handset)
{
    CRSF_MK_FRAME_T(crsf_sensor_vario_t)
    crsfvario = {0};
    crsfvario.p.verticalspd = htobe16(h2_vspd_value);
    CRSF::SetHeaderAndCrc((uint8_t *)&crsfvario, CRSF_FRAMETYPE_VARIO, CRSF_FRAME_SIZE(sizeof(crsf_sensor_vario_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    handset->sendTelemetryToTX((uint8_t *)&crsfvario);
}

static void send_h2_flight_mode(const char *name, Handset *handset)
{
    CRSF_MK_FRAME_T(crsf_flight_mode_t)
    crsffm = {0};
    snprintf(crsffm.p.flight_mode, sizeof(crsffm.p.flight_mode), "H2:%.12s", name);
    CRSF::SetHeaderAndCrc((uint8_t *)&crsffm, CRSF_FRAMETYPE_FLIGHT_MODE, CRSF_FRAME_SIZE(sizeof(crsffm)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    handset->sendTelemetryToTX((uint8_t *)&crsffm);
}
#endif

void convert_mavlink_to_crsf_telem(uint8_t *CRSFinBuffer, uint8_t count, Handset *handset)
{
#if !defined(PLATFORM_STM32)
    // Store the relative altitude for GPS altitude
    static int32_t relative_alt = 0;
    static bool h2_value_seen = false;
    static int16_t h2_vspd_value = 0;
    static uint16_t h2_gps_altitude = 1000;
    static char h2_debug_name[16] = "";

    for (uint8_t i = 0; i < count; i++)
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        bool have_message = mavlink_frame_char(MAVLINK_COMM_0, CRSFinBuffer[CRSF_FRAME_NOT_COUNTED_BYTES + i], &msg, &status);
        // convert mavlink messages to CRSF messages
        if (have_message)
        {
            if (msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT)
            {
                mavlink_named_value_float_t named_value_float;
                mavlink_msg_named_value_float_decode(&msg, &named_value_float);
                char named_value_name[sizeof(named_value_float.name) + 1];
                memcpy(named_value_name, named_value_float.name, sizeof(named_value_float.name));
                named_value_name[sizeof(named_value_float.name)] = '\0';

                if (mavlink_name_equals_ignore_case(named_value_float.name, "h2", 2, sizeof(named_value_float.name)) ||
                    mavlink_name_equals_ignore_case(named_value_float.name, "MAV_H2", 6, sizeof(named_value_float.name)))
                {
                    h2_value_seen = true;
                    snprintf(h2_debug_name, sizeof(h2_debug_name), "H2:%.12s", named_value_name);
                    h2_vspd_value = (int16_t)(named_value_float.value * 10.0f);
                    h2_gps_altitude = crsf_gps_altitude_from_h2(named_value_float.value);

                    send_h2_vario(h2_vspd_value, handset);
                    send_h2_flight_mode(named_value_name, handset);
                }
            }

            // Only parse heartbeats from the autopilot (not GCS)
            if (msg.compid != MAV_COMP_ID_AUTOPILOT1)
            {
                continue;
            }
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_BATTERY_STATUS: {
                mavlink_battery_status_t battery_status;
                mavlink_msg_battery_status_decode(&msg, &battery_status);
                if (battery_status.id != 0) {
                    break;
                }
                CRSF_MK_FRAME_T(crsf_sensor_battery_t)
                crsfbatt = {0};
                // mV -> mv*100
                crsfbatt.p.voltage = htobe16(battery_status.voltages[0] / 100);
                // cA -> mA*100
                crsfbatt.p.current = htobe16(battery_status.current_battery / 10);
                crsfbatt.p.capacity = htobe32(battery_status.current_consumed) & 0x0FFF;
                // Temporary placeholder to expose a predictable Fuel value in EdgeTX.
                crsfbatt.p.remaining = 69;
                CRSF::SetHeaderAndCrc((uint8_t *)&crsfbatt, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
                handset->sendTelemetryToTX((uint8_t *)&crsfbatt);
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                mavlink_gps_raw_int_t gps_int;
                mavlink_msg_gps_raw_int_decode(&msg, &gps_int);
                CRSF_MK_FRAME_T(crsf_sensor_gps_t)
                crsfgps = {0};
// We use altitude relative to home for GPS altitude, by default, but we can also use GPS altitude if USE_MAVLINK_GPS_ALTITUDE is defined
#if defined(USE_MAVLINK_GPS_ALTITUDE)
                // mm -> meters + 1000
                crsfgps.p.altitude = htobe16(gps_int.alt / 1000 + 1000);
#else
                crsfgps.p.altitude = htobe16(h2_value_seen ? h2_gps_altitude : (uint16_t)(relative_alt / 1000 + 1000));
#endif
                // cm/s -> km/h / 10
                crsfgps.p.groundspeed = htobe16(gps_int.vel * 36 / 100);
                crsfgps.p.latitude = htobe32(gps_int.lat);
                crsfgps.p.longitude = htobe32(gps_int.lon);
                crsfgps.p.gps_heading = htobe16(gps_int.cog);
                crsfgps.p.satellites_in_use = gps_int.satellites_visible;
                CRSF::SetHeaderAndCrc((uint8_t *)&crsfgps, CRSF_FRAMETYPE_GPS, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
                handset->sendTelemetryToTX((uint8_t *)&crsfgps);
                break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                mavlink_global_position_int_t global_pos;
                mavlink_msg_global_position_int_decode(&msg, &global_pos);
                CRSF_MK_FRAME_T(crsf_sensor_vario_t)
                crsfvario = {0};
                // store relative altitude for GPS Alt so we don't have 2 Alt sensors
                relative_alt = global_pos.relative_alt;
                crsfvario.p.verticalspd = htobe16(h2_value_seen ? h2_vspd_value : -global_pos.vz); // MAVLink vz is positive down
                CRSF::SetHeaderAndCrc((uint8_t *)&crsfvario, CRSF_FRAMETYPE_VARIO, CRSF_FRAME_SIZE(sizeof(crsf_sensor_vario_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
                handset->sendTelemetryToTX((uint8_t *)&crsfvario);
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
                crsfatt = {0};
                crsfatt.p.pitch = htobe16(attitude.pitch * 10000); // in Betaflight & INAV, CRSF positive pitch is nose down, but in Ardupilot, it's nose up - we follow Ardupilot
                crsfatt.p.roll = htobe16(attitude.roll * 10000);
                crsfatt.p.yaw = htobe16(attitude.yaw * 10000);
                CRSF::SetHeaderAndCrc((uint8_t *)&crsfatt, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
                handset->sendTelemetryToTX((uint8_t *)&crsfatt);
                break;
            }
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                CRSF_MK_FRAME_T(crsf_flight_mode_t)
                crsffm = {0};
                if (h2_debug_name[0] != '\0')
                {
                    snprintf(crsffm.p.flight_mode, sizeof(crsffm.p.flight_mode), "%s", h2_debug_name);
                }
                else
                {
                    ap_flight_mode_name4(crsffm.p.flight_mode, ap_vehicle_from_mavtype(heartbeat.type), heartbeat.custom_mode);
                }
                // if we have a good flight mode, and we're armed, suffix the flight mode with a * - see Ardupilot's AP_CRSF_Telem::calc_flight_mode()
                size_t len = strnlen(crsffm.p.flight_mode, sizeof(crsffm.p.flight_mode));
                if (h2_debug_name[0] == '\0' && len > 0 && (len + 1 < sizeof(crsffm.p.flight_mode)) && (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
                    crsffm.p.flight_mode[len] = '*';
                    crsffm.p.flight_mode[len + 1] = '\0';
                }
                CRSF::SetHeaderAndCrc((uint8_t *)&crsffm, CRSF_FRAMETYPE_FLIGHT_MODE, CRSF_FRAME_SIZE(sizeof(crsffm)), CRSF_ADDRESS_CRSF_TRANSMITTER);
                handset->sendTelemetryToTX((uint8_t *)&crsffm);
                break;
            }
            }
        }
    }
#endif
}

bool isThisAMavPacket(uint8_t *buffer, uint16_t bufferSize)
{
#if !defined(PLATFORM_STM32)
    for (uint8_t i = 0; i < bufferSize; ++i)
    {
        uint8_t c = buffer[i];

        mavlink_message_t msg;
        mavlink_status_t status;

        // Try parse a mavlink message
        if (mavlink_frame_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            // Message decoded successfully
            return true;
        }
    }
#endif
    return false;
}

uint16_t buildMAVLinkELRSModeChange(uint8_t mode, uint8_t *buffer)
{
#if !defined(PLATFORM_STM32)
    constexpr uint8_t ELRS_MODE_CHANGE = 0x8;
    mavlink_command_int_t commandMsg;
    commandMsg.target_system = 255;
    commandMsg.target_component = MAV_COMP_ID_UDP_BRIDGE;
    commandMsg.command = MAV_CMD_USER_1;
    commandMsg.param1 = ELRS_MODE_CHANGE;
    commandMsg.param2 = mode;
    mavlink_message_t msg;
    mavlink_msg_command_int_encode(255, MAV_COMP_ID_TELEMETRY_RADIO, &msg, &commandMsg);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return len;
#else
    return 0;
#endif
}
