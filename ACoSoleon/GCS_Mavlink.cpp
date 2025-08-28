#include "Soleon.h"

#include "GCS_Mavlink.h"
#include <AP_RPM/AP_RPM_config.h>
#include <AP_EFI/AP_EFI_config.h>

// ---- tunnel payload types
#define SO_PLTYPE_COMMAND       0xF000
#define SO_PLTYPE_COMMAND_RESP  0xF001
#define SO_PLTYPE_INTVAL_F010   0xF010     //-- from payload send after interval request ---getTunnelPayloadtype(); may be to made dynamic)

// ---- Soleon commands (SO_PLTYPE_COMMAND)
#define SO_PLCMD_RES_ALL        1       //-- Reset all
#define SO_PLCMD_RES_CNTRS      2       //-- Reset counters
#define SO_PLCMD_RES_CONTROLLER 3       //-- Reset controller
#define SO_PLCMD_RES_ERRORS     4       //-- Reset Error counters + flags
#define SO_PLCMD_RES_MAVLINK    5       //-- Reset mavlink diagnose 


MAV_TYPE GCS_Soleon::frame_type() const
{
    /*
      for GCS don't give MAV_TYPE_GENERIC as the GCS would have no
      information and won't display UIs such as flight mode
      selection
    */
//    const MAV_TYPE mav_type_default = MAV_TYPE_HELICOPTER; 
    const MAV_TYPE mav_type_default = MAV_TYPE_QUADROTOR;
//    const MAV_TYPE mav_type_default = MAV_TYPE_GENERIC;  //--- Note: If set the so-parameter texts/descriptions not shown in Missionplanner!!!
    
    return mav_type_default;
}

MAV_MODE GCS_MAVLINK_Soleon::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Soleon::custom_mode() const
{
    return 0;
}

MAV_STATE GCS_MAVLINK_Soleon::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (soleon.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (soleon.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}


void GCS_MAVLINK_Soleon::send_attitude_target()
{

}

void GCS_MAVLINK_Soleon::send_position_target_global_int()
{
    Location target;


    // convert altitude frame to AMSL (this may use the terrain database)
    if (!target.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return;
    }
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        target.lat, // latitude as 1e7
        target.lng, // longitude as 1e7
        target.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}

void GCS_MAVLINK_Soleon::send_position_target_local_ned()
{
#if MODE_GUIDED_ENABLED == ENABLED
    if (!soleon.flightmode->in_guided_mode()) {
        return;
    }

    const ModeGuided::SubMode guided_mode = soleon.mode_guided.submode();
    Vector3f target_pos;
    Vector3f target_vel;
    Vector3f target_accel;
    uint16_t type_mask = 0;

    switch (guided_mode) {
    case ModeGuided::SubMode::Angle:
        // we don't have a local target when in angle mode
        return;
    case ModeGuided::SubMode::TakeOff:
    case ModeGuided::SubMode::WP:
    case ModeGuided::SubMode::Pos:
        type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position
        target_pos = soleon.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        break;
    case ModeGuided::SubMode::PosVelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position, velocity & acceleration
        target_pos = soleon.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        target_vel = soleon.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = soleon.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::VelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_vel = soleon.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = soleon.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::Accel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_accel = soleon.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(), // time boot ms
        MAV_FRAME_LOCAL_NED, 
        type_mask,
        target_pos.x,   // x in metres
        target_pos.y,   // y in metres
        -target_pos.z,  // z in metres NED frame
        target_vel.x,   // vx in m/s
        target_vel.y,   // vy in m/s
        -target_vel.z,  // vz in m/s NED frame
        target_accel.x, // afx in m/s/s
        target_accel.y, // afy in m/s/s
        -target_accel.z,// afz in m/s/s NED frame
        0.0f, // yaw
        0.0f); // yaw_rate
#endif
}

//-- sends the status of the Soleon AirController
//int temp;
void GCS_MAVLINK_Soleon::send_so_status(void)
{
    so_tunnel_f010.cntrTxStatus++;

    mavlink_msg_so_status_send(chan,
                                AP_HAL::millis(), 
                                soleon.soleon_ctrl_mode->_mp_status,  
                                soleon.soleon_ctrl_mode->_fill_level,
                                soleon.soleon_ctrl_mode->_mp_sprayrate,
                                soleon.soleon_ctrl_mode->_mp_liter_ha,
                                soleon.soleon_ctrl_mode->_mp_line_dist,
                                soleon.soleon_ctrl_mode->_mp_planned_spd);

    
}

void GCS_MAVLINK_Soleon::send_so_tunnel(void) 
{
    unsigned long size = sizeof(so_tunnel_f010);
    
    update_so_tunnel_f010();  //-- update the so_tunnel_f010 data

//    mav_array_memcpy((void*)tunnel_buf, (void*)so_tunnel_f010, size);
    
    _mav_put_uint8_t_array((char *)tunnel_buf, 0, so_tunnel_f010.buf, size);
    

    mavlink_msg_tunnel_send(chan, 
                   mavlink_system.sysid, 
                   mavlink_system.compid, 
                   SO_PLTYPE_INTVAL_F010,  // <---- getTunnelPayloadtype(); may be to made dynamic
                   size, 
                   tunnel_buf);

    
    //gcs().send_text(MAV_SEVERITY_WARNING, "Debugging: tank level = %fl",  soleon.soleon_ctrl_mode->_fill_level);  //-- debugging

}

void GCS_MAVLINK_Soleon::send_nav_controller_output() const
{

}

float GCS_MAVLINK_Soleon::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    // airspeed sensors are best. While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed. When reporting we should send the true airspeed
    // value if possible:
    if (soleon.airspeed.enabled() && soleon.airspeed.healthy()) {
        return soleon.airspeed.get_airspeed();
    }
#endif
    
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // we are running the EKF3 wind estimation code which can give
        // us an airspeed estimate
        return airspeed_vec_bf.length();
    }
    return AP::gps().ground_speed();
}

int16_t GCS_MAVLINK_Soleon::vfr_hud_throttle() const
{
    // if (soleon.motors == nullptr) {
    //     return 0;
    // }
    // return (int16_t)(soleon.motors->get_throttle() * 100);

    return 0;
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Soleon::send_pid_tuning()
{

}

// send winch status message
void GCS_MAVLINK_Soleon::send_winch_status() const
{
 ;
}

uint8_t GCS_MAVLINK_Soleon::sysid_my_gcs() const
{
    return soleon.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Soleon::sysid_enforce() const
{
    return soleon.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Soleon::telem_delay() const
{
    return (uint32_t)(soleon.g.telem_delay);
}

bool GCS_Soleon::vehicle_initialised() const {
    return soleon.ap.initialised;
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Soleon::try_send_message(enum ap_message id)
{
    switch(id) {
    //--- make the Payload to be quite
    case MSG_AHRS:
    case MSG_ATTITUDE:
    case MSG_BATTERY_STATUS:
    case MSG_GPS_RAW:
    case MSG_POWER_STATUS:
    case MSG_RAW_IMU:
    case MSG_SCALED_IMU2:
    case MSG_SCALED_IMU3:
    case MSG_SCALED_PRESSURE:
    case MSG_SCALED_PRESSURE2:
    case MSG_VFR_HUD:
    case MSG_VIBRATION:
//    case MSG_SIM_STATE: 
    case     MSG_AHRS2:

    //----
    case MSG_POSITION_TARGET_GLOBAL_INT:
    case MSG_TERRAIN:
    case MSG_ADSB_VEHICLE:
    case MSG_WIND:
    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
    case MSG_FENCE_STATUS:
    case MSG_OPTICAL_FLOW:

    // --- the next 2 messages should be enabled/disabled via parameter for diagnosis (currently enabled)!!!
//    case MSG_RC_CHANNELS:   //---HaRe ToDo: this sends message with 2Hz; should be activated only if configured
//    case MSG_SERVO_OUTPUT_RAW:   //---ToDo: this sends message with 2Hz; should be activated only if configured
        // unused
       break;

    case MSG_SO_STATUS:
        send_so_status();
        break;

    case MSG_SO_TUNNEL:
        send_so_tunnel();
        break;



    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: MAVLink Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, and SCALED_PRESSURE3
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate
    // @Description: MAVLink Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW_INT (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, FENCE_STATUS, and GLOBAL_TARGET_POS_INT
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate
    // @Description: MAVLink Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Unused
    // @Description: Unused
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate
    // @Description: MAVLink Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate
    // @Description: MAVLink Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2 and PID_TUNING
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate
    // @Description: MAVLink Stream rate of VFR_HUD
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate
    // @Description: MAVLink Stream rate of AHRS, SYSTEM_TIME, WIND, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY_STATUS, GIMBAL_DEVICE_ATTITUDE_STATUS, OPTICAL_FLOW, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION, RPM, ESC TELEMETRY,GENERATOR_STATUS, and WINCH_STATUS

    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate
    // @Description: MAVLink Stream rate of PARAM_VALUE
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  0),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate
    // @Description: MAVLink ADSB stream rate
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK_Parameters, streamRates[9],  0),
AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MCU_STATUS,
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT, // MISSION_CURRENT
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_POSITION_TARGET_GLOBAL_INT,
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
#if AP_SIM_ENABLED
    MSG_SIMSTATE,
#endif
    MSG_AHRS2,
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_SYSTEM_TIME,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN,
#endif
    MSG_BATTERY_STATUS,
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
#if AP_RPM_ENABLED
    MSG_RPM,
#endif
    MSG_ESC_TELEMETRY,
    MSG_GENERATOR_STATUS,
    MSG_WINCH_STATUS,
#if HAL_EFI_ENABLED
    MSG_EFI_STATUS,
#endif
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE,
    MSG_AIS_VESSEL,
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

MISSION_STATE GCS_MAVLINK_Soleon::mission_state(const class AP_Mission &mission) const
{
    return GCS_MAVLINK::mission_state(mission);
}

bool GCS_MAVLINK_Soleon::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
#if MODE_AUTO_ENABLED == ENABLED
    return soleon.mode_auto.do_guided(cmd);
#else
    return false;
#endif
}

void GCS_MAVLINK_Soleon::packetReceived(const mavlink_status_t &status,
                                        const mavlink_message_t &msg)
{
    // we handle these messages here to avoid them being blocked by mavlink routing code

#if MODE_FOLLOW_ENABLED == ENABLED
    // pass message to follow library
    soleon.g2.follow.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Soleon::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return soleon.ap.initialised_params;
}

void GCS_MAVLINK_Soleon::send_banner()
{
    GCS_MAVLINK::send_banner();
}

void GCS_MAVLINK_Soleon::handle_command_ack(const mavlink_message_t &msg)
{
    soleon.command_ack_counter++;
    GCS_MAVLINK::handle_command_ack(msg);
}

/*
  handle a LANDING_TARGET command. The timestamp has been jitter corrected
*/
void GCS_MAVLINK_Soleon::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if AC_PRECLAND_ENABLED
    soleon.precland.handle_msg(packet, timestamp_ms);
#endif
}

MAV_RESULT GCS_MAVLINK_Soleon::_handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    /*Hare
    if (is_equal(packet.param6,1.0f)) {
        // compassmot calibration
        return soleon.mavlink_compassmot(*this);
    }*/

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet, msg);
}


MAV_RESULT GCS_MAVLINK_Soleon::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Soleon::handle_preflight_reboot(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    // reject reboot if user has also specified they want the "Auto" ESC calibration on next reboot
    if (soleon.g.esc_calibrate == (uint8_t)Soleon::ESCCalibrationModes::ESCCAL_AUTO) {
        send_text(MAV_SEVERITY_CRITICAL, "Reboot rejected, ESC cal on reboot");
        return MAV_RESULT_FAILED;
    }

    // call parent
    return GCS_MAVLINK::handle_preflight_reboot(packet, msg);
}

bool GCS_MAVLINK_Soleon::set_home_to_current_location(bool _lock) {
    return soleon.set_home_to_current_location(_lock);
}
bool GCS_MAVLINK_Soleon::set_home(const Location& loc, bool _lock) {
    return soleon.set_home(loc, _lock);
}

MAV_RESULT GCS_MAVLINK_Soleon::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
#if MODE_GUIDED_ENABLED == ENABLED
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!soleon.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(soleon.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // we need to do this first, as we don't want to change the flight mode unless we can also set the target
    if (!soleon.mode_guided.set_destination(request_location, false, 0, false, 0)) {
        return MAV_RESULT_FAILED;
    }

    if (!soleon.flightmode->in_guided_mode()) {
        if (!soleon.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!soleon.mode_guided.set_destination(request_location, false, 0, false, 0)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
#else
    return MAV_RESULT_UNSUPPORTED;
#endif
}

MAV_RESULT GCS_MAVLINK_Soleon::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{

    switch(packet.command) {
#if MODE_FOLLOW_ENABLED == ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            soleon.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_DENIED;
#endif

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    // pause or resume an auto mission
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return handle_command_pause_continue(packet);

    // script messages 
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        switch ((uint8_t)packet.param1){ //--process the selector
            case 0: //--HaRe only for test - remove it
                soleon.soleon_ctrl_mode->_delta_fill = packet.param2;
                gcs().send_text(MAV_SEVERITY_INFO, "SprayRateScript = %f", packet.param2);  ///-HaRe debug
                break;

            case 1: //-- mission plan startup command/configuration
                soleon.soleon_ctrl_mode->_mp_liter_ha = packet.param2;
                soleon.soleon_ctrl_mode->_mp_line_dist = packet.param3;
                soleon.soleon_ctrl_mode->_mp_planned_spd = packet.param4;
                break;
            
            case 2: //-- mission plan command
                soleon.soleon_ctrl_mode->_mp_cmd  = (u_int8_t) packet.param2;
                soleon.soleon_ctrl_mode->_mp_dist_waypoint = packet.param3;  
                break;
            
            default:
                gcs().send_text(MAV_SEVERITY_WARNING, "DO_SEND_SCRIPT_MESSAGE wrong selector [%d] ", (uint8_t) packet.param1);  ///-HaRe debug
                break;
        }
        so_tunnel_f010.cntrRxMp++;
        return MAV_RESULT_ACCEPTED;
        
    case MAV_CMD_SO_SYSMODE:  // Soleon Sysmode
        soleon.soleon_ctrl_mode->_delta_fill = packet.param1;
        gcs().send_text(MAV_SEVERITY_INFO, "SprayRate = %f", packet.param1);  ///-HaRe debug
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_REQUEST_MESSAGE:
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        if ((uint32_t)packet.param1 == MAVLINK_MSG_ID_TUNNEL)
        {  //-- set interval for TUNNEL arrived --> store/update the payload type (param3) for tunnel handler
           // updateTunnelPayloadtype(packet.param3); 
           gcs().send_text(MAV_SEVERITY_WARNING, "Debugging: SET_MSG_INTERVAL %d %x", (int)packet.param2, (unsigned int)packet.param3);  //-- debugging 
        }
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);


    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

#if HAL_MOUNT_ENABLED
MAV_RESULT GCS_MAVLINK_Soleon::handle_command_mount(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_DO_MOUNT_CONTROL:

        break;
    default:
        break;
    }
    return GCS_MAVLINK::handle_command_mount(packet, msg);
}
#endif



MAV_RESULT GCS_MAVLINK_Soleon::handle_command_pause_continue(const mavlink_command_int_t &packet)
{

    return MAV_RESULT_DENIED;
}

#if HAL_MOUNT_ENABLED
void GCS_MAVLINK_Soleon::handle_mount_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if ((soleon.camera_mount.get_mount_type() != AP_Mount::Type::None) &&
            !soleon.camera_mount.has_pan_control()) {
            /*Hare 
            soleon.flightmode->auto_yaw.set_yaw_angle_rate(
                mavlink_msg_mount_control_get_input_c(&msg) * 0.01f,
                0.0f);
            */

            break;
        }
    }
    GCS_MAVLINK::handle_mount_message(msg);
}
#endif

// this is called on receipt of a MANUAL_CONTROL packet and is
// expected to call manual_override to override RC input on desired
// axes.
void GCS_MAVLINK_Soleon::handle_manual_control_axes(const mavlink_manual_control_t &packet, const uint32_t tnow)
{
    if (packet.z < 0) { // Copter doesn't do negative thrust
        return;
    }

    /*manual_override(soleon.channel_roll, packet.y, 1000, 2000, tnow);
    manual_override(soleon.channel_pitch, packet.x, 1000, 2000, tnow, true);
    manual_override(soleon.channel_throttle, packet.z, 0, 1000, tnow);
    manual_override(soleon.channel_yaw, packet.r, 1000, 2000, tnow);*/
}

// sanity check velocity or acceleration vector components are numbers
// (e.g. not NaN) and below 1000. vec argument units are in meters/second or
// metres/second/second
bool GCS_MAVLINK_Soleon::sane_vel_or_acc_vector(const Vector3f &vec) const
{
    for (uint8_t i=0; i<3; i++) {
        // consider velocity invalid if any component nan or >1000(m/s or m/s/s)
        if (isnan(vec[i]) || fabsf(vec[i]) > 1000) {
            return false;
        }
    }
    return true;
}


void GCS_MAVLINK_Soleon::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_TUNNEL:
    {
        handle_msg_tunnel(msg);
        ///--- do bin i  --->>>> reset für counters einbauen (alle resets)...
        //gcs().send_text(MAV_SEVERITY_WARNING, "Debugging: MAVLINK_MSG_ID_TUNNEL came in");  //-- debugging
        break;
    }

    case MAVLINK_MSG_ID_VFR_HUD:           // MAV ID: 74
    {
        handle_copter_hud_msg(msg, soleon.should_log(MASK_LOG_PM));  //-- ToDo: logging
        break;
    }

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, soleon.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        soleon.terrain.handle_data(chan, msg);
#endif
        break;

#if TOY_MODE_ENABLED == ENABLED
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        soleon.g2.toy_mode.handle_message(msg);
        break;
#endif
    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }     // end switch
} // end handle mavlink


void GCS_MAVLINK_Soleon::handle_msg_tunnel(const mavlink_message_t &msg)
{
    mavlink_tunnel_t packet;

    mavlink_msg_tunnel_decode(&msg, &packet);

    gcs().send_text(MAV_SEVERITY_WARNING, "MAVLINK_MSG_ID_TUNNEL -- pltype=%x plbyte=%d", packet.payload_type, packet.payload[0]);  //-- debugging

    if (packet.payload_type != SO_PLTYPE_COMMAND) return;     //--- payload type not matching

    switch ((int)packet.payload[0])
    {
    case SO_PLCMD_RES_CNTRS:
        soleon.clearSoleonCtrlCntr();
        so_tunnel_f010.cntrRxMp = 0;
        so_tunnel_f010.cntrTxStatus= 0;    
        break;

    case SO_PLCMD_RES_CONTROLLER:
        so_tunnel_f010.ppmPumpLeft = 0;
        so_tunnel_f010.ppmPumpRight = 0; 
        so_tunnel_f010.ppmPumpLeftMax = 0;
        so_tunnel_f010.ppmPumpRightMax = 0; 
        so_tunnel_f010.eValLeftMax = 0;
        so_tunnel_f010.eValRightMax = 0;
        break;


    case SO_PLCMD_RES_ERRORS:
        so_tunnel_f010.errorFlags = 0;
        so_tunnel_f010.cntPressLeftWindups = 0; 
        so_tunnel_f010.cntPressRightWindups = 0;
        so_tunnel_f010.cntMavLinkErrors = 0; 
        break;

    case SO_PLCMD_RES_MAVLINK:
        mavlink_reset_channel_status(0);
        mavlink_reset_channel_status(1);
        so_tunnel_f010.dbg3Uint32++;
        break;

    }
    
}




MAV_RESULT GCS_MAVLINK_Soleon::handle_flight_termination(const mavlink_command_int_t &packet) {


    return MAV_RESULT_ACCEPTED; //-HaRe
}

float GCS_MAVLINK_Soleon::vfr_hud_alt() const
{
    if (soleon.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatibility option for older mavlink-aware devices that
        // assume Copter returns a relative altitude in VFR_HUD.alt
        return soleon.current_loc.alt * 0.01f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

uint64_t GCS_MAVLINK_Soleon::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE
            (soleon.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}

MAV_LANDED_STATE GCS_MAVLINK_Soleon::landed_state() const
{
    if (soleon.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }

    return MAV_LANDED_STATE_IN_AIR;
}


#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Soleon::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    //return units are m
    if (soleon.ap.initialised) {
        return 0.01 * (global_position_current.alt + soleon.pos_control->get_pos_error_z_cm());
    }
    return 0;
    
}

uint8_t GCS_MAVLINK_Soleon::high_latency_tgt_heading() const
{

    return 0;     
}
    
uint16_t GCS_MAVLINK_Soleon::high_latency_tgt_dist() const
{

    return 0;
}

uint8_t GCS_MAVLINK_Soleon::high_latency_tgt_airspeed() const
{
    if (soleon.ap.initialised) {
        // return units are m/s*5
        return MIN(soleon.pos_control->get_vel_target_cms().length() * 5.0e-2, UINT8_MAX);
    }
    return 0;  
}

uint8_t GCS_MAVLINK_Soleon::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are m/s*5
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        return wind.length() * 5;
    }
    return 0; 
}

uint8_t GCS_MAVLINK_Soleon::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are deg/2
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        // need to convert -180->180 to 0->360/2
        return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED

void GCS_MAVLINK_Soleon::handle_copter_hud_msg(const mavlink_message_t &msg, bool log_hud)
{
    static int test_cnt;
    float my_speed;
    mavlink_vfr_hud_t packet;
  
    mavlink_msg_vfr_hud_decode(&msg, &packet);
    my_speed = norm(packet.groundspeed, packet.climb);

    if (test_cnt++ > 4){
        test_cnt = 0;
        //gcs().send_text(MAV_SEVERITY_INFO, "Mavlink msgid: %d; sysid: %d; compid: %d; ang: %d", msg.msgid, msg.sysid, msg.compid, packet.heading);   //--debug
     //HaRe: disabled for the moment!!!!!!!   gcs().send_text(MAV_SEVERITY_WARNING, "as=%0.2f gs=%0.2f cl=%0.2f ms=%0.2f an=%d", packet.airspeed, packet.groundspeed, packet.climb ,my_speed ,packet.heading );   //--debug
       my_speed=my_speed*test_cnt;  //- to make the compiler passing
    }
    
}

//-- this fills and updates the datastructure before sending this via tunnel to missionplanner 'soleon service' plugin
/// https://github.com/mavlink/c_library_v2/blob/master/mavlink_helpers.h
bool GCS_MAVLINK_Soleon::update_so_tunnel_f010(void) 
{
    mavlink_status_t * pt_mavlink_status;

    pt_mavlink_status = mavlink_get_channel_status(0);
    so_tunnel_f010.buffer_overrun_0 = pt_mavlink_status->buffer_overrun;
    so_tunnel_f010.msg_received_0 = pt_mavlink_status->msg_received;
    so_tunnel_f010.packet_rx_drop_count_0 = pt_mavlink_status->packet_rx_drop_count;
    so_tunnel_f010.packet_rx_success_count_0 = pt_mavlink_status->packet_rx_success_count;
    so_tunnel_f010.parse_error_0 = pt_mavlink_status->parse_error;
    
    pt_mavlink_status = mavlink_get_channel_status(1);
    so_tunnel_f010.buffer_overrun_1 = pt_mavlink_status->buffer_overrun;
    so_tunnel_f010.msg_received_1 = pt_mavlink_status->msg_received;
    so_tunnel_f010.packet_rx_drop_count_1 = pt_mavlink_status->packet_rx_drop_count;
    so_tunnel_f010.packet_rx_success_count_1 = pt_mavlink_status->packet_rx_success_count;
    so_tunnel_f010.parse_error_1 = pt_mavlink_status->parse_error;
    
    so_tunnel_f010.timestamp = AP_HAL::millis();
    so_tunnel_f010.cntrCtrLoops=soleon.getSoleonCtrlCntr();  //clearSoleonCtrlCntr
//    so_tunnel_f010.cntrRxMp=21;  
//    so_tunnel_f010.cntrTxStatus=22;
//    so_tunnel_f010.dbg1Float=30;
//    so_tunnel_f010.dbg2Float=31;
//    so_tunnel_f010.dbg3Uint32=32;
//    so_tunnel_f010.dbg4Uint32=33;

    so_tunnel_f010.cntMavLinkErrors = so_tunnel_f010.packet_rx_drop_count_1 + so_tunnel_f010.packet_rx_drop_count_0;

    so_tunnel_f010.offsetTrim = soleon.soleon_ctrl_mode->getOffsetTrim();
    so_tunnel_f010.owerRideOnSw = (uint8_t) soleon.channel_on_mode->get_aux_switch_pos(); 
    so_tunnel_f010.owerRideSw = (uint8_t) soleon.channel_override->get_aux_switch_pos(); 
    so_tunnel_f010.pressureLeft=11.12;
    so_tunnel_f010.pressureRight=soleon.g2.so_press.getPressure(0);  //--- PressureValue 
    so_tunnel_f010.errorFlags=0xFE31FE31;

    so_tunnel_f010.cntPressLeftWindups=30;
    so_tunnel_f010.cntPressRightWindups=31;
    so_tunnel_f010.eValLeftMax=32;
    so_tunnel_f010.eValRightMax=33;
    so_tunnel_f010.ppmPumpLeft = soleon.soleon_ctrl_mode->getPumpPPMleft();
    so_tunnel_f010.ppmPumpLeftMax=71;
    so_tunnel_f010.ppmPumpRight = soleon.soleon_ctrl_mode->getPumpPPMright();
    so_tunnel_f010.ppmPumpRightMax=73;

    return true;
}; 


