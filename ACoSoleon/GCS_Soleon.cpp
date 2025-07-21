#include "GCS_Soleon.h"

#include "Soleon.h"

uint8_t GCS_Soleon::sysid_this_mav() const
{
    return soleon.g.sysid_this_mav;
}

const char* GCS_Soleon::frame_string() const
{
//    if (soleon.motors == nullptr) {
//        return "MultiCopter";
//    }
//    return soleon.motors->get_frame_string();
    return "SO-PayLoad";
}

bool GCS_Soleon::simple_input_active() const
{
    return soleon.simple_mode == Soleon::SimpleMode::SIMPLE;
}

bool GCS_Soleon::supersimple_input_active() const
{
    return soleon.simple_mode == Soleon::SimpleMode::SUPERSIMPLE;
}

void GCS_Soleon::update_vehicle_sensor_status_flags(void)
{
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    const Soleon::ap_t &ap = soleon.ap;

    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (ap.rc_receiver_present && !soleon.failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    // update flightmode-specific flags:
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;


    // optional sensors, some of which are essentially always
    // available in the firmware:
#if HAL_PROXIMITY_ENABLED
    if (soleon.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (soleon.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (!soleon.g2.proximity.sensor_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (soleon.rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

#if AC_PRECLAND_ENABLED
    if (soleon.precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (soleon.precland.enabled() && soleon.precland.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif

#if AP_TERRAIN_AVAILABLE
    switch (soleon.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in soleon
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    // only mark propulsion healthy if all of the motors are producing
    // nominal thrust
    // if (!soleon.motors->get_thrust_boost()) {
    //     control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    // }
}
