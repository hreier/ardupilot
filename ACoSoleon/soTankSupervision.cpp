#include "GCS_Mavlink.h"

#include "soTankSupervision.h"

SO_TankSupervision::SO_TankSupervision()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many TankSupervisions");
#endif
        return;
    }
    init(false);
    _singleton = this;
    // To-Do: ensure that the pump and spinner servo channels are enabled
}


/*
 * Init and run calls for brake flight mode
 */
// brake_init - initialise brake controller
void SO_TankSupervision::set(float val)
{
    _delta_fill = val;
}


// brake_init - initialise brake controller
bool SO_TankSupervision::init(bool ignore_checks)
{
    _fill_level = 100.0f;
    _delta_fill = 0.0f;

  /*  // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);

    // initialise position controller
    pos_control->init_xy_controller();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    _timeout_ms = 0;
*/
    return true;
}

void SO_TankSupervision::update()
{
    _fill_level -= _delta_fill;
    if (_fill_level < 0.0f) _fill_level = 100.0f;
};

// brake_run - runs the brake controller
// should be called at 100hz or more
void SO_TankSupervision::run()
{
;
 /*   // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // use position controller to stop
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    // MAV_CMD_SOLO_BTN_PAUSE_CLICK (Solo only) is used to set the timeout.
    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    } */
}

/*
void SO_TankSupervision::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}*/


SO_TankSupervision *SO_TankSupervision::_singleton;

namespace SO {
SO_TankSupervision *TankSupervision()
{
    return SO_TankSupervision::get_singleton();
}
}

