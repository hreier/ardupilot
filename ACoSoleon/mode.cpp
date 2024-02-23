#include "Soleon.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(soleon.g),
    g2(soleon.g2),
    wp_nav(soleon.wp_nav),
    loiter_nav(soleon.loiter_nav),
    pos_control(soleon.pos_control),
    inertial_nav(soleon.inertial_nav),
    ahrs(soleon.ahrs),
    motors(soleon.motors),
    channel_roll(soleon.channel_roll),
    channel_pitch(soleon.channel_pitch),
    channel_throttle(soleon.channel_throttle),
    channel_yaw(soleon.channel_yaw),
    G_Dt(soleon.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Soleon::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {

#if MODE_FLOWHOLD_ENABLED == ENABLED
        case Mode::Number::FLOWHOLD:
            ret = (Mode *)g2.mode_flowhold_ptr;
            break;
#endif


        default:
            break;
    }

    return ret;
}


// called when an attempt to change into a mode is unsuccessful:
void Soleon::mode_change_failed(const Mode *mode, const char *reason)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    // make sad noise
    if (soleon.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

// Check if this mode can be entered from the GCS
bool Soleon::gcs_mode_enabled(const Mode::Number mode_num)
{
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::ALT_HOLD,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::DRIFT,
        (uint8_t)Mode::Number::SPORT,
        (uint8_t)Mode::Number::FLIP,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::POSHOLD,
        (uint8_t)Mode::Number::BRAKE,
        (uint8_t)Mode::Number::THROW,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED_NOGPS,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::FLOWHOLD,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::ZIGZAG,
        (uint8_t)Mode::Number::SYSTEMID,
        (uint8_t)Mode::Number::AUTOROTATE,
        (uint8_t)Mode::Number::AUTO_RTL,
        (uint8_t)Mode::Number::TURTLE
    };

    if (!block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list))) {
        return true;
    }

    // Mode disabled, try and grab a mode name to give a better warning.
    Mode *new_flightmode = mode_from_mode_num(mode_num);
    if (new_flightmode != nullptr) {
        mode_change_failed(new_flightmode, "GCS entry disabled (FLTMODE_GCSBLOCK)");
    } else {
        notify_no_such_mode((uint8_t)mode_num);
    }

    return false;
}

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Soleon::set_mode(Mode::Number mode, ModeReason reason)
{

    // return success
    return true;
}

bool Soleon::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && soleon.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return soleon.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Soleon::update_flight_mode()
{
   /*HaRe surface_tracking.invalidate_for_logging();  // invalidate surface tracking alt, flight mode will set to true if used

    flightmode->run();*/
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Soleon::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{

    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

    // perform cleanup required for each flight mode
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!old_flightmode->has_manual_throttle()){
        if (new_flightmode == &mode_stabilize){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_flightmode == &mode_acro){
            input_manager.set_stab_col_ramp(0.0);
        }
    }
#endif //HELI_FRAME
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Soleon::notify_flight_mode() {
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Mode::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // throttle failsafe check
    if (soleon.failsafe.radio || !soleon.ap.rc_receiver_present) {
        roll_out_cd = 0.0;
        pitch_out_cd = 0.0;
        return;
    }

    //transform pilot's normalised roll or pitch stick input into a roll and pitch euler angle command
    float roll_out_deg;
    float pitch_out_deg;
    rc_input_to_roll_pitch(channel_roll->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), channel_pitch->get_control_in()*(1.0/ROLL_PITCH_YAW_INPUT_MAX), angle_max_cd * 0.01,  angle_limit_cd * 0.01, roll_out_deg, pitch_out_deg);

    // Convert to centi-degrees
    roll_out_cd = roll_out_deg * 100.0;
    pitch_out_cd = pitch_out_deg * 100.0;
}

// transform pilot's roll or pitch input into a desired velocity
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    // throttle failsafe check
    if (soleon.failsafe.radio || !soleon.ap.rc_receiver_present) {
        return vel;
    }
    // fetch roll and pitch inputs
    float roll_out = channel_roll->get_control_in();
    float pitch_out = channel_pitch->get_control_in();

    // convert roll and pitch inputs to -1 to +1 range
    float scaler = 1.0 / (float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // convert roll and pitch inputs into velocity in NE frame
    vel = Vector2f(-pitch_out, roll_out);
    if (vel.is_zero()) {
        return vel;
    }
 
    // Transform square input range to circular output
    // vel_scaler is the vector to the edge of the +- 1.0 square in the direction of the current input
    Vector2f vel_scaler = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    // We scale the output by the ratio of the distance to the square to the unit circle and multiply by vel_max
    vel *= vel_max / vel_scaler.length();
    return vel;
}

bool Mode::_TakeOff::triggered(const float target_climb_rate) const
{
    if (!soleon.ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    if (soleon.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // hold aircraft on the ground until rotor speed runup has finished
        return false;
    }

    return true;
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !soleon.ap.auto_armed || soleon.ap.land_complete) {
        return true;
    }
    return false;
}

void Mode::zero_throttle_and_relax_ac(bool spool_up)
{

}

void Mode::zero_throttle_and_hold_attitude()
{

}

// handle situations where the vehicle is on the ground waiting for takeoff
// force_throttle_unlimited should be true in cases where we want to keep the motors spooled up
// (instead of spooling down to ground idle).  This is required for tradheli's in Guided and Auto
// where we always want the motor spooled up in Guided or Auto mode.  Tradheli's main rotor stops 
// when spooled down to ground idle.
// ultimately it forces the motor interlock to be obeyed in auto and guided modes when on the ground.
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{

}

/*
  get a height above ground estimate for landing
 */
int32_t Mode::get_alt_above_ground_cm(void)
{
    int32_t alt_above_ground_cm;
    if (soleon.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }
    if (!pos_control->is_active_xy()) {
        return soleon.current_loc.alt;
    }
    if (soleon.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }

    // Assume the Earth is flat:
    return soleon.current_loc.alt;
}

void Mode::land_run_vertical_control(bool pause_descent)
{
    float cmb_rate = 0;
    bool ignore_descent_limit = false;
    if (!pause_descent) {

        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = (MAX(g2.land_alt_low,100) > get_alt_above_ground_cm()) || soleon.ap.land_complete_maybe;

        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_max_speed_down_cms();
        }

        // Don't speed up for landing.
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if AC_PRECLAND_ENABLED
        const bool navigating = pos_control->is_active_xy();
        bool doing_precision_landing = !soleon.ap.land_repo_active && soleon.precland.target_acquired() && navigating;

        if (doing_precision_landing) {
            // prec landing is active
            Vector2f target_pos;
            float target_error_cm = 0.0f;
            if (soleon.precland.get_target_position_cm(target_pos)) {
                const Vector2f current_pos = inertial_nav.get_position_xy_cm();
                // target is this many cm away from the vehicle
                target_error_cm = (target_pos - current_pos).length();
            }
            // check if we should descend or not
            const float max_horiz_pos_error_cm = soleon.precland.get_max_xy_error_before_descending_cm();
            Vector3f target_pos_meas;
            soleon.precland.get_target_position_measurement_cm(target_pos_meas);
            if (target_error_cm > max_horiz_pos_error_cm && !is_zero(max_horiz_pos_error_cm)) {
                // doing precland but too far away from the obstacle
                // do not descend
                cmb_rate = 0.0f;
            } else if (target_pos_meas.z > 35.0f && target_pos_meas.z < 200.0f) {
                // very close to the ground and doing prec land, lets slow down to make sure we land on target
                // compute desired descent velocity
                const float precland_acceptable_error_cm = 15.0f;
                const float precland_min_descent_speed_cms = 10.0f;
                const float max_descent_speed_cms = abs(g.land_speed)*0.5f;
                const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent_speed_cms/precland_acceptable_error_cm));
                cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms+land_slowdown);
            }
        }
#endif
    }

    // update altitude target and call position controller
    pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_z_controller();
}

void Mode::land_run_horizontal_control()
{


}

// run normal or precision landing (if enabled)
// pause_descent is true if vehicle should not descend
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if AC_PRECLAND_ENABLED
    if (pause_descent || !soleon.precland.enabled()) {
        // we don't want to start descending immediately or prec land is disabled
        // in both cases just run simple land controllers
        land_run_horiz_and_vert_control(pause_descent);
    } else {
        // prec land is enabled and we have not paused descent
        // the state machine takes care of the entire prec landing procedure
        precland_run();
    }
#else
    land_run_horiz_and_vert_control(pause_descent);
#endif
}



float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Mode::get_pilot_desired_throttle() const
{
    const float thr_mid = throttle_hover();
    int16_t throttle_control = channel_throttle->get_control_in();

    int16_t mid_stick = soleon.get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    } else {
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }

    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

float Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
#if AC_AVOID_ENABLED == ENABLED
    AP::ac_avoid()->adjust_velocity_z(pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), target_rate, G_Dt);
    return target_rate;
#else
    return target_rate;
#endif
}

// send output to the motors, can be overridden by subclasses
void Mode::output_to_motors()
{
    motors->output();
}

Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
{
    // Alt Hold State Machine Determination
    if (!motors->armed()) {
        // the aircraft should moved to a shut down state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // transition through states as aircraft spools down
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            return AltHold_MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            return AltHold_Landed_Ground_Idle;

        default:
            return AltHold_Landed_Pre_Takeoff;
        }

    } else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
        // the aircraft is currently landed or taking off, asking for a positive climb rate and in THROTTLE_UNLIMITED
        // the aircraft should progress through the take off procedure
        return AltHold_Takeoff;

    } else if (!soleon.ap.auto_armed || soleon.ap.land_complete) {
        // the aircraft is armed and landed
        if (target_climb_rate_cms < 0.0f && !soleon.ap.using_interlock) {
            // the aircraft should move to a ground idle state
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // the aircraft should prepare for imminent take off
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // the aircraft is waiting in ground idle
            return AltHold_Landed_Ground_Idle;

        } else {
            // the aircraft can leave the ground at any time
            return AltHold_Landed_Pre_Takeoff;
        }

    } else {
        // the aircraft is in a flying state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHold_Flying;
    }
}

// transform pilot's yaw input into a desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Mode::get_pilot_desired_yaw_rate(float yaw_in)
{
    // throttle failsafe check
    if (soleon.failsafe.radio || !soleon.ap.rc_receiver_present) {
        return 0.0f;
    }

    // convert pilot input to the desired yaw rate
    return 0;
}

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
float Mode::get_pilot_desired_climb_rate(float throttle_control)
{
return 0.0; 
}

float Mode::get_non_takeoff_throttle()
{
 return 0.0; 
}

void Mode::update_simple_mode(void) {
    soleon.update_simple_mode();
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return soleon.set_mode(mode, reason);
}

void Mode::set_land_complete(bool b)
{
 
}

GCS_Soleon &Mode::gcs()
{
    return soleon.gcs();
}

uint16_t Mode::get_pilot_speed_dn()
{

return 0;
}
