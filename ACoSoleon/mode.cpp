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
        (uint8_t)Mode::Number::CTRL_DISABLED,
        (uint8_t)Mode::Number::CTRL_SPRAY_PPM,
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
void Soleon::set_mode(Mode &newmode, ModeReason reason)
{
    control_mode_reason = reason;

    if (soleon_ctrl_mode == &newmode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    soleon_ctrl_mode = &newmode;

    //-- init the new control mode
    soleon_ctrl_mode->init();

	// log mode change
	logger.Write_Mode((uint8_t)soleon_ctrl_mode->mode_number(), reason);
    gcs().send_text(MAV_SEVERITY_INFO, "Soleon Controller mode %s loaded (reason: %d)", soleon_ctrl_mode->name(), (int)reason);
    //gcs().send_message(MSG_HEARTBEAT);

}

bool Soleon::set_mode(Mode::Number mode, ModeReason reason)
{
    Mode *fred = nullptr;

    switch ((Mode::Number)mode) {
      default:
      case Mode::Number::CTRL_DISABLED:
        fred = &ctrl_disabled;
        break;
/*    case Mode::Number::CTRL_SPRAY_PPM:
        fred = &ctrl_disabled;
        break;*/
    }

    if (fred == nullptr) {
        return false;
    }

    set_mode(*fred, reason);
 
    return true;
}

bool Soleon::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return soleon.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_soleon_ctrl_mode - calls the appropriate control algorithms
// called at 100hz or more
void Soleon::update_soleon_ctrl_mode()
{
    soleon_ctrl_mode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Soleon::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{

    // cancel any takeoffs in progress
    //old_flightmode->takeoff_stop();

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


GCS_Soleon &Mode::gcs()
{
    return soleon.gcs();
}

