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
    channel_speed(soleon.channel_speed),
    channel_offset(soleon.channel_offset),
    channel_override(soleon.channel_override),
    G_Dt(soleon.G_Dt)
{ };

//---------------------------------------------------------------------
// this shows the active Controlmode with the PUMP
// returns true as long bootsequence is ongoing
// returns false if bootsequence is done
#define PULS_LENGHT 1000
bool Mode::bootsequence(void)
{
    uint16_t spray_pwm; 
    uint32_t d_time = AP_HAL::millis() - _time_stamp;

    if (d_time >= (PULS_LENGHT* ((int)mode_number()+1))){
        _time_stamp = AP_HAL::millis();
        return false; //- done
    }
    
    SRV_Channels::get_output_pwm(SRV_Channel::k_sprayer_pump, spray_pwm);

    d_time %= PULS_LENGHT;

    if (d_time < PULS_LENGHT/2){
        if (spray_pwm != g.so_servo_out_spraying.get()) SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, g.so_servo_out_spraying.get());
    }
    else {
        if (spray_pwm != g.so_servo_out_nospraying.get()) SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, g.so_servo_out_nospraying.get());
    }

    return true; 
}

//-------------------------------------------------------------------------
// This modulates in_value with the channel_offset trim (offset_trim_proz)
// max_deviation defines the maximal value for the modulation;
// this would be the value modification in case the offset_trim_proz == 1.0;
// return: the value with offset
float Mode::modulate_value_trim(float in_value, float max_deviation)
{
    float offset_val;

    offset_val = (max_deviation * offset_trim_proz) / 100;   //- the offset

    return (in_value + offset_val);
}

//-------------------------------------------------------------------------
// This overrides the Pump server PPM signal depending on the override RCin
// handles the _mp_status ready bit;
// manipulates _ppm_pump if overriding
// Note: this needs to be last manipulation before to update servo channel
void Mode::override_ppm(void)
{
    RC_Channel::AuxSwitchPos override_sw = channel_override->get_aux_switch_pos();

    switch (override_sw)
    {
        //-- Pump forced to stand still
        default:
        case RC_Channel::AuxSwitchPos::LOW:
            _mp_status &= ~0x04;                            //- set sprayer status to not ready 
            _ppm_pump = g.so_servo_out_nospraying.get();    //- override with no spaying ppm-value
            offset_trim_proz = 0;
            break;

        //-- Pump controlled by mission
        case RC_Channel::AuxSwitchPos::MIDDLE:
            _mp_status |= 0x04;                             //- sprayer is ready 
            break;

        //-- Pump forced to run
        case RC_Channel::AuxSwitchPos::HIGH:
            _mp_status &= ~0x04;                            //- set sprayer status to not ready 
            _ppm_pump = g.so_servo_out_spraying.get();      //- override with spaying ppm-value
            break;
    }
}

//-------------------------------------------------------------------------
// This manages the offset channel from remote control  
// - inc/dec the offset percent value in 0.5% steps   
// - limit between -5% to 5%
// - verbose: send info message to Ground Station if offset reached limits or 0
#define TRIM_DELTA 0.5
#define OFFSET_MAX 5.0
#define OFFSET_MIN -5.0
void Mode::manage_offset_trim(bool verbose)
{
    RC_Channel::AuxSwitchPos channel_pos = channel_offset->get_aux_switch_pos();

    switch (channel_pos)
    {
        //-- TrimDec
        case RC_Channel::AuxSwitchPos::LOW:
            if (_last_offset_trim_pos == RC_Channel::AuxSwitchPos::MIDDLE) offset_trim_proz -= TRIM_DELTA;
            break;

        //-- Middle
        default:
        case RC_Channel::AuxSwitchPos::MIDDLE:
            if (_last_offset_trim_pos == RC_Channel::AuxSwitchPos::MIDDLE) break;
            
            if (fabs(offset_trim_proz) < 0.001) {
                if (verbose) gcs().send_text(MAV_SEVERITY_NOTICE, "Offsettrim zero percent");
                break;
            }
            if (offset_trim_proz >= OFFSET_MAX) {
                if (verbose) gcs().send_text(MAV_SEVERITY_NOTICE, "Offsettrim at Max value: %f", OFFSET_MAX);
                offset_trim_proz = OFFSET_MAX;
                break;
            }
            if (offset_trim_proz <= OFFSET_MIN) {
                if (verbose) gcs().send_text(MAV_SEVERITY_NOTICE, "Offsettrim at Min value: %f", OFFSET_MIN);
                offset_trim_proz = OFFSET_MIN;
            }
            break;

        //-- TrimInc
        case RC_Channel::AuxSwitchPos::HIGH:
            if (_last_offset_trim_pos == RC_Channel::AuxSwitchPos::MIDDLE) offset_trim_proz += TRIM_DELTA;
            break;
    }

    _last_offset_trim_pos = channel_pos;  //- store actual value to 
}


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
        (uint8_t)Mode::Number::CTRL_TEST,
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
      case Mode::Number::CTRL_SPRAY_PPM:
        fred = &ctrl_spray_ppm;
        break;
      case Mode::Number::CTRL_TEST:
        fred = &ctrl_test;
        break;
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
    
    //- look if the control mode has be changed (configuration change)
    if (soleon_ctrl_mode->mode_number() !=  (enum Mode::Number)g.so_controlmode.get()){
        set_mode((enum Mode::Number)g.so_controlmode.get(), ModeReason::RC_COMMAND);
    }
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

