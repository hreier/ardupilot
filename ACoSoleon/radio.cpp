#include "Soleon.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------



void Soleon::init_rc_in()
{
    channel_speed    = rc().channel(so_rcmap.speed()-1);
    channel_offset   = rc().channel(so_rcmap.offset()-1);
    channel_override = rc().channel(so_rcmap.override()-1);

    // set rc channel ranges
    channel_speed->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_offset->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_override->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    //channel_throttle->set_range(1000);

    // set default dead zones
    channel_speed->set_default_dead_zone(0);   //20
    channel_offset->set_default_dead_zone(0);
    channel_override->set_default_dead_zone(0);

    // initialise throttle_zero flag
    //ap.throttle_zero = true;
}

 // init_rc_out -- initialise rc outputs
void Soleon::init_rc_out()
{
     // enable aux servos to cope with multiple output channels per motor
     SRV_Channels::enable_aux_servos();

}


void Soleon::read_radio()
{
    const uint32_t tnow_ms = millis();

    if (rc().read_input()) {
        ap.new_radio_frame = true;

       // set_throttle_and_failsafe(channel_throttle->get_radio_in());
       // set_throttle_zero_flag(channel_throttle->get_control_in());

        // RC receiver must be attached if we've just got input
        ap.rc_receiver_present = true;

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
       // radio_passthrough_to_motors();

        //const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
       // rc_throttle_control_in_filter.apply(channel_throttle->get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
        return;
    }

    // No radio input this time
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    // trigger failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    // if (!ap.rc_receiver_present && !motors->armed()) {
    //     // we only failsafe if we are armed OR we have ever seen an RC receiver
    //     return;
    // }

    // Log an error and enter failsafe.
    AP::logger().Write_Error(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
//haRe    set_failsafe_radio(true);
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Soleon::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present /*|| motors->armed()*/)) {
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
            }
        }
        // pass through throttle
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void Soleon::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !SRV_Channels::get_emergency_stop()) ||
//        (ap.using_interlock && motors->get_interlock()) ||
        ap.armed_with_airmode_switch || air_mode == AirMode::AIRMODE_ENABLED) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}



/*
  return the throttle input for mid-stick as a control-in value
 */
/*int16_t Soleon::get_throttle_mid(void)
{
#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        return g2.toy_mode.get_throttle_mid();
    }
#endif
    return channel_throttle->get_control_mid();
}*/
