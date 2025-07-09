#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlTest::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message
    _fill_level = 30.0f;
    _delta_fill = 0.0f;
    _mp_status = 0;
    _mp_cmd = 0;
    _mp_liter_ha =0;
    _mp_line_dist =0;
    _mp_planned_spd =0;
    _mp_dist_waypoint =0;
    _mp_sprayrate =0;

    _mode_booting = true;
    _time_stamp  = AP_HAL::millis();
    return true;
}

bool test = false;
// Controller disabled - runs the disabled controller mode
void ModeCtrlTest::run()
{
    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump_r)) {
        return;
    }

    //-- show the boot sequence
    if (_mode_booting) {
        _mode_booting = bootsequence();
        return;
    }

    if (!test){
        gcs().send_text(MAV_SEVERITY_INFO, "passed");
        test = true;
    }
   // return;
    
    // ---- use override switch to test the sprayer ---
    RC_Channel::AuxSwitchPos override_sw = channel_override->get_aux_switch_pos();
    float ppm_pumps = g.so_servo_out_spraying.get();
    float ppm_pumps_off = g.so_servo_out_nospraying.get();
    uint8_t command;

/*     if (!test){
        gcs().send_text(MAV_SEVERITY_INFO, "passed:%d", (u_int16_t) override_sw);
        test = true;
    }
   // return;
 */
    switch (override_sw)
    {
        default:
        case RC_Channel::AuxSwitchPos::LOW:
            command = 0;    //- spray off
            break;

        case RC_Channel::AuxSwitchPos::MIDDLE:
            command = 3;    //- spray back both
            break;

        case RC_Channel::AuxSwitchPos::HIGH:
            command = 12;    //- spray front both
            break;
    }

    updateSprayerValveRelays(command);
    updateSprayerPumpPPMs(command, ppm_pumps, ppm_pumps, ppm_pumps_off);

}


bool ModeCtrlTest::is_spraying()
{
    return false;
}