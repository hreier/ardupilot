#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlSprayPPM::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message
    should_be_spraying = false;
    _fill_level = g2.so_scale.get_measure(0);
    offset_trim_proz = 0;
    _mode_booting = true;
    _time_stamp  = AP_HAL::millis();
    return true;
}

//#define start_delay 0
// Controller disabled - runs the disabled controller mode
void ModeCtrlSprayPPM::run()
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

    _ppm_pump = g.so_servo_out_spraying.get();
    if (is_spraying()) _mp_sprayrate = g.so_sprayrate_est.get();
    else               _mp_sprayrate = 0;
    //_mp_sprayrate = g2.so_press.get_measure(0);  //-- this will somehow be used for regulated modes

    _fill_level = g2.so_scale.get_measure(0); 

    //--- _mp_status update!
    if (_mp_cmd & MASK_CMD_PUMP_RIGHT)  _mp_status = _mp_status | MASK_STAT_SPR_RIGHT;
    else                                _mp_status = _mp_status & ~MASK_STAT_SPR_RIGHT;

    if (_mp_cmd & MASK_CMD_PUMP_LEFT)   _mp_status = _mp_status | MASK_CMD_PUMP_LEFT;
    else                                _mp_status = _mp_status & ~MASK_CMD_PUMP_LEFT;

    manage_offset_trim(true);    //- update the offset trim value from remote controller (-5.0...0...+5.0%; 0,5% steps) 

    //_ppm_pump = modulate_value_trim(_ppm_pump, 1000);  //-- Manage the trimming; may be needs to be validated if between the limits???

    override_ppm();              //- remote controller switch can force to run/stop the pump (NOTE: this changes the _mp_status)

    updateSprayerValveRelays(_mp_cmd);
    updateSprayerPumpPPMs(_mp_cmd, _ppm_pump, _ppm_pump, g.so_servo_out_nospraying.get());

}

bool ModeCtrlSprayPPM::is_spraying()
{
    //return should_be_spraying;
    return (_ppm_pump > g.so_servo_out_nospraying.get());
}