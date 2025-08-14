#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlSprayPPM::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoCtrlMode init: <%s>", name()); //-- the activation routine send similar message
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
    _mp_cmd_act = _mp_cmd; //--- first get the active command from missionplan (_mp_cmd)  

    //_mp_sprayrate = g2.so_press.get_measure(0);  //-- this will somehow be used for regulated modes

    manage_offset_trim(true);    //- update the offset trim value from remote controller (-5.0...0...+5.0%; 0,5% steps) 

    //_ppm_pump = modulate_value_trim(_ppm_pump, 1000);  //-- Manage the trimming; may be needs to be validated if between the limits???

    overrideBySwitch(_mp_cmd_act, _mp_status);       //- remote controller switch can force to run/stop the pump 

    // ---- update valve relays and ppm values ----
    updateSprayerValveRelays(_mp_cmd_act);
    updateSprayerPumpPPMs(_mp_cmd_act, _ppm_pump, _ppm_pump, g.so_servo_out_nospraying.get());

    //--- update status informations
    updateSprayerStatus(_mp_cmd_act, _mp_status);
    
    //--- update the filllevel
    _fill_level = g2.so_scale.get_measure(0); 


    if (is_spraying()) _mp_sprayrate = g.so_sprayrate_est.get();
    else               _mp_sprayrate = 0;

}

bool ModeCtrlSprayPPM::is_spraying()
{
    //return should_be_spraying;
    return (_ppm_pump > g.so_servo_out_nospraying.get());
}