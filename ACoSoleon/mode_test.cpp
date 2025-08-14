#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlTest::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoCtrlMode init: <%s>", name()); //-- the activation routine send similar message
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

// Controller disabled - runs the disabled controller mode
void ModeCtrlTest::run()
{

    //-- show the boot sequence
    if (_mode_booting) {
        _mode_booting = bootsequence();
        return;
    }

    _ppm_pump = g.so_servo_out_spraying.get();
    _mp_cmd_act = _mp_cmd; //--- first get the active command from missionplan (_mp_cmd)  

     // ---- update valve relays and ppm values ----
    updateSprayerValveRelays(_mp_cmd_act);
    updateSprayerPumpPPMs(_mp_cmd_act, _ppm_pump, _ppm_pump, g.so_servo_out_nospraying.get());

    //--- update status informations
    if (_mp_cmd_act & MASK_CMD_PUMP_RIGHT)  _mp_status = _mp_status | MASK_STAT_SPR_RIGHT;
    else                                    _mp_status = _mp_status & ~MASK_STAT_SPR_RIGHT;

    if (_mp_cmd_act & MASK_CMD_PUMP_LEFT)    _mp_status = _mp_status | MASK_STAT_SPR_LEFT;
    else                                     _mp_status = _mp_status & ~MASK_STAT_SPR_LEFT;

    _fill_level = g2.so_scale.get_measure(0); 

    if (is_spraying()) _mp_sprayrate = g.so_sprayrate_est.get();
    else               _mp_sprayrate = 0;
}


bool ModeCtrlTest::is_spraying()
{
    return false;
}