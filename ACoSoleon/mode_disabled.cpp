#include "Soleon.h"



// Controller disabled - initialise the disabled controller mode
bool ModeCtrlDisabled::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoCtrlMode init: <%s>", name()); //-- the activation routine send similar message
    _time_stamp  = AP_HAL::millis();
    _mode_booting = true;
    return true;
}


// Controller disabled - runs the disabled controller mode
void ModeCtrlDisabled::run()
{
    //-- show the boot sequence
    if (_mode_booting) {
        _mode_booting = bootsequence();
        return;
    }

    if ((AP_HAL::millis() - _time_stamp) < 5000) return;
    
    _time_stamp  = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, "SoCtrlMode <%s> is running", name());  ///-HaRe debug
    //gcs().send_text(MAV_SEVERITY_INFO, "SoCtrlMode <%f> is running", SO::TankSupervision()->get_level());  ///-HaRe debug
  
}

bool ModeCtrlDisabled::is_spraying()
{
    return false;
}