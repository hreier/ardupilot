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
    _fill_level = 30.0f;
    _delta_fill = 0.0f;
    _mp_status = 0;
    _mp_cmd = 0;
    _mp_liter_ha =0;
    _mp_line_dist =0;
    _mp_planned_spd =0;
    _mp_dist_waypoint =0;
    _mp_sprayrate =0;


    return true;
}

int temp2;
void SO_TankSupervision::update()
{
    _fill_level -= _delta_fill;
    if (_fill_level < 0.0f) _fill_level = 30.0f;
    
   /* if (temp2++> 30){  //-- debugging
        gcs().send_text(MAV_SEVERITY_INFO, "ACo Tanklevel = %f", _fill_level);  ///-HaRe debug
        temp2=0;
    }*/
    _mp_sprayrate = _delta_fill;

    if (_mp_liter_ha != 0.0f){
    _mp_status |= 0x04;

    switch (_mp_cmd) {
        case 0:
            _mp_status = (_mp_status & 0xfc);
            _delta_fill = 0;
            break;
        
        case 1:
            _mp_status = (_mp_status & 0xfc) | 0x1;
            _delta_fill = 0.001;
            break;

        case 2:
            _mp_status = (_mp_status & 0xfc) | 0x2;
             _delta_fill = 0.05;
           break;

        case 3:
            _mp_status = (_mp_status & 0xfc) | 0x3;
            _delta_fill = 0.1;
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "illegal mp-cmd: %d", _mp_cmd);
            break;
        }
    } 
    else{
        _mp_status = 0;
    }


};

// brake_run - runs the brake controller
// should be called at 100hz or more
void SO_TankSupervision::run()
{
;
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

