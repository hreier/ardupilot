#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlSprayPPM::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message
    should_be_spraying = false;
    _fill_level = 30;   //-- let assume the tank is full
    offset_trim_proz = 0;
    _mode_booting = true;
    _time_stamp  = AP_HAL::millis();
    return true;
}

#define start_delay 0
// Controller disabled - runs the disabled controller mode
void ModeCtrlSprayPPM::run()
{
    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    //-- show the boot sequence
    if (_mode_booting) {
        _mode_booting = bootsequence();
        return;
    }

      
    switch (_mp_cmd) {
        case mp_cmd_t::SPR_OFF :
            _mp_status = (_mp_status & 0xfc);
            should_be_spraying = false;
            break;
        
        case mp_cmd_t::SPR_RIGHT :
            _mp_status = (_mp_status & 0xfc) | 0x1;
            should_be_spraying = true;
            break;

        case mp_cmd_t::SPR_LEFT:
            _mp_status = (_mp_status & 0xfc) | 0x2;
            should_be_spraying = true;
            break;

        case mp_cmd_t::SPR_BOTH:
            _mp_status = (_mp_status & 0xfc) | 0x3;
            should_be_spraying = true;
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "illegal mission plan command: %d", (int) _mp_cmd);
            should_be_spraying = false;
            break;
        }

    if (should_be_spraying) {
        _mp_sprayrate = g.so_sprayrate_est.get();
        _ppm_pump = g.so_servo_out_spraying.get();
        }
    else {
        _mp_sprayrate = 0;
        _ppm_pump = g.so_servo_out_nospraying.get();
        }
    
    manage_offset_trim(true);    //- update the offset trim value from remote controller (-5.0...0...+5.0%; 0,5% steps) 

    _fill_level = modulate_value_trim(30, 30);  //- for test --> this will be 
    _ppm_pump = modulate_value_trim(_ppm_pump, 1000);

    override_ppm();              //- remote controller switch can force to run/stop the pump
    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump, _ppm_pump);
}

