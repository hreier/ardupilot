#include "Soleon.h"

static int temp;

// Controller disabled - initialise the disabled controller mode
bool ModeCtrlSprayPPM::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message
    should_be_spraying = false;
    temp=0; //--debug
    return true;
}


// Controller disabled - runs the disabled controller mode
void ModeCtrlSprayPPM::run()
{
    if (temp <= 10000)
    {
        temp++;        
        if (temp<5000) return;
        if (temp==5000) {
            gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode <%s> is starting up; let test the sprayer", name());  ///-HaRe debug
            return;
        }
        if (temp==6000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Test1: sprayer activated");  
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_spraying.get(), 0, 10000);
            return;
        }
        if (temp==7000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Test1: sprayer deactivated");  
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_nospraying.get(), 0, 10000);
            return;
        }
        if (temp==8000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Test2: sprayer activated");  
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_spraying.get(), 0, 10000);
            return;
        }
        if (temp==9000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Test2: sprayer deactivated");  
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_nospraying.get(), 0, 10000);
            return;
        }
        if (temp==10000) {
        gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode <%s> is running", name());  ///-HaRe debug
        }

        return;
    }
    
    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
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
            should_be_spraying = true;
            // _delta_fill = 0.05;
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

    if (should_be_spraying) SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_spraying.get(), 0, 10000);
    else                    SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, g.so_servo_out_nospraying.get(), 0, 10000);

}

/*
void ModeCtrlSprayPPM::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::Limit::MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::Limit::MIN);

    _flags.spraying = false;
}*/

/*
    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        float pos = ground_speed * _pump_pct_1ms;
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range
        SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    } else {
        stop_spraying();
    }
*/