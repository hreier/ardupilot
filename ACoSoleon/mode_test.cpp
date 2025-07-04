#include "Soleon.h"


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlTest::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message
    _fill_level = 30.0f;
    _delta_fill = 0.0f;
    _mp_status = 0;
    _mp_cmd = mp_cmd_t::SPR_OFF;
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
    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump_r)) {
        return;
    }

    //-- show the boot sequence
    if (_mode_booting) {
        _mode_booting = bootsequence();
        return;
    }
    
 
    //--- simultate sprayer...
     _fill_level -= _delta_fill;
    if (_fill_level < 0.0f) _fill_level = 30.0f;
    
    _mp_sprayrate = _delta_fill;

    if (_mp_liter_ha > 0.0f){
    _mp_status |= 0x04;

    switch (_mp_cmd) {
        case mp_cmd_t::SPR_OFF :
            _mp_status = (_mp_status & 0xfc);
            _delta_fill = 0;
            break;
        
        case mp_cmd_t::SPR_RIGHT :
            _mp_status = (_mp_status & 0xfc) | 0x1;
            _delta_fill = 0.001;
            break;

        case mp_cmd_t::SPR_LEFT:
            _mp_status = (_mp_status & 0xfc) | 0x2;
            _delta_fill = 0.05;
           break;

        case mp_cmd_t::SPR_BOTH:
            _mp_status = (_mp_status & 0xfc) | 0x3;
            _delta_fill = 0.1;
            break;

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "illegal mission plan command: %d", (int) _mp_cmd);
            break;
        }

    } 
    else{
        _mp_status = 0;
    }

    //--- print the RC-signals for debugging
    if ((AP_HAL::millis() - _time_stamp) < 5000) return;

    _time_stamp  = AP_HAL::millis();
   /* gcs().send_text(MAV_SEVERITY_INFO, "Speed: %d; Offset: %d; Override: %d;", \
          channel_speed->get_radio_in(), channel_offset->get_radio_in(), channel_override->get_radio_in());  ///-HaRe debug
          
          AuxSwitchPos get_aux_switch_pos()
          bool RC_Channel::read_3pos_switch(RC_Channel::AuxSwitchPos &ret) 

          chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW

          float       norm_input() const;
          float       norm_input_dz() const;
          uint8_t     percent_input() const;
          
          */
    gcs().send_text(MAV_SEVERITY_INFO, "SpeedPWM: %d; Speed: %d; SpeedProz: %d", \
          channel_speed->get_radio_in(), channel_speed->get_control_in(), channel_speed->percent_input());  ///-HaRe debug
          

}

bool ModeCtrlTest::is_spraying()
{
    return false;
}