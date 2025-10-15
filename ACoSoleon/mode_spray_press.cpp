#include "Soleon.h"

// table of user settable parameters
const AP_Param::GroupInfo ModeCtrlSprayPress::var_info[] = {

    // @Param: SO_CTRL_P
    // @DisplayName: Pump controller P gain
    // @Description: Pump controller P gain. Corrects in proportion to the difference between the desired rate vs actual rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard
   // AP_GROUPINFO("SO_CTRL_P", 0, UserParameters, _mp_cmd_act, 10.0f),

  //  AP_GROUPINFO("SO_CTRL_P", 0, ModeCtrlSprayPress, _mp_cmd_act, 0),

    // @Param: SO_CTRL_I
    // @DisplayName: Pump controller I gain
    // @Description: Pump controller I gain.  Corrects long-term difference in desired rate vs actual rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: SO_CTRL_IMAX
    // @DisplayName: Pump controller I gain maximum
    // @Description: Pump controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: SO_CTRL_D
    // @DisplayName: Pump controller D gain
    // @Description: Pump controller D gain.  Compensates for short-term change in desired rate vs actual rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: SO_CTRL_FF
    // @DisplayName: Pump controller feed forward
    // @Description: Pump controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: SO_CTRL_FLTT
    // @DisplayName: Pump controller target frequency in Hz
    // @Description: Pump controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: SO_CTRL_FLTE
    // @DisplayName: Pump controller error frequency in Hz
    // @Description: Pump controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: SO_CTRL_FLTD
    // @DisplayName: Pump controller derivative frequency in Hz
    // @Description: Pump controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: SO_CTRL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: SO_CTRL_PDMX
    // @DisplayName: Pump controller PD sum maximum
    // @Description: Pump controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced

    // @Param: SO_CTRL_D_FF
    // @DisplayName: Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: SO_CTRL_NTF
    // @DisplayName: Target notch filter index
    // @Description: Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: SO_CTRL_NEF
    // @DisplayName: Error notch filter index
    // @Description: Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_press_right, "SO_CTRL_", 0, ModeCtrlSprayPress, AC_PID),
  //  AP_SUBGROUPINFO(_pid_press_left, "SO_CTRL__", 0, ModeCtrlSprayPress, AC_PID),
  //  AP_SUBGROUPINFO(_pid_press_right, "SO_", 1, ModeCtrlSprayPress, AC_PID),
//AP_SUBGROUPINFO(user_parameters, "SO", 28, ParametersG2, UserParameters),
    AP_GROUPEND
};


ModeCtrlSprayPress::ModeCtrlSprayPress(void)://Mode(),
 //   g(soleon.g),
 //   g2(soleon.g2),
    _pid_press_right(SO_PRESS_RATE_RP_P, SO_PRESS_RATE_RP_I, SO_PRESS_RATE_RP_D, SO_PRESS_RATE_RP_FF, SO_PRESS_RATE_RP_IMAX, SO_PRESS_RATE_RP_FILT_T_HZ, SO_PRESS_RATE_RP_FILT_E_HZ, SO_PRESS_RATE_RP_FILT_D_HZ),
    _pid_press_left(SO_PRESS_RATE_RP_P, SO_PRESS_RATE_RP_I, SO_PRESS_RATE_RP_D, SO_PRESS_RATE_RP_FF, SO_PRESS_RATE_RP_IMAX, SO_PRESS_RATE_RP_FILT_T_HZ, SO_PRESS_RATE_RP_FILT_E_HZ, SO_PRESS_RATE_RP_FILT_D_HZ)
{
//    AP_Param::setup_object_defaults(this, var_info);

    //--- set left site PID configuration same as right site PID configuration ---
    _pid_press_left (_pid_press_right.kP(), _pid_press_right.kI(), _pid_press_right.kD(), _pid_press_right.ff(), _pid_press_right.kIMAX(), \
                       _pid_press_right.filt_T_hz(), _pid_press_right.filt_E_hz(), _pid_press_right.filt_D_hz(), _pid_press_right.kDff() );

}


// Controller disabled - initialise the disabled controller mode
bool ModeCtrlSprayPress::init()
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
void ModeCtrlSprayPress::run()
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

bool ModeCtrlSprayPress::is_spraying()
{
    //return should_be_spraying;
    return (_ppm_pump > g.so_servo_out_nospraying.get());
}