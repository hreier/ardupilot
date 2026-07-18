#pragma once

#include <AP_Param/AP_Param.h>

// default rate controller PID gains
#define SO_PRESS_RATE_RP_P                       0.135f
#define SO_PRESS_RATE_RP_I                       0.135f
#define SO_PRESS_RATE_RP_D                       0.0036f
#define SO_PRESS_RATE_RP_IMAX                    0.4f
#define SO_PRESS_RATE_RP_FF                      0.15f
#define SO_PRESS_RATE_RP_FILT_T_HZ               20.0f
#define SO_PRESS_RATE_RP_FILT_E_HZ               0.0f
#define SO_PRESS_RATE_RP_FILT_D_HZ               20.0f
#define SO_PRESS_CTRL_MIN_FLOW                   0.0f




class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float get_ctrl_p() const { return _ctrl_p; }
    AP_Float get_ctrl_i() const { return _ctrl_i; }
    AP_Float get_ctrl_d() const { return _ctrl_d; }
    AP_Float get_ctrl_imax() const { return _ctrl_imax; }
    AP_Float get_ctrl_ff() const { return _ctrl_ff; }
    AP_Float get_ctrl_filt_t() const { return _ctrl_filt_t; }
    AP_Float get_ctrl_filt_e() const { return _ctrl_filt_e; }
    AP_Float get_ctrl_filt_d() const { return _ctrl_filt_d; }

    AP_Float get_ctrl_min_flow() const { return _ctrl_min_flow; }

    AP_Float get_dbg_0() const { return _dbg_0; }
    AP_Float get_dbg_1() const { return _dbg_1; }
    AP_Float get_dbg_2() const { return _dbg_2; }

private:
    // Put your parameter variable definitions here
    //---- Soleon PID controller configuration
    AP_Float _ctrl_p;
    AP_Float _ctrl_i;
    AP_Float _ctrl_d;
    AP_Float _ctrl_imax;
    AP_Float _ctrl_ff;
    AP_Float _ctrl_filt_t;
    AP_Float _ctrl_filt_e;
    AP_Float _ctrl_filt_d;

    AP_Float _ctrl_min_flow;   //-- flow  minimum limitation for ModeCtrlSprayFlow-class

    //---- debug stuff: intended for testing purpose
    AP_Float _dbg_0;
    AP_Float _dbg_1;
    AP_Float _dbg_2;
};
