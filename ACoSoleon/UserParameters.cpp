#include "UserParameters.h"


// "SO" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    // Soleon debugging
    AP_GROUPINFO("_DBG_0", 0, UserParameters, _dbg_0, 0.0f),
    AP_GROUPINFO("_DBG_1", 1, UserParameters, _dbg_1, 0.0f),
    AP_GROUPINFO("_DBG_2", 2, UserParameters, _dbg_2, 0.0f),

    // Soleon PID controller 
    AP_GROUPINFO("_CTRL_P", 3, UserParameters, _ctrl_p, SO_PRESS_RATE_RP_P),
    AP_GROUPINFO("_CTRL_I", 4, UserParameters, _ctrl_i, SO_PRESS_RATE_RP_I),
    AP_GROUPINFO("_CTRL_D", 5, UserParameters, _ctrl_d, SO_PRESS_RATE_RP_D),
    AP_GROUPINFO("_CTRL_IMAX", 6, UserParameters, _ctrl_imax, SO_PRESS_RATE_RP_IMAX),
    AP_GROUPINFO("_CTRL_FF", 7, UserParameters, _ctrl_ff, SO_PRESS_RATE_RP_FF),
    AP_GROUPINFO("_CTRL_FILT_T", 8, UserParameters, _ctrl_filt_t, SO_PRESS_RATE_RP_FILT_T_HZ),
    AP_GROUPINFO("_CTRL_FILT_E", 9, UserParameters, _ctrl_filt_e, SO_PRESS_RATE_RP_FILT_E_HZ),
    AP_GROUPINFO("_CTRL_FILT_D", 10, UserParameters, _ctrl_filt_d, SO_PRESS_RATE_RP_FILT_D_HZ),

    AP_GROUPINFO("_SPRAYRATE_MIN", 11, UserParameters, _ctrl_min_flow, SO_PRESS_CTRL_MIN_FLOW),


    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
