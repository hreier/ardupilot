#include "UserParameters.h"

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_CTRL_P", 0, UserParameters, _ctrl_p, 10.0f),
    AP_GROUPINFO("_CTRL_I", 1, UserParameters, _ctrl_i, 0.07f),
    AP_GROUPINFO("_CTRL_D", 2, UserParameters, _ctrl_d, 0.0f),

    AP_GROUPINFO("_DBG_0", 3, UserParameters, _dbg_0, 3.0f),
    AP_GROUPINFO("_DBG_1", 4, UserParameters, _dbg_1, 2.0f),
    AP_GROUPINFO("_DBG_2", 5, UserParameters, _dbg_2, 0.0f),


    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
