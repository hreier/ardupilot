#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float get_ctrl_p() const { return _ctrl_p; }
    AP_Float get_ctrl_i() const { return _ctrl_i; }
    AP_Float get_ctrl_d() const { return _ctrl_d; }

    AP_Float get_dbg_0() const { return _dbg_0; }
    AP_Float get_dbg_1() const { return _dbg_1; }
    AP_Float get_dbg_2() const { return _dbg_2; }

private:
    // Put your parameter variable definitions here
    AP_Float _ctrl_p;
    AP_Float _ctrl_i;
    AP_Float _ctrl_d;

    //---- debug stuff: intended for testing purpose
    AP_Float _dbg_0;
    AP_Float _dbg_1;
    AP_Float _dbg_2;
};
