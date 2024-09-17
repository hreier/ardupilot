#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class SO_PressSens_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    SO_PressSens_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(SO_PressSens_Params);

    AP_Int8  pin;
    AP_Float range;
    AP_Int8  valv_type;
    AP_Int8  valves;    
 
};
