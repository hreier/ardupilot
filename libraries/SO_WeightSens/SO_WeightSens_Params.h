#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class SO_WeightSens_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    SO_WeightSens_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(SO_WeightSens_Params);

    AP_Float offset;
    AP_Int8  mode;
    AP_Int8  type;
    AP_Int8  address;
    AP_Float weight;
    AP_Int16 range;

};
