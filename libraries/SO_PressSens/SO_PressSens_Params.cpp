#include "SO_PressSens_Params.h"
#include "SO_PressSens.h"

// Currently this implements pressure measurement using 4-20mA sensors 
//  pin 4:power2_pin3,8:adc_pin2,13:power2_pin4,14:power1_pin4,15:power1_pin3      --- default 8
// table of user settable parameters
const AP_Param::GroupInfo SO_PressSens_Params::var_info[] = {
    // @Param: PIN
    // @DisplayName: Sensor input pin 
    // @Description: This selects the Pin to connect the pressure sensor 
    // @Units: l
    // @Values: 4:power2_pin3,8:adc_pin2,13:power2_pin4,14:power1_pin4,15:power1_pin3
    // @User: Standard
    AP_GROUPINFO("PIN", 1, SO_PressSens_Params, pin, 15),

    // @Param: RANGE
    // @DisplayName: Sensor range
    // @Description: Sensor range is the pressure value that correlates to 20mA current on the sensor output. 
    // @Units: Bar
    // @Range: 1.0 15.0 Bar
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RANGE",  2, SO_PressSens_Params, range, 10.0f),
   

    // @Param: VALVTP
    // @DisplayName: Valve type
    // @Description: This parameter defines the valve type
    // @Values: 0:-025-viol,1:-030-blue,2:-040-red
    // @User: Standard
    AP_GROUPINFO("VALVTP", 3, SO_PressSens_Params, valv_type, 1),

    // @Param: VALVES
    // @DisplayName: Valve number
    // @Description: This parameter defines the amount of valves
    // @Range: 1 8
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("VALVES", 4, SO_PressSens_Params, valves, 5),


    AP_GROUPEND
};

SO_PressSens_Params::SO_PressSens_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
