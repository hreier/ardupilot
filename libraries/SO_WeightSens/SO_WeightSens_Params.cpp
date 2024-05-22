#include "SO_WeightSens_Params.h"
#include "SO_WeightSens.h"


// table of user settable parameters
const AP_Param::GroupInfo SO_WeightSens_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: WeightSensor type
    // @Description: Type of connected WeightSensor
    // @Values: 0:None,1:FX29_I2C
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, SO_WeightSens_Params, type, 1),

    // @Param: MODE
    // @DisplayName: WeightSensor mode
    // @Description: mode of connected WeightSensor
    // @Values: 0:disabled,1:enabled
    // @User: Standard
    AP_GROUPINFO("MODE", 2, SO_WeightSens_Params, mode, 1),


    // @Param: OFFSET
    // @DisplayName: WeightSensor offset
    // @Description: Offset in liters for weight zero adjustment. 
    // @Units: l
    // @Range: -5.0 15.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("OFFSET",  3, SO_WeightSens_Params, offset, 0.0f),
    
    // @Param: ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the address of the sensor, where applicable (or first address if module with more sensors). Used for the I2C sensors to allow for multiple sensors on different addresses. 
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 4, SO_WeightSens_Params, address, 40),

    // @Param: SENSRNG
    // @DisplayName: Sensor range
    // @Description: This parameter defines the range in Newton that the sensor can measure
    // @Units: N
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("PWRRNG", 5, SO_WeightSens_Params, range, 500),


    AP_GROUPEND
};

SO_WeightSens_Params::SO_WeightSens_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
