/*
   This is Soleon payload weight measurement 
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
//#include "SO_WeightSens.h"
#include "SO_WeightSens_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
SO_WeightSens_Backend::SO_WeightSens_Backend(WeightSens::WeightSens_State &_state, SO_WeightSens_Params &_params) :
        state(_state),
        params(_params)
{
    _backend_type = type();
}



WeightSens::Status SO_WeightSens_Backend::status() const {
    if (type() == WeightSens::Type::NONE) {
        // turned off at runtime?
        return WeightSens::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool SO_WeightSens_Backend::has_data() const {
    return ((state.status != WeightSens::Status::NotConnected) &&
            (state.status != WeightSens::Status::NoData));
}

// update status based on distance measurement
void SO_WeightSens_Backend::update_status()
{
   /*zu adaptieren 
    // check distance
    if (state.distance_m > max_distance_cm() * 0.01f) {
        set_status(WeightSens::Status::OutOfRangeHigh);
    } else if (state.distance_m < min_distance_cm() * 0.01f) {
        set_status(WeightSens::Status::OutOfRangeLow);
    } else {
        set_status(WeightSens::Status::Good);
    }*/
}

// set status and update valid count
void SO_WeightSens_Backend::set_status(WeightSens::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == WeightSens::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

