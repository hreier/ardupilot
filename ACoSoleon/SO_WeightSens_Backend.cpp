/*
   This is Soleon payload weight measurement backend
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

    strcpy (gcs_message, "driver init"); 
    msg_updated = false;
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

// get the measured value from backend
float SO_WeightSens_Backend::get_measure()
{
    return state.weight_kg;
}

bool SO_WeightSens_Backend::set_gcs_message(const char *msg)
{
    if (strlen(msg) >= sizeof(gcs_message)) {
        gcs_message[0] = 0;
        return false;
    }

    strcpy (gcs_message, msg);
    msg_updated = true;
    return true;
};