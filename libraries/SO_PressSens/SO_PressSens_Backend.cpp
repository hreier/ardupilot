/*
   This is Soleon payload pressure measurement backend
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "SO_PressSens.h"
#include "SO_PressSens_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
SO_PressSens_Backend::SO_PressSens_Backend(PressSens::PressSens_State &_state, SO_PressSens_Params &_params) :
        state(_state),
        params(_params)
{
    _backend_type = type();

    strcpy (gcs_message, "driver init"); 
    msg_updated = false;
}



PressSens::Status SO_PressSens_Backend::status() const {
    if (type() == PressSens::Type::NONE) {
        // turned off at runtime?
        return PressSens::Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool SO_PressSens_Backend::has_data() const {
    return ((state.status != PressSens::Status::NotConnected) &&
            (state.status != PressSens::Status::NoData));
}


// set status and update valid count
void SO_PressSens_Backend::set_status(PressSens::Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == PressSens::Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

// get the measured value from backend
float SO_PressSens_Backend::get_measure()
{
   // return state.pressure;
    return state.flow;
}



bool SO_PressSens_Backend::set_gcs_message(const char *msg)
{
    if (strlen(msg) >= sizeof(gcs_message)) {
        gcs_message[0] = 0;
        return false;
    }

    strcpy (gcs_message, msg);
    msg_updated = true;
    return true;
};


float SO_PressSens_Backend::estimate_flow(float pressure)
{
   // _press_tab_t * pt_nozzle_tab = &nozzle_tabs[params.valv_type];
   float flow = 0;
   int i = 0;
   float (*flow_tab)[PRESS_TABSIZE] = &nozzle_tabs[params.valv_type].tab;
   float dt_flow;

    if (pressure >= (float) PRESS_TABSIZE)      return (*flow_tab)[PRESS_TABSIZE-1];

    //- interpolate
    while (pressure > (float) (i+1))
    {
        flow = (*flow_tab)[i];
        i++;
    }
    
    dt_flow = (*flow_tab)[i] - flow;
    dt_flow = dt_flow * (pressure - (float)i);

    flow += dt_flow;
    
    flow *= params.valves;   //-- each nozzle produces that flow
    
    return flow;
}

//-- returns consumed volume [ml] from flow[l/min] and dt_msec
float SO_PressSens_Backend::calc_consumed_ml (float flow, float dt_msec)
{
    float consumed_val = flow*1000;  //- flow[ml/min]

    consumed_val *= dt_msec; 
    consumed_val /= 60000;           //- consumed[ml]

    return consumed_val;
}
