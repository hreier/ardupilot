/*
   This is Soleon pressure sensor driver frontend:
   > implements a pressure measurement that uses 4-20mA sensors on the backend site
   > calculates the flow using nozzle curves and amount of nozzles 
 */

#include "SO_PressSens.h"
#include "SO_PressSens_Analog.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
//#include <AP_HAL/I2CDevice.h>

#include <GCS_MAVLink/GCS.h> //-testing


extern const AP_HAL::HAL &hal;

// table of user settable parameters 
const AP_Param::GroupInfo PressSens::var_info[] = {
    // @Group: 1_
    // @Path: SO_PressSens_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 1, PressSens, SO_PressSens_Params),

    // @Group: 1_
    // @Path: SO_PressSens_Analog.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  2, PressSens, backend_var_info[0]),

#if PRESS_SENS_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_PressSens_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, PressSens, SO_PressSens_Params),

    // @Group: 2_
    // @Path: AP_PressSens_Wasp.cpp,AP_PressSens_Benewake_CAN.cpp,AP_PressSens_USD1_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, PressSens, backend_var_info[1]),
#endif
// -- to extend like this in case of more instances should be needed 

    AP_GROUPEND
};

const AP_Param::GroupInfo *PressSens::backend_var_info[PRESS_SENS_MAX_INSTANCES];

PressSens::PressSens()
{
    AP_Param::setup_object_defaults(this, var_info);
    num_instances = 0;

    _singleton = this;
}

/*
  initialise the PressSens class. We do detection of attached pressure sensor modules here. 
  For now we won't allow for hot-plugging of pressure sensors.
 */
void PressSens::init()
{
    if (num_instances != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

    for (uint8_t i=0; i<PRESS_SENS_MAX_INSTANCES; i++) {
        // detect and load the configured drivers
        WITH_SEMAPHORE(detect_sem);
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN press_sens may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update PressSens state for all instances. This should be called at
  around 10Hz by main loop
 */

//int dbg_cnt; //--debug
void PressSens::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            drivers[i]->update(); 
        }
    }
#if HAL_LOGGING_ENABLED
    Log_RFND();
#endif
//------------debug---
//if (dbg_cnt++ > 80)
//  {
//    dbg_cnt = 0;
//
//    gcs().send_text(MAV_SEVERITY_WARNING, "an=%.2f; prs=%.2f; flw=%.2f; ml=%.0f; %d", drivers[0]->state.analog_val, drivers[0]->state.pressure, drivers[0]->state.flow, drivers[0]->state.ml_accu, (int) drivers[0]->state.status);
//  }
}

bool PressSens::_add_backend(SO_PressSens_Backend *backend, uint8_t instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= PRESS_SENS_MAX_INSTANCES) {
        AP_HAL::panic("Too many weight module instances");
    }
    if (drivers[instance] != nullptr) {
        // we've allocated the same instance twice
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    drivers[instance] = backend;
    num_instances = MAX(num_instances, instance+1);

    return true;
}

/*
  detect if an instance of a press_sens is connected. 
 */
void PressSens::detect_instance(uint8_t instance)
{
#if SO_PRESS_SENS_ENABLED   //--- install backends
    //-- This is where we load the different types of backend sensor drivers depending on the configuration
    //-- we currently support only analog sensors (4-20mA; 1-5V) 
    
    _add_backend(SO_PressSens_Analog::detect(state[instance], params[instance]), instance);

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
#endif //SO_PRESS_SENS_ENABLED
}

SO_PressSens_Backend *PressSens::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    return drivers[id];
};

// returns the Pressure value [bar]
float PressSens::getPressure(uint8_t id) 
{
    if (id >= PRESS_SENS_MAX_INSTANCES) return 0;
    else return drivers[id]->getPressure();
}


// returns the PressSens status
PressSens::Status PressSens::get_status(uint8_t id)
{
    if (id >= PRESS_SENS_MAX_INSTANCES) return PressSens::Status::NotConnected;
    else return drivers[id]->status();
}



// Write an RFND (press_sens) packet
void PressSens::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
        
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<PRESS_SENS_MAX_INSTANCES; i++) {
        const SO_PressSens_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_RFND pkt = {
                LOG_PACKET_HEADER_INIT(LOG_RFND_MSG),
                time_us      : AP_HAL::micros64(),
                instance     : i,
                dist         : 0,  //s->distance_cm(),
                status       : (uint8_t)s->status(),
                orient       : 0,  //s->orientation(),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}



PressSens *PressSens::_singleton;

namespace SO {

PressSens *press_sens()
{
    return PressSens::get_singleton();
}

}

