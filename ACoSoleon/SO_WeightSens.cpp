/*
   This is Soleon weight sensor driver frontend 
 */

#include "SO_WeightSens.h"
#include "SO_WeightSens_FX29_I2C.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters 
const AP_Param::GroupInfo WeightSens::var_info[] = {
    // @Group: 1_
    // @Path: AP_WeightSens_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 25, WeightSens, SO_WeightSens_Params),

    // @Group: 1_
    // @Path: AP_WeightSens_Wasp.cpp,AP_WeightSens_Benewake_CAN.cpp,AP_WeightSens_USD1_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  57, WeightSens, backend_var_info[0]),

#if WEIGHTSENS_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_WeightSens_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 27, WeightSens, SO_WeightSens_Params),

    // @Group: 2_
    // @Path: AP_WeightSens_Wasp.cpp,AP_WeightSens_Benewake_CAN.cpp,AP_WeightSens_USD1_CAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  58, WeightSens, backend_var_info[1]),
#endif
// -- to extend like this in case of more instances should be possible 

    AP_GROUPEND
};

const AP_Param::GroupInfo *WeightSens::backend_var_info[WEIGHTSENS_MAX_INSTANCES];

WeightSens::WeightSens()
{
    AP_Param::setup_object_defaults(this, var_info);

    _singleton = this;
}

/*
  initialise the WeightSens class. We do detection of attached weight modules here. 
  For now we won't allow for hot-plugging of weightsensors.
 */
void WeightSens::init(enum Rotation orientation_default)
{
    if (num_instances != 0) {
        // don't re-init if we've found some sensors already
        return;
    }

    // set orientation defaults
    //for (uint8_t i=0; i<WEIGHTSENS_MAX_INSTANCES; i++) {
    //    params[i].orientation.set_default(orientation_default);
    //}

    for (uint8_t i=0; i<WEIGHTSENS_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        WITH_SEMAPHORE(detect_sem);
        //detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy). We use MAX()
            // here as a UAVCAN weightsens may already have been
            // found
            num_instances = MAX(num_instances, i+1);
        }

        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update WeightSens state for all instances. This should be called at
  around 10Hz by main loop
 */
void WeightSens::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                // allow user to disable a weightsens at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
        }
    }
#if HAL_LOGGING_ENABLED
    Log_RFND();
#endif
}

bool WeightSens::_add_backend(SO_WeightSens_Backend *backend, uint8_t instance)
{
    if (!backend) {
        return false;
    }
    if (instance >= WEIGHTSENS_MAX_INSTANCES) {
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
  detect if an instance of a weightsens is connected. 
 */
void WeightSens::detect_instance(uint8_t instance)
{
#if SO_WEIGHTSENS_ENABLED

    const Type _type = (Type)params[instance].type.get();
    switch (_type) {

    case Type::FX29_I2C:
        //_add_backend(SO_WeightSens_FX29_I2C::detect(state[instance], params[instance],
        //                                           hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, params[instance].address)),
        //                                           instance);
    
        FOREACH_I2C(i) {
             if (_add_backend(SO_WeightSens_FX29_I2C::detect(state[instance], params[instance],
                                                           hal.i2c_mgr->get_device(i, params[instance].address)), instance)) {
                     break;
                 }
             }

        break;
    


    case Type::NONE:
        break;
    }


    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);

        // param count could have changed
        AP_Param::invalidate_count();
    }
#endif //SO_WEIGHTSENS_ENABLED
}

SO_WeightSens_Backend *WeightSens::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == Type::NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};




// Write an RFND (weightsens) packet
void WeightSens::Log_RFND() const
{
    if (_log_rfnd_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_rfnd_bit)) {
        return;
    }

    for (uint8_t i=0; i<WEIGHTSENS_MAX_INSTANCES; i++) {
        const SO_WeightSens_Backend *s = get_backend(i);
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

bool WeightSens::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < WEIGHTSENS_MAX_INSTANCES; i++) {
        if ((Type)params[i].type.get() == Type::NONE) {
            continue;
        }

        if (drivers[i] == nullptr) {
            hal.util->snprintf(failure_msg, failure_msg_len, "WeightSens %X: Not Detected", i + 1);
            return false;
        }

        // backend-specific checks.  This might end up drivers[i]->arming_checks(...).
        switch (drivers[i]->allocated_type()) {
        case Type::FX29_I2C: 
            // HaRe todo: check if all 3 sensors are answering --> errormessage if not the case
            
            break;
        
        default:
            break;
        }

        switch (drivers[i]->status()) {
        case Status::NoData:
            hal.util->snprintf(failure_msg, failure_msg_len, "WeightSens %X: No Data", i + 1);
            return false;
        case Status::NotConnected:
            hal.util->snprintf(failure_msg, failure_msg_len, "WeightSens %X: Not Connected", i + 1);
            return false;
        case Status::OutOfRangeLow:
        case Status::OutOfRangeHigh:
        case Status::Good:  
            break;
        }
    }

    return true;
}

WeightSens *WeightSens::_singleton;

namespace SO {

WeightSens *weightsens()
{
    return WeightSens::get_singleton();
}

}

