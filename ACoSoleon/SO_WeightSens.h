/*
   Soleon weightsens class
 */
#pragma once

#include "SO_WeightSens_config.h" 

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "SO_WeightSens_Params.h"

// Maximum number of weight sensor module instances available on this platform
#ifndef WEIGHTSENS_MAX_INSTANCES 
  #define WEIGHTSENS_MAX_INSTANCES 1
#endif


class SO_WeightSens_Backend;

class WeightSens
{
    friend class SO_WeightSens_Backend;

public:
    WeightSens();

    /* Do not allow copies */
    CLASS_NO_COPY(WeightSens);

    // WeightSens driver types
    enum class Type {
        NONE     = 0,
        FX29_I2C = 1
        };

    enum class Function {
        LINEAR    = 0,
        INVERTED  = 1,
        HYPERBOLA = 2
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        Good,
        SensorError,
        setAdd
    };

    // The WeightSens_State structure is filled in by the backend driver
    struct WeightSens_State {
        float weight_kg;                // weight in kg
        enum WeightSens::Status status; // sensor status
        uint8_t  range_valid_count;     // number of consecutive valid readings (maxes out at 10)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[WEIGHTSENS_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /*
      Return the number of weight sensor instances. 
      intended for future extension....
    */
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len) const;

    // detect and initialise any available rangefinders  HaRe remove rotation_default
    void init(void);

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);


    SO_WeightSens_Backend *get_backend(uint8_t id) const;

    // get rangefinder type for an ID
    Type get_type(uint8_t id) const {
        return id >= WEIGHTSENS_MAX_INSTANCES? Type::NONE : Type(params[id].type.get());
    }

    // get rangefinder address (for AP_Periph CAN)
    uint8_t get_address(uint8_t id) const {
        return id >= WEIGHTSENS_MAX_INSTANCES? 0 : uint8_t(params[id].address.get());
    }
    
    // returns the WeightSens measured value [kg / l]
    float get_measure(uint8_t id);

    // returns the WeightSens status
    WeightSens::Status get_status(uint8_t id);
    
    // backend messages
    bool is_new_gcs_message(uint8_t id);
    const char * get_gcs_message(uint8_t id);



    static WeightSens *get_singleton(void) { return _singleton; }

protected:
    SO_WeightSens_Params params[WEIGHTSENS_MAX_INSTANCES];

private:
    static WeightSens *_singleton;

    WeightSens_State state[WEIGHTSENS_MAX_INSTANCES];
    SO_WeightSens_Backend *drivers[WEIGHTSENS_MAX_INSTANCES];
    uint8_t num_instances;
    HAL_Semaphore detect_sem;


    void detect_instance(uint8_t instance);
    bool _add_backend(SO_WeightSens_Backend *driver, uint8_t instance);

    uint32_t _log_rfnd_bit = -1;
    void Log_RFND() const;
};

namespace SO {
    WeightSens *weightsens();
};
