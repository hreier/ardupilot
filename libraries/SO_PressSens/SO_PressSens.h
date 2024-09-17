/*
   Soleon press_sens class
 */
#pragma once

#include "SO_PressSens_config.h" 

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include "SO_PressSens_Params.h"
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega Flash Memory Library


// Maximum number of weight sensor module instances available on this platform
#ifndef PRESS_SENS_MAX_INSTANCES 
  #define PRESS_SENS_MAX_INSTANCES 1
#endif

#define PRESS_MODULE_LOGGING      //-- this activates logging of the Weight sensor module for debugging

// Nozzle types
enum class NozzleType {
    VIOL_025 = 0,
    BLUE_030 = 1,
    RED_040 = 2
};




class SO_PressSens_Backend;

class PressSens
{
    friend class SO_PressSens_Backend;

public:
    PressSens();

    /* Do not allow copies */
    CLASS_NO_COPY(PressSens);

    // PressSens types
    enum class Type {
        NONE = 0,
        ANALOG = 1
    };


    enum class Status {
        NotConnected = 0,
        NoData,
        Good,
        SensorError
    };


    // The PressSens_State structure is filled in by the backend driver
    struct PressSens_State {
        float ml_accu;           // accumulated liquid [ml]
        float flow;              // estimated liquid flow [l/min]
        float pressure;          // measured pressure value
        float analog_val;        // measured analog value 
        enum PressSens::Status status;  // sensor status
        uint8_t  range_valid_count;     // number of consecutive valid readings (maxes out at 10)
        uint32_t last_reading_ms;       // system time of last successful update from sensor

        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[PRESS_SENS_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void set_log_rfnd_bit(uint32_t log_rfnd_bit) { _log_rfnd_bit = log_rfnd_bit; }

    /*
      Return the number of press sensor instances. 
      intended for future extension....
    */
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available press sensors 
    void init(void);

    // update state of all pressure sensors. Should be called at around
    // 10Hz from main loop
    void update(void);


    SO_PressSens_Backend *get_backend(uint8_t id) const;

    // get valve type for an ID
    NozzleType get_type(uint8_t id) const {
        return id >= PRESS_SENS_MAX_INSTANCES? NozzleType::VIOL_025 : NozzleType(params[id].valv_type.get());
    }

    // get valve amount
    uint8_t get_address(uint8_t id) const {
        return id >= PRESS_SENS_MAX_INSTANCES? 0 : uint8_t(params[id].valves.get());
    }
    
    // returns the PressSens measured value [kg / l]
    float get_measure(uint8_t id);

    // returns the PressSens status
    PressSens::Status get_status(uint8_t id);
    

    static PressSens *get_singleton(void) { return _singleton; }

protected:
    SO_PressSens_Params params[PRESS_SENS_MAX_INSTANCES];

private:
    static PressSens *_singleton;

    PressSens_State state[PRESS_SENS_MAX_INSTANCES];
    SO_PressSens_Backend *drivers[PRESS_SENS_MAX_INSTANCES];
    uint8_t num_instances;

    void detect_instance(uint8_t instance);
    bool _add_backend(SO_PressSens_Backend *driver, uint8_t instance);

    uint32_t _log_rfnd_bit = -1;
    void Log_RFND() const;

    HAL_Semaphore detect_sem;

};

namespace SO {
    PressSens *press_sens();
};
