/*
   Soleon flow_sens class
 */
#pragma once

//#include "SO_PressSens_config.h" 

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
//#include "SO_PressSens_Params.h"
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega Flash Memory Library


#define FLOW_MODULE_LOGGING      //-- this activates logging


class SoFlowSens
{

public:
    SoFlowSens();

    /* Do not allow copies */
    CLASS_NO_COPY(SoFlowSens);



 /*     enum class Status {
        NotConnected = 0,
        NoData,
        Good,
        SensorError
    }; 
 */

    // The SoFlowSens_State structure is filled in by the backend driver
    struct SoFlowSens_State {
        float ml_accu;           // accumulated liquid [ml]
        float flow;              // actual liquid flow [ml/min]
        float flow_right; 
        float flow_left; 
        uint32_t last_update_ms;       // system time of last successful update from sensor
    };

    void set_log_bit_mask(uint32_t log_flow_bit) { _log_flow_bit = log_flow_bit; }


    // detect and initialise any available press sensors 
    void init(void);

    // update state of both flow sensors. Should be called every 100msec (especially important for accurate ml_accu calculation)
    void update(void);
    
    // returns the SoFlowSens measured flow value [ml/min]
    float get_measure() {return state.flow;};

    // returns the SoFlowSens status
    SoFlowSens::SoFlowSens_State get_status();
    

    static SoFlowSens *get_singleton(void) { return _singleton; }

protected:

private:
    static SoFlowSens *_singleton;

    SoFlowSens_State state;

    uint32_t _log_flow_bit = -1;
    void Log_FlowMeasurements() const;

    //HAL_Semaphore detect_sem;
    const uint8_t RIGHT_SENS_ID = 0;
    const uint8_t LEFT_SENS_ID = 1;

    float ml_accu_internal;   // this is [ml/min] * [ms];  must be devided by 60000 for represent [ml]

};

namespace SO {
    SoFlowSens *flow_sens();
};
