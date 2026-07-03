/*
   This is Soleon flow measurement sensor:
   > implements a flow measurement that RPM measurement module as backend (RPM1 and RPM2)
   > Scaling setup of the RPM1/2 needs to be made to represent ml/min (rpm1/2_scaling = 1.2)
 */

#include "Soleon.h"
#include "SO_FlowSens.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
//#include <AP_HAL/I2CDevice.h>

#include <GCS_MAVLink/GCS.h> //-testing

//-- uncomment for module debugging 
#define DEBUG_ME


extern const AP_HAL::HAL &hal;


SoFlowSens::SoFlowSens()
{
    _singleton = this;
}

/*
  initialise the SoFlowSens class.
 */
void SoFlowSens::init()
{
    state.last_update_ms = AP_HAL::millis();
    ml_accu_internal = 0;
#ifdef DEBUG_ME
    gcs().send_text(MAV_SEVERITY_WARNING, "Soleon flow measurement starting up");
#endif
}

/*
  update FlowSens state.
  This should be called at around 10Hz
 */

#ifdef DEBUG_ME
int dbg_cnt; //--debug
#endif
void SoFlowSens::update(void)
{
    uint32_t timeStamp = AP_HAL::millis();
    uint32_t delta_ms;
    float fl_r, fl_l;

    delta_ms = timeStamp - state.last_update_ms;
    ml_accu_internal += state.flow * delta_ms;
    state.last_update_ms = timeStamp;
    state.ml_accu = ml_accu_internal/60000;      //- convert to ml
    state.healthy_right = soleon.rpm_sensor.get_rpm(RIGHT_SENS_ID, fl_r);
    state.healthy_left  = soleon.rpm_sensor.get_rpm(LEFT_SENS_ID, fl_l);
    if (fl_r > 0) state.flow_right = fl_r;
    else          state.flow_right = 0;
    if (fl_l > 0) state.flow_left = fl_l;
    else          state.flow_left = 0;
    state.flow = state.flow_right + state.flow_left;
    
#if HAL_LOGGING_ENABLED
    Log_FlowMeasurements();
#endif
//------------debug---
#ifdef DEBUG_ME
if (dbg_cnt++ > 50)
  {
    dbg_cnt = 0;

    gcs().send_text(MAV_SEVERITY_WARNING, "flow[ml/min]=%.2f; accu[ml]=%.2f; fl_r=%.2f; fl_l=%.2f;", state.flow, state.ml_accu, state.flow_right, state.flow_left);
  }
#endif
}


// returns the SoFlowSens status
SoFlowSens::SoFlowSens_State SoFlowSens::get_status()
{
    return state;
}



// Write an packet from flow sensors
void SoFlowSens::Log_FlowMeasurements() const
{    
    if (_log_flow_bit == uint32_t(-1)) {
        return;        
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_flow_bit)) {
        return;
    }

    AP::logger().Write("FLS", "TimeUS,flow,accu,fl_l,fl_r,hlt_l,hlt_r", "QffffBB",
        AP_HAL::micros64(),
        (double)state.flow,
        (double)state.ml_accu,
        (double)state.flow_right,
        (double)state.flow_left,
        (uint8_t)state.healthy_right,
        (uint8_t)state.healthy_left
        );
}



SoFlowSens *SoFlowSens::_singleton;

namespace SO {

SoFlowSens *flow_sens()
{
    return SoFlowSens::get_singleton();
}

}

