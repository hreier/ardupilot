/*
   This is pressure sensor backend implementation for 4-20mA analog sensors 
 */
 

#include "SO_PressSens_Analog.h"

#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define ANALOG_MAXMEAS_VAL      3723    //-- 3V at the pin (full adc: 3.3V --> 4095)
#define ANALOG_ZEROMEAS_VAL     744.6   //-- (3723*4/20)
#define ANALOG_RANGE            ANALOG_MAXMEAS_VAL-ANALOG_ZEROMEAS_VAL
#define ANALOG_SENSMISSING_VAL  350



SO_PressSens_Analog::SO_PressSens_Analog(PressSens::PressSens_State &_state,
        SO_PressSens_Params &_params)
    : SO_PressSens_Backend(_state, _params) {

    _analog_source = hal.analogin->channel(params.pin);
    };


/*
   Detects if a pressure sensor can be connected successfully. 
   
*/
SO_PressSens_Backend *SO_PressSens_Analog::detect(PressSens::PressSens_State &_state,
        SO_PressSens_Params &_params)
    {

    SO_PressSens_Analog *sensor = new SO_PressSens_Analog(_state, _params);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}



/* Driver initializes support for FX29 weight sensors 
 * driver is implemented as status machine
 */
bool SO_PressSens_Analog::init()
{
    // here we could init and register callback timer() 
    // not needed in that case

    return true;
}


/*
   update the state of the sensor
*/
void SO_PressSens_Analog::update(void)
{
    state.analog_val = _analog_source->read_latest();
    _time_stamp = AP_HAL::millis();

    if (state.analog_val < ANALOG_SENSMISSING_VAL)      state.status = PressSens::Status::NotConnected;
    else if (state.analog_val < ANALOG_ZEROMEAS_VAL-5)  state.status = PressSens::Status::NoData;
    else if (state.analog_val > ANALOG_MAXMEAS_VAL+5)   state.status = PressSens::Status::SensorError;
    else                                                state.status = PressSens::Status::Good;

    state.pressure = analog_to_bar (state.analog_val);
    state.flow = estimate_flow(state.pressure);
    state.ml_accu += calc_consumed_ml(state.flow, _time_stamp - state.last_reading_ms);
    state.last_reading_ms = _time_stamp;
}




//-- converts analog data to pressure value; --
float SO_PressSens_Analog::analog_to_bar (float analog_val)
{
    float ret_val;
    
    if (analog_val < ANALOG_ZEROMEAS_VAL)  ret_val = 0;
    else                                   ret_val = analog_val - ANALOG_ZEROMEAS_VAL;
    
    
    ret_val = (ret_val * params.range)/(ANALOG_RANGE);

    return ret_val;
}



