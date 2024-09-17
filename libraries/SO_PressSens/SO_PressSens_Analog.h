#pragma once

#include "SO_PressSens_config.h"

#include "SO_PressSens.h"
#include "SO_PressSens_Backend.h"

#include <AP_HAL/I2CDevice.h>



class SO_PressSens_Analog : public SO_PressSens_Backend
{

public:
    // static detection function
    static SO_PressSens_Backend *detect(PressSens::PressSens_State &_state,
                                          SO_PressSens_Params &_params);

    // update state
    void update(void) override;

protected:
    uint32_t _time_stamp;
    //float _analog_val;

    AP_HAL::AnalogSource *_analog_source;


private:
    // constructor
    SO_PressSens_Analog(PressSens::PressSens_State &_state,
                                SO_PressSens_Params &_params);
                                

    bool init();


    // get a reading
  //  bool get_reading(float &reading_m);
  //  void data_log(uint16_t *val);


    float analog_to_bar (float analog_val);
};


