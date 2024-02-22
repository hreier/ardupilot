#pragma once

#include "SO_WeightSens_config.h"



#include "SO_WeightSens.h"
#include "SO_WeightSens_Backend.h"

#include <AP_HAL/I2CDevice.h>

#define NUM_SF20_DATA_STREAMS 1

class SO_WeightSens_FX29_I2C : public SO_WeightSens_Backend
{

public:
    // static detection function
    static SO_WeightSens_Backend *detect(WeightSens::WeightSens_State &_state,
                                          SO_WeightSens_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

protected:


private:
    // constructor
    SO_WeightSens_FX29_I2C(WeightSens::WeightSens_State &_state,
                                SO_WeightSens_Params &_params,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
                                

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    bool init();
    bool legacy_init();
    void legacy_timer();

    // get a reading
    bool legacy_get_reading(float &reading_m);
    void data_log(uint16_t *val);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};


