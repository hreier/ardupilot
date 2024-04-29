#pragma once

#include "SO_WeightSens_config.h"



#include "SO_WeightSens.h"
#include "SO_WeightSens_Backend.h"

#include <AP_HAL/I2CDevice.h>

#define HAL_WEIGHTSENSOR_I2C_BUS 0


enum class drv_mode { disabled = 0, goMEASURE = 1, setAdd0 = 10, setAdd1 = 11, setAdd2 = 12}; 


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
    uint8_t setAdd, scanAdd, foundAdd, add_cnt;
    uint16_t sens0_err, sens1_err, sens2_err;
    float sens0_row, sens1_row, sens2_row;
    float weight_filtered;
    uint32_t time_stamp;
    

private:
    // constructor
    SO_WeightSens_FX29_I2C(WeightSens::WeightSens_State &_state,
                                SO_WeightSens_Params &_params,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
                                

    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    bool init();

    void timer();

    // get a reading
    bool get_reading(float &reading_m);
    void data_log(uint16_t *val);

    //---- driver backend state machine
    typedef  void (SO_WeightSens_FX29_I2C::*timer_stm_ptr_t)();  
    timer_stm_ptr_t  timer_stm_ptr;

    void _stm_init(); 
    void _measure(); 
    void _measure_s0(); 
    void _measure_s1(); 
    void _measure_s2(); 

    float fx29_to_kgL (int16_t fx29_measure);
    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};


