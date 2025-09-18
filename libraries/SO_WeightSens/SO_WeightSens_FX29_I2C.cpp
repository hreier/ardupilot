/*
   This is weight module sensor backend implementation for the FX29 sensors with I2C interface
 */
 
#include <GCS_MAVLink/GCS.h>

#include "SO_WeightSens_FX29_I2C.h"

#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define FX29_I2C_ZEROMEAS_VAL  1000
#define FX29_I2C_MAXMEAS_VAL   15000
#define FX29_SENSOR_SCALE      500
#define GRAVITATIONAL_ACCEL    9.81

#define SENS_ERR_LIMIT_CNT     20


SO_WeightSens_FX29_I2C::SO_WeightSens_FX29_I2C(WeightSens::WeightSens_State &_state,
        SO_WeightSens_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : SO_WeightSens_Backend(_state, _params)
    , _dev(std::move(dev)) {

     //   AP_Param::setup_object_defaults(this, var_info);
     //   state.var_info = var_info;

    };


/*
   Detects if a weightsensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
SO_WeightSens_Backend *SO_WeightSens_FX29_I2C::detect(WeightSens::WeightSens_State &_state,
        SO_WeightSens_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    SO_WeightSens_FX29_I2C *sensor = new SO_WeightSens_FX29_I2C(_state, _params, std::move(dev));

    if (!sensor) {
        return nullptr;
    }

    WITH_SEMAPHORE(sensor->_dev->get_semaphore());
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/**
 * Wrapper function over #transfer() to write a sequence of bytes to
 * device. No values are read.
 */
bool SO_WeightSens_FX29_I2C::write_bytes(uint8_t *write_buf_u8, uint32_t len_u8)
{
    return _dev->transfer(write_buf_u8, len_u8, NULL, 0);
}




/* Driver initializes support for FX29 weight sensors 
 * driver is implemented as status machine
 */
bool SO_WeightSens_FX29_I2C::init()
{
    // init and register callback timer() at 20Hz/50mSec
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_stm_init;
    _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&SO_WeightSens_FX29_I2C::timer, void));

    return true;
}


/*
   update the state of the sensor
*/
void SO_WeightSens_FX29_I2C::update(void)
{
    // nothing to do - its all done in the timer()
}


void SO_WeightSens_FX29_I2C::timer(void)
{
    (this->*timer_stm_ptr)();
}

//-----------------------------------------
//--------- driver state machine ----------

//----- init/restart the status machine
void SO_WeightSens_FX29_I2C::_stm_init()
{
  drv_mode act_mode = (drv_mode)((int) params.mode);

  switch (act_mode)
    {
    case drv_mode::goMEASURE:
        timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure;
        break;

    
    default: 
        set_status(WeightSens::Status::NoData);

    case drv_mode::disabled:    //- stay here
        break;
    }

}


//-- start measurement of all sensors---
void SO_WeightSens_FX29_I2C::_measure()
{   //-- weak up from power down and start measurement
    uint8_t buf[2];
    
    _dev->set_address(params.address);
    _dev->transfer(0, 0, buf, 0);

    _dev->set_address(params.address+1);
    _dev->transfer(0, 0, buf, 0);
 
    _dev->set_address(params.address+2);
    _dev->transfer(0, 0, buf, 0);

    do_LOG();   //-- log the data
    
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure_s0;
}

//-- read measured data from sensor 0 ---
void SO_WeightSens_FX29_I2C::_measure_s0()
{
    be16_t val;

    _dev->set_address(params.address);

    if (_dev->transfer(0, 0, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        signed_val &= 0x3FFF;  //mask bit 15 and 14; status info
        sens0_row = fx29_to_kgL(signed_val);

        if (sens0_err > 0) sens0_err --;
    }
    else{
        if (sens0_err < SENS_ERR_LIMIT_CNT) sens0_err += 2;
    }


    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure_s1;
}


//-- read measured data from sensor 1 ---
void SO_WeightSens_FX29_I2C::_measure_s1()
{
    be16_t val;

    _dev->set_address(params.address+1);
    
    if (_dev->transfer(0, 0, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        signed_val &= 0x3FFF;  //mask bit 15 and 14; status info
        sens1_row = fx29_to_kgL(signed_val);

        if (sens1_err > 0) sens1_err --;
    }
    else{
        if (sens1_err < SENS_ERR_LIMIT_CNT) sens1_err += 2;
    }

    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure_s2;
}

//-- read measured data from sensor 2 ---
//#define FILT_LENGHT  3 -> 32 -> 64 -> 15
#define FILT_LENGHT  15
void SO_WeightSens_FX29_I2C::_measure_s2()
{
    be16_t val;
    

    _dev->set_address(params.address+2);
    
    if (_dev->transfer(0, 0, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        signed_val &= 0x3FFF;  //mask bit 15 and 14; status info
        sens2_row = fx29_to_kgL(signed_val);

        if (sens2_err > 0) sens2_err --;
    }
    else{
        if (sens2_err < SENS_ERR_LIMIT_CNT) sens2_err += 2;
    }

    //out = (out*FILT_LENGHT + in) / (FILT_LENGHT+1); //FILT_LENGHT == 0 --> without filtering
    weight_filtered = weight_filtered * FILT_LENGHT + sens0_row  + sens1_row + sens2_row;
    weight_filtered = weight_filtered / (FILT_LENGHT+1);
    state.weight_kg = weight_filtered - params.offset;

    if ((drv_mode)((int) params.mode) != drv_mode::goMEASURE)  timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_stm_init;
    else                                                       timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure;
}



//-- converts fx28 readed data into kg liter ---
float SO_WeightSens_FX29_I2C::fx29_to_kgL (int16_t fx29_measure)
{
    float ret_val = fx29_measure - FX29_I2C_ZEROMEAS_VAL;
    ret_val *= FX29_SENSOR_SCALE;
    ret_val = ret_val/((FX29_I2C_MAXMEAS_VAL-FX29_I2C_ZEROMEAS_VAL)*GRAVITATIONAL_ACCEL);
    return ret_val;
}


// Logging the FX29_I2C sensor 
void SO_WeightSens_FX29_I2C::do_LOG()
{
    uint32_t log_bit_mask = SO::weightsens()->get_log_bit_mask();

   // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "do bin i: %d", (int)log_bit_mask);

    if (log_bit_mask == uint32_t(-1)) {
        return;       
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(log_bit_mask)) {
        return;
    }
    
    AP::logger().Write("SCAL", "TimeUS,OutFilt,RowSum,Row0,Row1,Row2,err0,err1,err2", "QfffffIII",
            AP_HAL::micros64(),
            (double)state.weight_kg,
            (double)(sens0_row+sens1_row+sens2_row),
            (double)sens0_row,
            (double)sens1_row,
            (double)sens2_row,
            (double)sens0_err,
            (double)sens1_err,
            (double)sens2_err
            );

 }
