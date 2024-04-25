/*
   This is weight module sensor backend implementation for the FX29 sensors with I2C interface
 */
 

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
        
    case drv_mode::setAdd0:
    case drv_mode::setAdd1:
    case drv_mode::setAdd2:
        setAdd = (int8_t)act_mode - (int8_t) drv_mode::setAdd0 + params.address;
        set_status(WeightSens::Status::setAdd);
        
        char * str;
        asprintf(&str, "Set I2C-address %u: disconnect all devices", setAdd);
        set_gcs_message(str);
        
        timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add;
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
    //-- HaRe todo: add trigger for the other two sensors
    
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure_s0;
}

//-- read measured data from sensor 0 ---
void SO_WeightSens_FX29_I2C::_measure_s0()
{
    be16_t val;

    // read the high and low byte distance registers
    if (_dev->transfer(0, 0, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        signed_val &= 0x3FFF;  //mask bit 15 and 14; status info
        sens0_row = fx29_to_kgL(signed_val);

        if (sens0_err > 0) sens0_err --;
    }
    else{
        if (sens0_err < SENS_ERR_LIMIT_CNT) sens0_err += 2;
    }

    //-- HaRe todo: replace this with measurement of the other sensors (load machine _measure_s1/_measure_s2)
    state.weight_kg = sens0_row;

    if ((drv_mode)((int) params.mode) != drv_mode::goMEASURE)  timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_stm_init;
    else                                                       timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_measure;
}


//----- set the I2C address init --
void SO_WeightSens_FX29_I2C::_set_add()
{ //- init the set address module 
    time_stamp = AP_HAL::millis();
    scanAdd =  setAdd;
    foundAdd = setAdd;
    add_cnt = 0;

    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_disc_all;
}

//----- set the I2C address disconnect all --
void SO_WeightSens_FX29_I2C::_set_add_disc_all()
{ //- wait for all I2C devices being disconnected
    uint8_t buf[2];
    
    _dev->set_address(scanAdd);
    if (_dev->transfer(0, 0, buf, 2))
    {  //-- this one is connected 
        foundAdd = scanAdd;
        checkTimeoutSetAddr();
        return;
    }

    time_stamp = AP_HAL::millis();
    if (scanAdd < 127) scanAdd++;
    else               scanAdd = 0;

    if (scanAdd == foundAdd) 
    {
        timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_conn;
        set_gcs_message("Now connect the weight sensor");
        return;
    }
};

//----- set the I2C address: wait for one sensor to be connected --
void SO_WeightSens_FX29_I2C::_set_add_conn()
{
    uint8_t buf[2];
    
    _dev->set_address(scanAdd);
    if (!_dev->transfer(0, 0, buf, 2))
    {  //-- this is not connected 
        if (scanAdd < 127) scanAdd++;
        else               scanAdd = 0;
        //foundAdd = scanAdd;
        checkTimeoutSetAddr();
        return;
    }

    char * str;
    
    if (scanAdd == setAdd)
    {
        asprintf(&str, "Sensor found [addr:%u]; DONE!!", scanAdd);
        set_gcs_message(str);
        timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_done;
        return;
    }

    time_stamp = AP_HAL::millis();
    foundAdd = scanAdd;
    
    asprintf(&str, "Sensor found [addr:%u]; disconnect once again!!!", scanAdd);
    set_gcs_message(str);
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_cmd;
}


//----- set the I2C address: wait until disconnected --
void SO_WeightSens_FX29_I2C::_set_add_cmd()
{
    uint8_t buf[2];
    
    _dev->set_address(scanAdd);
    if (_dev->transfer(0, 0, buf, 2))
    {  //-- not yet disconnected 
        checkTimeoutSetAddr();
        return;
    }

    time_stamp = AP_HAL::millis();

    set_gcs_message("And again CONNECT!!!");
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_cmd1;
}
    
//--- now enter into command mode ---
void SO_WeightSens_FX29_I2C::_set_add_cmd1()
{
    uint8_t buf[3];

    //-- enter command mode within 6mSec after sensor power on --
    buf[0] = 0xA0;   //- enter command mode
    buf[1] = 0x0;
    buf[2] = 0x0;

    while (!_dev->transfer(buf, 3, 0, 0))
    {  //-- wait for power cycle --
        if (checkTimeoutSetAddr()) return;
    }

    _dev->transfer(buf, 3, 0, 0);
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_cmd2;
}
    
//--- command mode active; read the eeprom word 2 ---
void SO_WeightSens_FX29_I2C::_set_add_cmd2()
{
    uint8_t buf[3];

    buf[0] = 0x2;   //- eeprom word 2
    buf[1] = 0x0;
    buf[2] = 0x0;

    _dev->transfer(buf, 3, 0, 0);

    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_cmd3;
}

//--- fetch the eeprom word 2 ---
void SO_WeightSens_FX29_I2C::_set_add_cmd3()
{
    uint8_t buf[3];

    _dev->transfer(0, 0, buf, 3);

    char * str;
    asprintf(&str, "EEProm: %X; %X; %X", buf[0], buf[1], buf[2]);
    set_gcs_message(str);
   
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_done;
}


//---- set the I2C address: done --> waiting for configuration change of mode --
/*void SO_WeightSens_FX29_I2C::_set_add_successful()
{
set_gcs_message("Successful!!!");
timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_done;
}*/

/*    
    void _set_add_disc();
    void _set_add_reconn();*/


//---- set the I2C address: done --> waiting for configuration change of mode --
void SO_WeightSens_FX29_I2C::_set_add_done()
{
    switch ((drv_mode)((int) params.mode))
    {
    case drv_mode::setAdd0:
    case drv_mode::setAdd1:
    case drv_mode::setAdd2:
        break;

    default:
        set_status(WeightSens::Status::NoData);
        timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_stm_init;
        break;
    }
}



#define I2C_ADD_TIMEOUT  25000
//-- watch the timeout and reset the state machine if stucking
//- returns true if timeouted
bool SO_WeightSens_FX29_I2C::checkTimeoutSetAddr()
{
    if ((time_stamp + I2C_ADD_TIMEOUT) >= AP_HAL::millis()) return false;
    
    set_gcs_message("ERROR: TimeOut!!!");
    timer_stm_ptr =  &SO_WeightSens_FX29_I2C::_set_add_done;
    return true; 
}


//-- converts fx28 readed data into kg liter ---
float SO_WeightSens_FX29_I2C::fx29_to_kgL (int16_t fx29_measure)
{
    float ret_val = fx29_measure - FX29_I2C_ZEROMEAS_VAL;
    ret_val *= FX29_SENSOR_SCALE;
    ret_val = ret_val/((FX29_I2C_MAXMEAS_VAL-FX29_I2C_ZEROMEAS_VAL)*GRAVITATIONAL_ACCEL);
    return ret_val;
}



