/*
   This is weight module sensor backend implementation for the FX29 sensors with I2C interface
 */
 
#include "SO_WeightSens_FX29_I2C.h"


#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

#define FX29_I2C_ZEROMEAS_VAL  1000
#define FX29_I2C_MAXMEAS_VAL   15000


#define LIGHTWARE_DISTANCE_READ_REG 0
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG 22
#define LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG 23
#define LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE 20      // number of lost signal confirmations for legacy protocol only

#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100



SO_WeightSens_FX29_I2C::SO_WeightSens_FX29_I2C(WeightSens::WeightSens_State &_state,
        SO_WeightSens_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : SO_WeightSens_Backend(_state, _params)
    , _dev(std::move(dev)) {};


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




/* Driver first attempts to initialize the sf20.
 * If for any reason this fails, the driver attempts to initialize the legacy LightWare LiDAR.
 * If this fails, the driver returns false indicating no LightWare LiDAR is present.
 */
bool SO_WeightSens_FX29_I2C::init()
{
    if (legacy_init()) {
        DEV_PRINTF("Found FX29-weightsensors\n");
        return true;
    }
    DEV_PRINTF("FX29-weightsensors not found\n");
    return false;
}

/*
  initialise lidar using legacy 16 bit protocol
 */
bool SO_WeightSens_FX29_I2C::legacy_init()
{
    union {
        be16_t be16_val;
        uint8_t bytes[2];
    } timeout;

    // Retrieve lost signal timeout register
    const uint8_t read_reg = LIGHTWARE_LOST_SIGNAL_TIMEOUT_READ_REG;
    if (!_dev->transfer(&read_reg, 1, timeout.bytes, 2)) {
        return false;
    }

    // Check lost signal timeout register against desired value and write it if it does not match
    if (be16toh(timeout.be16_val) != LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE) {
        timeout.be16_val = htobe16(LIGHTWARE_TIMEOUT_REG_DESIRED_VALUE);
        const uint8_t send_buf[3] = {LIGHTWARE_LOST_SIGNAL_TIMEOUT_WRITE_REG, timeout.bytes[0], timeout.bytes[1]};
        if (!_dev->transfer(send_buf, sizeof(send_buf), nullptr, 0)) {
            return false;
        }
    }

    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&SO_WeightSens_FX29_I2C::legacy_timer, void));

    return true;
}



// read - return last value measured by sensor
bool SO_WeightSens_FX29_I2C::legacy_get_reading(float &reading_m)
{
    be16_t val;

    const uint8_t read_reg = LIGHTWARE_DISTANCE_READ_REG;

    // read the high and low byte distance registers
    if (_dev->transfer(&read_reg, 1, (uint8_t *)&val, sizeof(val))) {
        int16_t signed_val = int16_t(be16toh(val));
        if (signed_val < 0) {
            // some lidar firmwares will return 65436 for out of range
          //  reading_m = uint16_t(max_distance_cm() + LIGHTWARE_OUT_OF_RANGE_ADD_CM) * 0.01f;
        } else {
            reading_m = uint16_t(signed_val) * 0.01f;
        }
        return true;
    }
    return false;
}



/*
   update the state of the sensor
*/
void SO_WeightSens_FX29_I2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void SO_WeightSens_FX29_I2C::legacy_timer(void)
{
    if (legacy_get_reading(state.weight_n)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(WeightSens::Status::NoData);
    }
}


