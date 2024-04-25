/*
   This is Soleon payload weight measurement 
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "SO_WeightSens.h"

class SO_WeightSens_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    SO_WeightSens_Backend(WeightSens::WeightSens_State &_state, SO_WeightSens_Params &_params);

    // we declare a virtual destructor so that WeightSens drivers can
    // override with a custom destructor if need be
    virtual ~SO_WeightSens_Backend(void) {}

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    virtual void handle_msg(const mavlink_message_t &msg) { return; }

#if AP_SCRIPTING_ENABLED
    // Returns false if scripting backing hasn't been setup
    // Get distance from lua script
    virtual bool handle_script_msg(float dist_m) { return false; }
#endif


    WeightSens::Status status() const;
    WeightSens::Type type() const { return (WeightSens::Type)params.type.get(); }

    // true if sensor is returning data
    bool has_data() const;

    float get_measure();
    char * get_gcs_message() {msg_updated = false; return gcs_message;}
    bool is_new_gcs_message() {return msg_updated;}

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

    // return the actual type of the weightsensor, as opposed to the
    // parameter value which may be changed at runtime.
    WeightSens::Type allocated_type() const { return _backend_type; }

protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(WeightSens::Status status);

    bool set_gcs_message(const char * msg);

    WeightSens::WeightSens_State &state;
    SO_WeightSens_Params &params;

    char gcs_message[100];
    bool msg_updated;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    WeightSens::Type _backend_type;

};
