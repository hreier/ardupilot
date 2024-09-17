/*
   This is Soleon payload pressure measurement and flow estimation
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "SO_PressSens.h"

#define PRESS_TABSIZE  20

typedef struct {
   NozzleType type;             //- type code
   char  type_str[16];          //- type string
   float tab[PRESS_TABSIZE];    //- table
} _press_tab_t;


class SO_PressSens_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    SO_PressSens_Backend(PressSens::PressSens_State &_state, SO_PressSens_Params &_params);

    // we declare a virtual destructor so that PressSens drivers can
    // override with a custom destructor if need be
    virtual ~SO_PressSens_Backend(void) {}

    // update the state structure
    virtual void update() = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    virtual void handle_msg(const mavlink_message_t &msg) { return; }

#if AP_SCRIPTING_ENABLED
    // Returns false if scripting backing hasn't been setup
    // Get distance from lua script
    virtual bool handle_script_msg(float dist_m) { return false; }
#endif


    PressSens::Status status() const;
    PressSens::Type type() const { return PressSens::Type::ANALOG; }  //- we only support analog sensors (no parameter provides for that);  

    // true if sensor is returning data
    bool has_data() const;

    float get_measure();
    char * get_gcs_message() {msg_updated = false; return gcs_message;}
    bool is_new_gcs_message() {return msg_updated;}

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

    // return the actual type of the pressure sensor, as opposed to the
    // parameter value which may be changed at runtime.
    PressSens::Type allocated_type() const { return _backend_type; }

    PressSens::PressSens_State &state; //-- debugging weg hier; protected

protected:

    // update status 
    void update_status();
    float estimate_flow(float pressure);
    float calc_consumed_ml (float flow, float dt_msec);


    // set status and update valid_count
    void set_status(PressSens::Status status);

    bool set_gcs_message(const char * msg);

 //debugging   PressSens::PressSens_State &state;
    SO_PressSens_Params &params;

    char gcs_message[100];
    bool msg_updated;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //------------------------------------------------------
    //- the nozzle tables for linear interpolation 
    //- tab[0] --> flow [l/min] @ 1Bar;  tab[19] --> flow @ 20Bar; 
    
    _press_tab_t nozzle_tabs[3] = {
        {//- table: 90-025-viol
        .type = NozzleType::VIOL_025,
        {.type_str = "90-025-viol"},
        .tab = {0.51f, 0.81f, 0.99f, 1.15f, 1.28f, 1.40f, 1.52f, 1.62f, 1.71f, 1.81f, 
                1.90f, 1.98f, 2.06f, 2.14f, 2.21f, 2.29f, 2.36f, 2.43f, 2.49f, 2.56f,}
        },

        {//- table: 90-030-blue
        .type = NozzleType::BLUE_030,
        {.type_str = "90-030-blue"},
        .tab = {0.63f, 0.97f, 1.19f, 1.37f, 1.53f, 1.68f, 1.81f, 1.94f, 2.06f, 2.17f,
                2.28f, 2.38f, 2.48f, 2.57f, 2.66f, 2.75f, 2.83f, 2.91f, 2.99f, 3.07f,}
        },

        {//- table: 90-040-red
        .type = NozzleType::RED_040,
        {.type_str = "90-040-red"},
        .tab = {0.86f, 1.29f, 1.58f, 1.82f, 2.04f, 2.23f, 2.41f, 2.58f, 2.74f, 2.88f, 
                3.03f, 3.16f, 3.29f, 3.41f, 3.53f, 3.65f, 3.76f, 3.87f, 3.98f, 4.08f,}
        },
    };


    //Type Backend initialised with
    PressSens::Type _backend_type;

};
