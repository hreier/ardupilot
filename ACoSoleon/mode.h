#pragma once

#include "Soleon.h"
#include <AP_Math/chirp.h>
class Parameters;
class ParametersG2;

class GCS_Soleon;

class Mode {

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        CTRL_DISABLED =     0,  // Soleon controller disabled
        CTRL_SPRAY_PPM =    1,  // Soleon spray controller generate ppm signals
        CTRL_TEST =         2,  // Soleon controller for test 


        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };
  
    // Missionplan command _mp_cmd
    // _mp_cmd:   Command bits
    #define MASK_CMD_SPR_RIGHT_REAR               (1<<0)
    #define MASK_CMD_SPR_LEFT_REAR                (1<<1)
    #define MASK_CMD_SPR_RIGHT_FRONT              (1<<2)
    #define MASK_CMD_SPR_LEFT_FRONT               (1<<3)
    #define MASK_CMD_SPR_ACTIVE                (MASK_CMD_SPR_RIGHT_FRONT | MASK_CMD_SPR_RIGHT_REAR | MASK_CMD_SPR_LEFT_FRONT | MASK_CMD_SPR_LEFT_REAR)
    #define MASK_CMD_PUMP_RIGHT                (MASK_CMD_SPR_RIGHT_FRONT | MASK_CMD_SPR_RIGHT_REAR)
    #define MASK_CMD_PUMP_LEFT                 (MASK_CMD_SPR_LEFT_FRONT | MASK_CMD_SPR_LEFT_REAR)
    #define MASK_CMD_PUMP_FRONT                (MASK_CMD_SPR_RIGHT_FRONT | MASK_CMD_SPR_LEFT_FRONT)
    #define MASK_CMD_SPR_REAR                  (MASK_CMD_SPR_RIGHT_REAR | MASK_CMD_SPR_LEFT_REAR)


    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    virtual bool init() {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool is_spraying() = 0;


    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

//private:
    //---- Soleon Spraycontrollers (mostly from missionplan) ----
    float    _mp_liter_ha, _mp_line_dist, _mp_planned_spd, _mp_dist_waypoint, _mp_sprayrate, _ppm_pump;
    uint8_t _mp_cmd; 
    float _delta_fill;
    bool _mode_booting;

    //---- Soleon Spraycontroller (measured; calculated)
    float        _fill_level;
    
    // _mp_status:   Status bits
    #define MASK_STAT_SPR_RIGHT_REAR         (1<<0)
    #define MASK_STAT_SPR_LEFT_REAR          (1<<1)
    #define MASK_STAT_CTRL_READY             (1<<2)
    #define MASK_STAT_ERR_PMP                (1<<3)
    #define MASK_STAT_ERR_NOZZLE             (1<<4)
    #define MASK_STAT_SPR_RIGHT_FRONT        (1<<6)
    #define MASK_STAT_SPR_LEFT_FRONT         (1<<7)

    uint8_t      _mp_status;
    
    uint32_t     _time_stamp; 
    RC_Channel::AuxSwitchPos _last_offset_trim_pos;
    float offset_trim_proz;
   

protected:
    virtual void overrideBySwitch(uint8_t & command, uint8_t & status);
    virtual float modulate_value_trim(float in_value, float max_deviation);
    virtual bool bootsequence(void);
    virtual void manage_offset_trim(bool verbose);
    void updateSprayerValveRelays(uint8_t mp_cmd);
    void updateSprayerPumpPPMs(uint8_t mp_cmd, float ppm_left, float ppm_right, float ppm_off);
    void updateSprayerStatus(uint8_t mp_cmd, uint8_t & mp_status);


    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    RC_Channel *&channel_speed;
    RC_Channel *&channel_offset;
    RC_Channel *&channel_override;
    RC_Channel *&channel_on_mode;
    float &G_Dt;

    
public:
   GCS_Soleon &gcs();
};


// --- DISABLED
class ModeCtrlDisabled : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CTRL_DISABLED; }

    bool init() override;
    void run() override;
    bool is_spraying() override; 


protected:

    const char *name() const override { return "DISABLED"; }
    const char *name4() const override { return "DIS"; }
};


// --- SPRAY_PPM 
class ModeCtrlSprayPPM : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CTRL_SPRAY_PPM; }

    bool init() override;
    void run() override;
    bool is_spraying() override; 


protected:

    const char *name() const override { return "SPRAY_PPM"; }
    const char *name4() const override { return "SPPM"; }

    bool should_be_spraying;
    uint8_t _mp_cmd_act;      //-- used for class internal processing 

};



// --- Control module for testing  
class ModeCtrlTest : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CTRL_TEST; }

    bool init() override;
    void run() override;
    bool is_spraying() override; 


protected:

    const char *name() const override { return "CTR_TESTING"; }
    const char *name4() const override { return "TEST"; }

    uint8_t _mp_cmd_act;

private:

    


};
