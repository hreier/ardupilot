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
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash

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
//    mp_cmd_t  _mp_cmd; 
    uint8_t _mp_cmd; 
    float _delta_fill;
    bool _mode_booting;

    //---- Soleon Spraycontroller (measured; calculated)
    float        _fill_level;
    
    // _mp_status:   Status bits
    #define MASK_STAT_SPR_RIGHT              (1<<0)
    #define MASK_STAT_SPR_LEFT               (1<<1)
    #define MASK_STAT_CTRL_READY             (1<<2)
    #define MASK_STAT_ERR_PMP                (1<<3)
    #define MASK_STAT_ERR_NOZZLE             (1<<4)
    uint8_t      _mp_status;
    
    uint32_t     _time_stamp; 
    RC_Channel::AuxSwitchPos _last_offset_trim_pos;
    float offset_trim_proz;
   

protected:
    virtual void override_ppm();
    virtual float modulate_value_trim(float in_value, float max_deviation);
    virtual bool bootsequence(void);
    virtual void manage_offset_trim(bool verbose);
    void updateSprayerValveRelays(uint8_t mp_cmd);
    void updateSprayerPumpPPMs(uint8_t mp_cmd, float ppm_left, float ppm_right, float ppm_off);


    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_speed;
    RC_Channel *&channel_offset;
    RC_Channel *&channel_override;
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

private:

    


};
