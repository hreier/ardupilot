#pragma once
#include "Soleon.h"
#include "UserParameters.h"

class Parameters;
class ParametersG2;

typedef struct __copter_hud {
    uint32_t rx_time; 
    uint32_t rx_cnt;
    float air_speed;     //- Copter airspeed [m/sec]
} copter_hud_t;

// --- SPRAY_PRESS 
class ModeCtrlSprayFlow : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    ModeCtrlSprayFlow(void);

    Number mode_number() const override { return Number::CTRL_SPRAY_PRESS; }

    bool init() override;
    void run() override;
    bool is_spraying() override; 
    void updateCopterHudData(float air_speed);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "SPRAY_PRESS"; }
    const char *name4() const override { return "SPRE"; }

    bool should_be_spraying;
    uint8_t _mp_cmd_act;      //-- used for class internal processing 

    void configurePid(AC_PID *ptPid);
    void updateFromParameters(void);

    AC_PID   _pid_press_right {
        AC_PID::Defaults{
            .p         = SO_PRESS_RATE_RP_P,
            .i         = SO_PRESS_RATE_RP_I,
            .d         = SO_PRESS_RATE_RP_D,
            .ff        = SO_PRESS_RATE_RP_FF,
            .imax      = SO_PRESS_RATE_RP_IMAX,
            .filt_T_hz = SO_PRESS_RATE_RP_FILT_T_HZ,
            .filt_E_hz = SO_PRESS_RATE_RP_FILT_E_HZ,
            .filt_D_hz = SO_PRESS_RATE_RP_FILT_D_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID   _pid_press_left{
        AC_PID::Defaults{
            .p         = SO_PRESS_RATE_RP_P,
            .i         = SO_PRESS_RATE_RP_I,
            .d         = SO_PRESS_RATE_RP_D,
            .ff        = SO_PRESS_RATE_RP_FF,
            .imax      = SO_PRESS_RATE_RP_IMAX,
            .filt_T_hz = SO_PRESS_RATE_RP_FILT_T_HZ,
            .filt_E_hz = SO_PRESS_RATE_RP_FILT_E_HZ,
            .filt_D_hz = SO_PRESS_RATE_RP_FILT_D_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    float ctrl_p;
    float ctrl_d;
    float ctrl_i;
    float ctrl_imax;
    float ctrl_ff;
    float ctrl_filt_d;
    float ctrl_filt_e;
    float ctrl_filt_t;

    float ctrl_min_flow;

    copter_hud_t copter_hud;

};



