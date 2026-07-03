#pragma once
#include "Soleon.h"
#include "UserParameters.h"

class Parameters;
class ParametersG2;



// --- SPRAY_PRESS 
class ModeCtrlSprayPress : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    ModeCtrlSprayPress(void);

    Number mode_number() const override { return Number::CTRL_SPRAY_PRESS; }

    bool init() override;
    void run() override;
    bool is_spraying() override; 

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

};



