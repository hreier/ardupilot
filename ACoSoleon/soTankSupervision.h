#pragma once
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>


//#include "Soleon.h"
//#include <AP_Math/chirp.h>
//class Parameters;
//class ParametersG2;

//class GCS_Copter;
//#define MAVLINK_MSG_ID_SCALED_PRESSURE 29
//#define MAVLINK_MSG_ID_MCU_STATUS 11039

#define MAVLINK_MSG_ID_SO_STATUS  50080


class SO_TankSupervision {
public:
    SO_TankSupervision();
    bool init(bool ignore_checks) ;
    void run() ;
    void update();
    void set(float val);
    float get_level() const { return _fill_level; }


    static SO_TankSupervision *get_singleton() {
        return _singleton;
    }


protected:

    const char *name() { return "SOTANK"; }
    const char *name4() { return "TANK"; }

private:

    float _fill_level, _delta_fill;
    uint32_t _timeout_start;
    uint32_t _timeout_ms;

    static SO_TankSupervision *_singleton;

};

namespace SO {
    SO_TankSupervision *TankSupervision();
};

