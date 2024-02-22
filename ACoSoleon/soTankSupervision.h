#pragma once
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>



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

    //---- from missionplan ----
    float    _mp_liter_ha, _mp_line_dist, _mp_planned_spd, _mp_dist_waypoint, _mp_sprayrate;
    uint8_t  _mp_cmd; uint8_t  _mp_status;

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

