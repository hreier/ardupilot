#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class SoRCMapper {
public:
    SoRCMapper();

    /* Do not allow copies */
    CLASS_NO_COPY(SoRCMapper);

    // get singleton instance
    static SoRCMapper *get_singleton()
    {
        return _singleton;
    }

    /// speed - return input channel number for the vehicle speed  
    uint8_t speed() const { return _ch_speed; }

    /// offset - return input channel number for offset 
    uint8_t offset() const { return _ch_offset; }

    /// override - return input channel number for override
    uint8_t override() const { return _ch_override; }

    /// on_mode - return input channel number for on_mode (switch to define behavior of ON - override)
    uint8_t on_mode() const { return _ch_on_mode; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // channel mappings
    AP_Int8 _ch_offset;
    AP_Int8 _ch_override;
    AP_Int8 _ch_speed;
    AP_Int8 _ch_on_mode;
    static SoRCMapper *_singleton;
};

namespace AP
{
SoRCMapper *so_rcmap();
};
