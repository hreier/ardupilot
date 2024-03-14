#include <AP_HAL/AP_HAL.h>
#include "SO_RCMapper.h"

const AP_Param::GroupInfo SoRCMapper::var_info[] = {
    // @Param: SPEED
    // @DisplayName: Speed channel
    // @Description: Speed channel number. 1-16: channel for the veicle Speed signal
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SPEED",        1, SoRCMapper, _ch_speed, 1),
 
    // @Param: OFFSET
    // @DisplayName: Offset channel
    // @Description: Offset  channel number. 1-16: channel for the offset signal
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OFFSET",        2, SoRCMapper, _ch_offset, 2),

    // @Param: OVERRIDE
    // @DisplayName: Channel enable to override mission plan commands  
    // @Description: Override channel number. 1-16: channel for the override signal
    // @Range: 1 16
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OVERRIDE",       3, SoRCMapper, _ch_override, 3),


    AP_GROUPEND
};

// singleton instance
SoRCMapper *SoRCMapper::_singleton;

// object constructor.
SoRCMapper::SoRCMapper(void)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("SoRCMapper must be singleton");
    }
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

SoRCMapper *AP::so_rcmap() {
    return SoRCMapper::get_singleton();
}
