#include <AP_HAL/AP_HAL.h>
#include "Soleon.h"
#include "SO_BkgProcessing.h"




// singleton instance
SoBkgProcessing *SoBkgProcessing::_singleton;

// object constructor.
SoBkgProcessing::SoBkgProcessing(void): g2(soleon.g2)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("SoBkgProcessing must be singleton");
    }

    _singleton = this;
}

//---------------------------------------------------------------------------------------------
// Monitor the tanklevel
// Fires warnmessage if the level is low
// shall be called from 1Sec background loop
void SoBkgProcessing::tanklevel_monitor(void)
{
    bool is_spraying = soleon.soleon_ctrl_mode->is_spraying();

    if (!is_spraying) {
        _level_cntr = 0;
        return;
    }

    //----- it is spraying -----
    if (g2.so_scale.get_measure(0) >= g2.user_parameters.get_dbg_0()){
        return; //-- above the level --> it's OK
    }
 
    if (_level_cntr == 0){
        gcs().send_text(MAV_SEVERITY_WARNING, "Tanklevel is low [<%0.2fl]", (float) g2.user_parameters.get_dbg_0());
    }

    if (++_level_cntr >= g2.user_parameters.get_dbg_1()) _level_cntr = 0;
}



SoBkgProcessing *AP::so_bkg_processing() {
    return SoBkgProcessing::get_singleton();
}
