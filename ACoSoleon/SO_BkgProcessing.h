/*
This collects background processing stuff for Soleon payload
*/
#pragma once
#include "Soleon.h"

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class ParametersG2;


class SoBkgProcessing {

public:
    SoBkgProcessing();

    void tanklevel_monitor(void);

    /* Do not allow copies */
    CLASS_NO_COPY(SoBkgProcessing);

    // get singleton instance
    static SoBkgProcessing *get_singleton()
    {
        return _singleton;
    }


 
    private:
    ParametersG2 &g2;
    int _level_cntr;

    static SoBkgProcessing *_singleton;
};

namespace AP
{
SoBkgProcessing *so_bkg_processing();
};
