#include "Soleon.h"



// Controller disabled - initialise the disabled controller mode
bool ModeCtrlDisabled::init()
{
    gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode init: <%s>", name()); //-- the activation routine send similar message

    return true;
}


// Controller disabled - runs the disabled controller mode
void ModeCtrlDisabled::run()
{
    static int temp2;
    if (temp2++> 150){  //-- debugging
        gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode <%s> is running", name());  ///-HaRe debug
        //gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode <%f> is running", SO::TankSupervision()->get_level());  ///-HaRe debug
        temp2=0;
    }
  
}