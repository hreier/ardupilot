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
    if (temp2++> 30){  //-- debugging
        gcs().send_text(MAV_SEVERITY_INFO, "SoleonControlMode <%s> is running", name());  ///-HaRe debug
        temp2=0;
    }
  
}