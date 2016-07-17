// ------------------------------------------
//
//     Motor.h
//
//    h's simple Motor class: note that this class
//    uses constants defined in Config.h,
//    as it's intended to be used in conjunction
//    with the PID ControlLoop class. See
//    Config.h for more details.
//
// ------------------------------------------


#include <stdint.h>
#include "Config.h"

class Motor {
  private:
    bool motorEnabled; 
    
  public:
    Motor();
    void enableMotor();
    void disableMotor();
    void setPWM(float PWMlevel);
    
};  
