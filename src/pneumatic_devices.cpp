#include "pneumatic_devices.h"
#include "motor.h"   // for extern Brain

// CHANGE H to whatever 3-wire port your solenoid is plugged into
vex::digital_out wingsPiston = vex::digital_out(Brain.ThreeWirePort.B);

// This is the missing definition that fixes your linker error
PneumaticController pneumaticWings(wingsPiston);
