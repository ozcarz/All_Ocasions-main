#ifndef PRE_AUTON_H
#define PRE_AUTON_H

#include "vex.h"
#include "motor.h"
#include "sensors.h"

using namespace vex;

inline void pre_auton(void) {
    initSensors(Brain, Controller1);

    Brain.Screen.clearScreen();
    Brain.Screen.setFont(fontType::mono20);
    Brain.Screen.setCursor(2, 2);
    Brain.Screen.print("Robot Initialized");
    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print("Ready for Competition");

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Ready");
}

#endif



