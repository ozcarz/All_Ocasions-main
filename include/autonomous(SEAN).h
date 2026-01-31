// Declaration-only header for autonomous routines
#pragma once
#include "vex.h"
#include "motor.h"
#include "sensors.h"

using namespace vex;

// Motion helper prototypes. Implementations live in src/autonomous.cpp
void totalDistance(double mm);
void turnLeft(double degrees);
void REVERSAL1(double mm);

// Autonomous callback used by the competition framework
void autonomous();
