#ifndef MOTOR_H
#define MOTOR_H

#include "vex.h"
#include "robot_config.h"

using namespace vex;

extern brain Brain;
extern controller Controller1;

extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor_group LeftMotorGroup;

extern motor RightMotorA;
extern motor RightMotorB;
extern motor_group RightMotorGroup;

extern drivetrain Drive;
// Motor intake/outtake system - single motor on PORT9
extern motor IntakeOutake;

#endif

