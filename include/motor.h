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
// Motor intake system orginally labeled as "Intake/Outake"
extern motor IntakeLeft;
extern motor IntakeRight;
extern motor Midtake;

extern motor RightOutake;
extern motor LeftOutake;
extern motor MidOutake;

#endif

