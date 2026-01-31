#include "vex.h"
#include "robot_config.h"
#include "motor.h"
#include "sensors.h"

using namespace vex;

// ---- Core ----
brain Brain;
controller Controller1 = controller(primary);

// ---- Drive ----
motor LeftMotorA  = motor(PORT1, DRIVE_GEAR_RATIO, config::LEFT_MOTOR_A_REVERSED);
motor LeftMotorB  = motor(PORT2, DRIVE_GEAR_RATIO, config::RIGHT_MOTOR_A_REVERSED);
motor_group LeftMotorGroup = motor_group(LeftMotorA, LeftMotorB);

motor RightMotorA = motor(PORT3, DRIVE_GEAR_RATIO, config::LEFT_MOTOR_B_REVERSED);
motor RightMotorB = motor(PORT4, DRIVE_GEAR_RATIO, config::RIGHT_MOTOR_B_REVERSED);
motor_group RightMotorGroup = motor_group(RightMotorA, RightMotorB);

drivetrain Drive(LeftMotorGroup, RightMotorGroup,
                 config::WHEEL_DIAMETER_MM, config::TRACK_WIDTH_MM,
                 config::WHEELBASE_MM, mm, config::EXTERNAL_GEAR_RATIO);

// ---- Intake/Outake ----
motor IntakeOutake = motor(PORT9, ratio18_1, false);

// ---- Sensors (DIRECT PORTS; no config port constants) ----    
// Change these if your wiring differs:
// Inertial sensor moved from PORT1 -> PORT12 to free PORT1 for left motor
inertial inertial_sensor = inertial(PORT12);
rotation verticalRot     = rotation(PORT9, false);
rotation horizontalRot   = rotation(PORT6, false);
