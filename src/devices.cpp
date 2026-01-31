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
motor IntakeLeft  = motor(PORT16, INTAKE_RATIO, config::INTAKE_LEFT_REVERSED);
motor IntakeRight = motor(PORT5,  INTAKE_RATIO, config::INTAKE_RIGHT_REVERSED);
motor Midtake     = motor(PORT8,  ratio18_1,    config::MIDTAKE_REVERSED);

motor RightOutake = motor(PORT7,  OUTAKE_RATIO, config::RIGHT_OUTAKE_REVERSED);
// LeftOutake moved from PORT2 -> PORT14 to free PORT2 for left motor
motor LeftOutake  = motor(PORT14, OUTAKE_RATIO, config::LEFT_OUTAKE_REVERSED);
motor MidOutake   = motor(PORT20, OUTAKE_RATIO, config::MID_OUTAKE_REVERSED);

// ---- Sensors (DIRECT PORTS; no config port constants) ----    
// Change these if your wiring differs:
// Inertial sensor moved from PORT1 -> PORT12 to free PORT1 for left motor
inertial inertial_sensor = inertial(PORT12);
rotation verticalRot     = rotation(PORT9, false);
rotation horizontalRot   = rotation(PORT6, false);
