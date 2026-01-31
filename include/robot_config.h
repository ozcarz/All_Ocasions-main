#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"
#include <cmath>

//extern vex::digital_out pneumaticWings;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace config {

// Drive motor reverse flags
constexpr bool LEFT_MOTOR_A_REVERSED  = false;
constexpr bool LEFT_MOTOR_B_REVERSED  = false;
constexpr bool LEFT_MOTOR_C_REVERSED  = false;

constexpr bool RIGHT_MOTOR_A_REVERSED = true;
constexpr bool RIGHT_MOTOR_B_REVERSED = true;
constexpr bool RIGHT_MOTOR_C_REVERSED = true;

// Gear ratios (VEX macros)
#define DRIVE_GEAR_RATIO     vex::ratio6_1
#define INTAKE_RATIO         vex::ratio6_1
#define OUTAKE_RATIO         vex::ratio6_1

// Intake reverse flags
constexpr bool INTAKE_LEFT_REVERSED  = false;
constexpr bool INTAKE_RIGHT_REVERSED = true;
constexpr bool MIDTAKE_REVERSED      = false;

// Outake reverse flags
constexpr bool RIGHT_OUTAKE_REVERSED = true;
constexpr bool LEFT_OUTAKE_REVERSED  = false;
constexpr bool MID_OUTAKE_REVERSED   = true;

// Geometry
constexpr double TRACK_WIDTH_MM = 320.0;
constexpr double WHEELBASE_MM   = 40.0;
constexpr double EXTERNAL_GEAR_RATIO = 1.0;

// Tracking wheel geometry (for rotation sensors)
constexpr double TRACKING_WHEEL_DIAMETER_MM = 83.5; // change if needed
constexpr double TRACKING_WHEEL_DIAMETER_M  = TRACKING_WHEEL_DIAMETER_MM / 1000.0;
constexpr double TRACKING_WHEEL_CIRCUMFERENCE_M = M_PI * TRACKING_WHEEL_DIAMETER_M;

// Drive wheel geometry (drivetrain object only)
constexpr double WHEEL_DIAMETER_MM = 101.6;
constexpr double WHEEL_DIAMETER_M  = WHEEL_DIAMETER_MM / 1000.0;
constexpr double WHEEL_CIRCUMFERENCE_M = M_PI * WHEEL_DIAMETER_M;

// Offsets (meters)
// If BOTH tracking wheels are mounted at the robot center, both offsets are 0.
constexpr double SIDE_SENSOR_OFFSET_M = 0.0;
constexpr double VERT_SENSOR_OFFSET_M = 0.0;

// Field / misc
constexpr double FIELD_WIDTH_M  = 3.65;
constexpr double FIELD_HEIGHT_M = 3.65;

constexpr double DEFAULT_DRIVE_SPEED = 100.0;
constexpr double MAX_DRIVE_SPEED     = 100.0;
constexpr double INTAKE_SPEED        = 100.0;

constexpr int ARCADE_DEADBAND = 2;
constexpr int TANK_DEADBAND   = 10;
//constexpr is for compiler this is important for manual.h and needs to be configured thusly
constexpr double DRIVE_ACCEL_STEP = 5.0;
constexpr double DRIVE_DECEL_STEP = 20.0;
constexpr double ARCADE_SCALE     = 0.75;

constexpr double PITCH_THRESHOLD_LOW  = 3.0;
constexpr double PITCH_THRESHOLD_MED  = 6.0;
constexpr double PITCH_THRESHOLD_HIGH = 10.0;
constexpr double ROLL_THRESHOLD       = 8.0;

constexpr double ACCEL_SCALE_NORMAL   = 1.0;
constexpr double ACCEL_SCALE_SLIGHT   = 0.75;
constexpr double ACCEL_SCALE_MODERATE = 0.5;
constexpr double ACCEL_SCALE_SEVERE   = 0.25;
constexpr double ANTI_TIP_DAMPING     = 0.8;

} // namespace config

#endif
