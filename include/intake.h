#ifndef INTAKE_H
#define INTAKE_H

#include "vex.h"
#include "motor.h"

using namespace vex;

// Intake forward
inline void runIntake(double speedPct = 100.0) {
  IntakeOutake.spin(directionType::fwd, speedPct, percent);
}

// Intake reverse
inline void reverseIntake(double speedPct = 100.0) {
  IntakeOutake.spin(directionType::rev, speedPct, percent);
}

// Stop motor
inline void stopIntake(brakeType mode = brakeType::coast) {
  IntakeOutake.stop(mode);
}

// Outtake forward (same as intake forward)
inline void runOutake(double speedPct = 100.0) {
  IntakeOutake.spin(directionType::fwd, speedPct, percent);
}

// Outtake reverse (same as intake reverse)
inline void reverseOutake(double speedPct = 100.0) {
  IntakeOutake.spin(directionType::rev, speedPct, percent);
}

// Stop motor
inline void stopOutake(brakeType mode = brakeType::coast) {
  IntakeOutake.stop(mode);
}

#endif
