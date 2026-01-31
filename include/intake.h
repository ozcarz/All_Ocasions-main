#ifndef INTAKE_H
#define INTAKE_H

#include "vex.h"
#include "motor.h"

using namespace vex;

inline void runIntake(double speedPct = 100.0) {
  IntakeLeft.spin(directionType::fwd, speedPct, percent);
  IntakeRight.spin(directionType::fwd, speedPct, percent);
  Midtake.spin(directionType::fwd, speedPct, percent);
}

inline void reverseIntake(double speedPct = 100.0) {
  IntakeLeft.spin(directionType::rev, speedPct, percent);
  IntakeRight.spin(directionType::rev, speedPct, percent);
  Midtake.spin(directionType::rev, speedPct, percent);
}

inline void stopIntake(brakeType mode = brakeType::coast) {
  IntakeLeft.stop(mode);
  IntakeRight.stop(mode);
  Midtake.stop(mode);
}

inline void runOutake(double speedPct = 100.0) {
  RightOutake.spin(directionType::fwd, speedPct, percent);
  LeftOutake.spin(directionType::fwd, speedPct, percent);
  MidOutake.spin(directionType::fwd, speedPct, percent);
}

inline void reverseOutake(double speedPct = 100.0) {
  RightOutake.spin(directionType::rev, speedPct, percent);
  LeftOutake.spin(directionType::rev, speedPct, percent);
  MidOutake.spin(directionType::rev, speedPct, percent);
}

inline void stopOutake(brakeType mode = brakeType::coast) {
  RightOutake.stop(mode);
  LeftOutake.stop(mode);
  MidOutake.stop(mode);
}

#endif
