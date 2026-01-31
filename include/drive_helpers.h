#ifndef DRIVE_HELPERS_H
#define DRIVE_HELPERS_H

#include "vex.h"
#include "motor.h"
#include "utils.h"
#include <algorithm>

using namespace vex;

inline void tank(motor_group& left, motor_group& right, double lPct, double rPct) {
    lPct = std::max(-100.0, std::min(100.0, lPct));
    rPct = std::max(-100.0, std::min(100.0, rPct));

    if (lPct >= 0) left.spin(directionType::fwd,  lPct, percent);
    else          left.spin(directionType::rev, -lPct, percent);

    if (rPct >= 0) right.spin(directionType::fwd,  rPct, percent);
    else           right.spin(directionType::rev, -rPct, percent);
}

inline void stopDrive(motor_group& left, motor_group& right,
                      brakeType mode = brakeType::brake) {
    left.stop(mode);
    right.stop(mode);
}

inline void arcadeDrive(motor_group& left, motor_group& right,
                        double fwd, double turn, double scale = 1.0) {
    double l = (fwd + turn) * scale;
    double r = (fwd - turn) * scale;
    tank(left, right, l, r);
}

inline double applySlewRate(double current, double target,
                            double accelStep, double decelStep) {
    target = clampPctD(target);

    double step = accelStep;
    if (target == 0 || absD(target) < absD(current)) {
        step = decelStep;
    }

    if (current < target) current = std::min(current + step, target);
    else if (current > target) current = std::max(current - step, target);

    return clampPctD(current);
}

struct DriveState {
    double leftVelocity = 0.0;
    double rightVelocity = 0.0;
    void reset() { leftVelocity = 0.0; rightVelocity = 0.0; }
};

#endif // DRIVE_HELPERS_H



