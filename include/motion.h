#ifndef MOTION_H
#define MOTION_H

#include "vex.h"
#include "PID.h"
#include "odom_task.h"       // provides heading_d() and wrap180()
#include "drive_helpers.h"
#include "robot_config.h"
#include "utils.h"
#include "sensors.h"
#include <cmath>
#include <algorithm>

using namespace vex;

struct DriveGains {
    PID dist{1.5, 0.0, 0.05, 1000.0, -100.0, 100.0};
    PID head{2.2, 0.0, 0.08, 1000.0, -100.0, 100.0}; // heading correction while driving

    // NEW: dedicated turning controller (filtered D to reduce IMU noise jitter)
    PID turn{1.4, 0.0, 0.12, 1000.0, -100.0, 100.0, 0.06}; // filterTf = 0.06s

    double vMax = 70.0;
    double wMax = 60.0;
    double kMix = 0.6;

    void setDistGains(double kp, double ki, double kd) { dist.setGains(kp, ki, kd); }
    void setHeadGains(double kp, double ki, double kd) { head.setGains(kp, ki, kd); }
    void setTurnGains(double kp, double ki, double kd) { turn.setGains(kp, ki, kd); } // NEW
};


// wrap180() REMOVED from here (use the one in odom_task.h)

inline double angleDiff(double target, double current) {
    return wrap180(target - current);
}

// =============================================================================
// TURN TO - Absolute heading
// =============================================================================
// =============================================================================
// TURN TO - Absolute heading (IMU heading diff)
// =============================================================================
// =============================================================================
// TURN TO - Absolute heading (uses IMU heading + rate settle)
// =============================================================================
inline bool turnTo(motor_group& left, motor_group& right,
                   double targetDeg, DriveGains& G,
                   int timeoutMs = 3000, double tolDeg = 1.0)
{
    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    // Copy so we don't mutate G.turn's internal state outside this call
    PID turnPid = G.turn;
    turnPid.setSetpoint(0.0);
    turnPid.setOutputLimits(-G.wMax, G.wMax);

    // Initialize PID state "bumpless" with current error
    {
        const double err0 = angleDiff(targetDeg, heading_d());
        turnPid.reset(-err0);
    }

    // Rate-based settling (prevents overshoot -> correct -> overshoot oscillation)
    const double rateTolDegPerSec = 8.0;   // tighten to 5-6 if you want crisper stops
    const int settleMs = 150;
    int goodMs = 0;

    double prevHead = heading_d();
    double rateFilt = 0.0;
    const double rateTau = 0.06;                 // ~60ms low-pass on rate estimate
    const double alpha = dt / (rateTau + dt);

    // Static friction kick (only when still clearly off-target)
    const double wStatic = 6.0;

    timer t; t.reset();

    while (t.time(msec) < timeoutMs) {
        const double head = heading_d();
        const double err  = angleDiff(targetDeg, head);
        const double absErr = std::fabs(err);

        // Estimate turn rate from heading delta (wrap-safe)
        const double dHead = wrap180(head - prevHead);
        prevHead = head;

        const double rate = dHead / dt;
        rateFilt += alpha * (rate - rateFilt);

        // If we're inside tolerance AND not still rotating, start settling
        if (absErr <= tolDeg && std::fabs(rateFilt) <= rateTolDegPerSec) {
            goodMs += dtMs;

            // Stop pushing while we settle (this kills the “bounce”)
            tank(left, right, 0, 0);

            if (goodMs >= settleMs) {
                stopDrive(left, right, brakeType::hold);
                return true;
            }

            wait(dtMs, msec);
            continue;
        } else {
            goodMs = 0;
        }

        // PID drives (-err) toward 0 -> output sign matches your old PD behavior
        double w = turnPid.update(-err, dt);

        // Soft slow-down close to target (NO minimum scale floor)
        const double slowZoneDeg = 20.0;
        if (absErr < slowZoneDeg) {
            w *= (absErr / slowZoneDeg);
        }

        // Static friction compensation ONLY when not near target
        if (absErr > std::max(2.0, 2.0 * tolDeg) && std::fabs(w) < wStatic) {
            w = std::copysign(wStatic, w);
        }

        // Clamp and command
        w = std::max(-G.wMax, std::min(G.wMax, w));
        tank(left, right, +w, -w);

        wait(dtMs, msec);
    }

    stopDrive(left, right, brakeType::hold);
    return false;
}

// =============================================================================
// TURN BY - Relative turn (uses IMU rotation for clean relative turns)
// =============================================================================
inline bool turnBy(motor_group& left, motor_group& right,
                   double deltaDeg, DriveGains& G,
                   int timeoutMs = 3000, double tolDeg = 1.0)
{
    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    PID turnPid = G.turn;
    turnPid.setSetpoint(0.0);
    turnPid.setOutputLimits(-G.wMax, G.wMax);

    const double startRot  = inertial_sensor.rotation(degrees);
    const double targetRot = startRot + deltaDeg;

    {
        const double err0 = targetRot - inertial_sensor.rotation(degrees);
        turnPid.reset(-err0);
    }

    const double rateTolDegPerSec = 8.0;
    const int settleMs = 150;
    int goodMs = 0;

    double prevRot = inertial_sensor.rotation(degrees);
    double rateFilt = 0.0;
    const double rateTau = 0.06;
    const double alpha = dt / (rateTau + dt);

    const double wStatic = 6.0;

    timer t; t.reset();

    while (t.time(msec) < timeoutMs) {
        const double rot = inertial_sensor.rotation(degrees);
        const double err = targetRot - rot;
        const double absErr = std::fabs(err);

        const double dRot = rot - prevRot;
        prevRot = rot;

        const double rate = dRot / dt;
        rateFilt += alpha * (rate - rateFilt);

        if (absErr <= tolDeg && std::fabs(rateFilt) <= rateTolDegPerSec) {
            goodMs += dtMs;
            tank(left, right, 0, 0);
            if (goodMs >= settleMs) {
                stopDrive(left, right, brakeType::hold);
                return true;
            }
            wait(dtMs, msec);
            continue;
        } else {
            goodMs = 0;
        }

        double w = turnPid.update(-err, dt);

        const double slowZoneDeg = 20.0;
        if (absErr < slowZoneDeg) {
            w *= (absErr / slowZoneDeg);
        }

        if (absErr > std::max(2.0, 2.0 * tolDeg) && std::fabs(w) < wStatic) {
            w = std::copysign(wStatic, w);
        }

        w = std::max(-G.wMax, std::min(G.wMax, w));
        tank(left, right, +w, -w);

        wait(dtMs, msec);
    }

    stopDrive(left, right, brakeType::hold);
    return false;
}



// =============================================================================
// DRIVE DISTANCE (center vertical wheel => no offset correction)
// =============================================================================
inline bool driveDistance(motor_group& left, motor_group& right,
                          double distance_m, DriveGains& G,
                          int timeoutMs = 3000,
                          double posTol = 0.005, double /*headTol*/ = 2.0) {

    const double holdHead = heading_d();

    const double direction  = (distance_m >= 0) ? 1.0 : -1.0;
    const double targetDist = std::fabs(distance_m);

    const double startVertDeg = getVerticalDeg();
    const double kDegToM = config::TRACKING_WHEEL_CIRCUMFERENCE_M / 360.0;

    const double wheelOffsetM = 0.0; // centered

    const int settleMs = 100;
    int inRangeMs = 0;

    const double vMinFar  = 5.0;
    const double vMinNear = 3.0;
    const double nearDist = 0.05;

    G.dist.reset(0);
    G.dist.setSetpoint(targetDist);
    G.head.reset(0.0);
    G.head.setSetpoint(0.0);

    const int dt = 20;
    timer t; t.reset();

    const double decelZone = std::min(0.80, std::max(0.20, 0.20 * targetDist));

    while (t.time(msec) < timeoutMs) {
        const double deltaVertDeg = getVerticalDeg() - startVertDeg;
        const double vertM = deltaVertDeg * kDegToM;

        const double dHeadDeg = wrap180(heading_d() - holdHead);
        const double dTheta   = dHeadDeg * M_PI / 180.0;

        const double forwardM = vertM - wheelOffsetM * dTheta;

        const double traveled = std::fabs(forwardM);
        const double distErr  = targetDist - traveled;

        if (distErr <= posTol) {
            inRangeMs += dt;
            if (inRangeMs >= settleMs) {
                stopDrive(left, right, brakeType::brake);
                return true;
            }
        } else {
            inRangeMs = 0;
        }

        double v = G.dist.update(traveled, dt / 1000.0);
        v = std::max(-G.vMax, std::min(G.vMax, v)) * direction;

        if (distErr > 0 && distErr < decelZone) {
            double scale = distErr / decelZone;
            scale = std::max(0.0, std::min(1.0, scale));
            v *= scale;
        }

        if (distErr > posTol) {
            const double vMin = (distErr > nearDist) ? vMinFar : vMinNear;
            if (std::fabs(v) < vMin) v = std::copysign(vMin, v);
        }

        const double headErr = angleDiff(holdHead, heading_d());
        double w = G.head.update(-headErr, dt / 1000.0);
        w = std::max(-G.wMax, std::min(G.wMax, w));

        tank(left, right, v + G.kMix * w, v - G.kMix * w);

        wait(dt, msec);
    }

    stopDrive(left, right, brakeType::brake);
    return false;
}

#endif // MOTION_H

