#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "vex.h"
#include "motion.h"
#include "odom_task.h"
#include <cmath>

using namespace vex;


//sus describes bot curve  with pid sharp turn instead faster curve and turn

struct Poly3 {
    double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0;

    inline double pos(double t) const {
        return ((a3 * t + a2) * t + a1) * t + a0;
    }

    inline double vel(double t) const {
        return (3.0 * a3 * t + 2.0 * a2) * t + a1;
    }
};

inline Poly3 makeCubic(double p0, double v0,
                       double pf, double vf,
                       double T) {
    Poly3 p;
    p.a0 = p0;
    p.a1 = v0;

    const double dP = pf - p0 - v0 * T;
    const double dV = vf - v0;

    p.a3 = (dV - 2.0 * dP / T) / (T * T);
    p.a2 = (dP - p.a3 * T * T * T) / (T * T);
    return p;
}

inline void followCubicSegment(motor_group& left,
                               motor_group& right,
                               double startX, double startY,
                               double targetX, double targetY,
                               double nextX, double nextY,
                               double& prevVx, double& prevVy,
                               DriveGains& G,
                               double refVel_mps = 0.3,
                               double segmentTime = 2.0,
                               double dt_s = 0.02)
{
    double x0 = x_m();
    double y0 = y_m();

    double xf = targetX;
    double yf = targetY;

    double targTheta = std::atan2(nextY - yf, nextX - xf);
    double targVx = refVel_mps * std::cos(targTheta);
    double targVy = refVel_mps * std::sin(targTheta);

    Poly3 px = makeCubic(x0, prevVx, xf, targVx, segmentTime);
    Poly3 py = makeCubic(y0, prevVy, yf, targVy, segmentTime);

    G.head.reset(heading_d());

    const int steps = static_cast<int>(segmentTime / dt_s);
    for (int i = 0; i <= steps; i++) {
        double t = i * dt_s;

        double vx = px.vel(t);
        double vy = py.vel(t);
        double vMag = std::sqrt(vx * vx + vy * vy);

        double targetThetaDeg = std::atan2(vy, vx) * 180.0 / M_PI;

        G.head.setSetpoint(targetThetaDeg);
        double w = G.head.update(heading_d(), dt_s);
        w = std::max(-G.wMax, std::min(G.wMax, w));

        double vCmd = (refVel_mps > 1e-6) ? (vMag / refVel_mps) * G.vMax : 0.0;
        vCmd = std::max(-G.vMax, std::min(G.vMax, vCmd));

        tank(left, right, vCmd - G.kMix * w, vCmd + G.kMix * w);
        wait(static_cast<int>(dt_s * 1000.0), msec);
    }

    prevVx = targVx;
    prevVy = targVy;
}

inline void followWaypointPath(motor_group& left,
                               motor_group& right,
                               const double waypoints[][2],
                               int count,
                               DriveGains& G,
                               double refVel_mps = 0.3,
                               double segmentTime = 2.0,
                               double dt_s = 0.02)
{
    if (count < 2) return;

    double prevVx = 0.0;
    double prevVy = 0.0;

    double startX = x_m();
    double startY = y_m();

    for (int i = 0; i < count - 1; i++) {
        double currX = waypoints[i][0];
        double currY = waypoints[i][1];
        double nextX = waypoints[i + 1][0];
        double nextY = waypoints[i + 1][1];

        followCubicSegment(left, right,
                           startX, startY,
                           currX, currY,
                           nextX, nextY,
                           prevVx, prevVy,
                           G, refVel_mps, segmentTime, dt_s);

        startX = currX;
        startY = currY;
    }

    stopDrive(left, right);
}

#endif


