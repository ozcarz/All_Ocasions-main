#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "vex.h"
#include "robot_config.h"
#include <cmath>

using namespace config;

class Odometry {
public:
    Odometry(double initialHeadingDeg = 0.0,
             double vertDeg = 0.0,
             double horizDeg = 0.0)
        : prevHeadingDeg_(initialHeadingDeg),
          prevVertDeg_(vertDeg),
          prevHorizDeg_(horizDeg),
          theta_(degToRad(initialHeadingDeg)) {}

    void setWheelCircumference(double meters) { wheelCirc_ = meters; }
    void setSideOffset(double meters) { sideOffset_ = meters; }
    void setImuBlendFactor(double factor) { imuBlendFactor_ = clampVal(factor, 0.0, 1.0); }

    double getX() const { return x_; }
    double getY() const { return y_; }

    double getTotalDistanceM() const { return totalDistM_; }
    void resetTotalDistance() { totalDistM_ = 0.0; }

    void reset(double x = 0.0, double y = 0.0, double headingDeg = 0.0,
               double vertDeg = 0.0, double horizDeg = 0.0) {
        x_ = x;
        y_ = y;
        theta_ = degToRad(headingDeg);
        prevHeadingDeg_ = headingDeg;
        prevVertDeg_ = vertDeg;
        prevHorizDeg_ = horizDeg;
        totalDistM_ = 0.0;
    }

    // 2-wheel odom update using IMU heading + (vertical,horizontal) rotation sensors
    void update(double imuHeadingDeg, double vertDeg, double horizDeg) {
        const double dVdeg = vertDeg - prevVertDeg_;
        const double dHdeg = horizDeg - prevHorizDeg_;
        const double dHead = wrapAngle(imuHeadingDeg - prevHeadingDeg_);

        prevVertDeg_ = vertDeg;
        prevHorizDeg_ = horizDeg;
        prevHeadingDeg_ = imuHeadingDeg;

        const double dVm = wheelCirc_ * (dVdeg / 360.0);
        const double dHm = wheelCirc_ * (dHdeg / 360.0);

        // IMPORTANT: heading difference (IMU) is what we use for rotation compensation
        const double dTheta = degToRad(dHead);
        const double midTheta = theta_ + dTheta * 0.5;

        // Compensate rotation-induced wheel motion
        const double dForward = dVm - VERT_SENSOR_OFFSET_M * dTheta;
        const double dStrafe  = dHm - sideOffset_ * dTheta;

        // heading 0 => +Y forward
        const double localX = dStrafe;
        const double localY = dForward;

        const double dx = localX * std::cos(midTheta) - localY * std::sin(midTheta);
        const double dy = localX * std::sin(midTheta) + localY * std::cos(midTheta);

        x_ += dx;
        y_ += dy;

        totalDistM_ += std::sqrt(dx * dx + dy * dy);

        theta_ = degToRad(imuHeadingDeg);
    }

private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

    double prevHeadingDeg_ = 0.0;
    double prevVertDeg_ = 0.0;
    double prevHorizDeg_ = 0.0;

    double wheelCirc_ = TRACKING_WHEEL_CIRCUMFERENCE_M;
    double sideOffset_ = SIDE_SENSOR_OFFSET_M;
    double imuBlendFactor_ = 0.95;

    double totalDistM_ = 0.0;

    static double degToRad(double deg) { return deg * M_PI / 180.0; }

    static double wrapAngle(double a) {
        a = std::fmod(a, 360.0);
        if (a < -180.0) a += 360.0;
        if (a >= 180.0) a -= 360.0;
        return a;
    }

    static double clampVal(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
};

#endif


