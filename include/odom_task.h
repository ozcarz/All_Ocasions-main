#ifndef ODOM_TASK_H
#define ODOM_TASK_H

#include "vex.h"
#include "sensors.h"
#include "robot_config.h"
#include <cmath>

using namespace vex;
using namespace config;

inline double wrap180(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

inline double heading_d() {
  return inertial_sensor.heading(degrees);
}

inline double rotDegToM(double deg) {
  return (deg * TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
}

// ------------------------------
// Header-only odom state (single TU friendly)
// ------------------------------
struct _OdomState {
  double x = 0.0;
  double y = 0.0;

  double prevHeadingDeg = 0.0;
  double prevVertDeg = 0.0;
  double prevHorizDeg = 0.0;

  double totalDistM = 0.0;
};

inline _OdomState& _odom() {
  static _OdomState s;
  return s;
}

// Public API used everywhere else
inline double x_m() { return _odom().x; }
inline double y_m() { return _odom().y; }

inline void resetTotalDistance() {
  _odom().totalDistM = 0.0;
  _odom().prevVertDeg  = getVerticalDeg();
  _odom().prevHorizDeg = getHorizontalDeg();
  _odom().prevHeadingDeg = heading_d();
}

inline double totalDistance_m() {
  return _odom().totalDistM;
}

inline void setOdometry(double x, double y, double headingDeg) {
  _odom().x = x;
  _odom().y = y;

  // We still *use IMU* for heading in runtime, but allow this for initialization.
  _odom().prevHeadingDeg = headingDeg;

  _odom().prevVertDeg  = getVerticalDeg();
  _odom().prevHorizDeg = getHorizontalDeg();
}

inline void resetOdometry() {
  setOdometry(0.0, 0.0, heading_d());
  resetTotalDistance();
}

// ------------------------------
// Task update: center vertical + center horizontal wheels
// heading 0 => +Y forward (matches your old convention)
// ------------------------------
inline int odomTaskFn(void*) {
  // Initialize baselines
  _odom().prevHeadingDeg = heading_d();
  _odom().prevVertDeg  = getVerticalDeg();
  _odom().prevHorizDeg = getHorizontalDeg();

  while (true) {
    const double newHead = heading_d();
    const double dHeadDeg = wrap180(newHead - _odom().prevHeadingDeg);
    const double midTheta = ( (_odom().prevHeadingDeg + dHeadDeg * 0.5) * M_PI / 180.0 );

    const double vDeg = getVerticalDeg();
    const double hDeg = getHorizontalDeg();

    const double dV = rotDegToM(vDeg - _odom().prevVertDeg);   // forward (robot frame)
    const double dH = rotDegToM(hDeg - _odom().prevHorizDeg);  // strafe (robot frame)

    // Field transform (same convention you had):
    // forward affects: x += -dV*sin(theta), y += dV*cos(theta)
    // strafe affects:  x +=  dH*cos(theta), y += dH*sin(theta)
    _odom().x += (-dV * std::sin(midTheta)) + (dH * std::cos(midTheta));
    _odom().y += ( dV * std::cos(midTheta)) + (dH * std::sin(midTheta));

    _odom().totalDistM += std::sqrt(dV*dV + dH*dH);

    _odom().prevHeadingDeg = newHead;
    _odom().prevVertDeg  = vDeg;
    _odom().prevHorizDeg = hDeg;

    wait(10, msec);
  }
  return 0;
}

// ------------------------------
// Brain output (what you asked for)
// ------------------------------
inline void printOdomDebug(brain& Brain) {
  const double vDeg = getVerticalDeg();
  const double hDeg = getHorizontalDeg();
  const double vM = rotDegToM(vDeg);
  const double hM = rotDegToM(hDeg);

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("ODOM + ROT");

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("X:%.2f Y:%.2f", x_m(), y_m());

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Head: %.1f", heading_d());

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("V: %.1fdeg %.3fm", vDeg, vM);

  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("H: %.1fdeg %.3fm", hDeg, hM);

  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("Total: %.3fm", totalDistance_m());
}

inline void printImuDebug(brain& Brain) {
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("IMU");

  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Heading: %.1f", inertial_sensor.heading(degrees));

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Rotation: %.1f", inertial_sensor.rotation(degrees));

  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Pitch: %.1f", inertial_sensor.pitch(degrees));

  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Roll: %.1f", inertial_sensor.roll(degrees));

  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("Ax:%.2f Ay:%.2f",
    inertial_sensor.acceleration(xaxis),
    inertial_sensor.acceleration(yaxis));
}

#endif // ODOM_TASK_H

