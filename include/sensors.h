#ifndef SENSORS_H
#define SENSORS_H

#include "vex.h"
using namespace vex;

extern inertial inertial_sensor;
extern rotation verticalRot;
extern rotation horizontalRot;

inline void initSensors(brain& Brain, controller& ctrl) {
  inertial_sensor.calibrate();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating IMU...");

  ctrl.Screen.clearScreen();
  ctrl.Screen.setCursor(1, 1);
  ctrl.Screen.print("Calibrating...");

  int timeout = 6000, elapsed = 0;
  while (inertial_sensor.isCalibrating() && elapsed < timeout) {
    wait(50, msec);
    elapsed += 50;
  }

  verticalRot.resetPosition();
  horizontalRot.resetPosition();

  Brain.Screen.clearScreen();
  ctrl.Screen.clearScreen();
  ctrl.Screen.setCursor(1, 1);
  ctrl.Screen.print("Ready");
  wait(200, msec);
}

inline void resetSensors() {
  inertial_sensor.resetRotation();
  verticalRot.resetPosition();
  horizontalRot.resetPosition();
}

inline bool sensorsReady() { return !inertial_sensor.isCalibrating(); }

inline double getHeading() { return inertial_sensor.heading(degrees); }
inline double getPitch()   { return inertial_sensor.pitch(degrees); }
inline double getRoll()    { return inertial_sensor.roll(degrees); }

inline double getVerticalDeg()   { return verticalRot.position(degrees); }
inline double getHorizontalDeg() { return horizontalRot.position(degrees); }

// compatibility aliases (old code expects these)
inline double getLeftVerticalDeg()  { return getVerticalDeg(); }
inline double getRightVerticalDeg() { return getVerticalDeg(); }
inline double getForwardDeg()       { return getVerticalDeg(); }

#endif


