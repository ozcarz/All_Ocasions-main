#ifndef MANUAL_H
#define MANUAL_H

#include "vex.h"
#include "motor.h"
#include "sensors.h"
#include "robot_config.h"
#include "drive_helpers.h"
#include "intake.h"
#include "pneumatic_devices.h"
#include "utils.h"
#include "odom_task.h"

using namespace vex;
using namespace config;

//extern PneumaticController pneumaticWings;

struct InertialData {
  double pitch = 0.0;
  double roll = 0.0;
  double accelX = 0.0;
  double accelY = 0.0;
  double accelScale = ACCEL_SCALE_SLIGHT;
  bool tippingForward = false;
  bool tippingBackward = false;
  bool tippingSideways = false;
};

static double _slewFwd = 0.0;          // NEW: slew only forward
static double _pitchSmoothed = 0.0;
static double _rollSmoothed = 0.0;
static double _accelXSmoothed = 0.0;
static double _accelYSmoothed = 0.0;
static double _driveSpeedScale = 1.0;

inline double expMovingAvg(double smoothed, double raw, double alpha = 0.3) {
  return alpha * raw + (1.0 - alpha) * smoothed;
}

inline bool edgePressed(bool& previous, bool current) {
  bool rising = (!previous && current);
  previous = current;
  return rising;
}

inline InertialData getInertialData() {
  InertialData data;

  if (inertial_sensor.isCalibrating()) {
    data.accelScale = ACCEL_SCALE_MODERATE;
    return data;
  }

  _pitchSmoothed  = expMovingAvg(_pitchSmoothed, inertial_sensor.pitch(degrees), 0.4);
  _rollSmoothed   = expMovingAvg(_rollSmoothed,  inertial_sensor.roll(degrees),  0.4);
  _accelXSmoothed = expMovingAvg(_accelXSmoothed, inertial_sensor.acceleration(xaxis), 0.3);
  _accelYSmoothed = expMovingAvg(_accelYSmoothed, inertial_sensor.acceleration(yaxis), 0.3);

  data.pitch = _pitchSmoothed;
  data.roll  = _rollSmoothed;
  data.accelX = _accelXSmoothed;
  data.accelY = _accelYSmoothed;

  data.tippingForward  = data.pitch > PITCH_THRESHOLD_LOW;
  data.tippingBackward = data.pitch < -PITCH_THRESHOLD_LOW;
  data.tippingSideways = absD(data.roll) > ROLL_THRESHOLD;

  double absPitch = absD(data.pitch);
  double absRoll  = absD(data.roll);

  if (absPitch > PITCH_THRESHOLD_HIGH || absRoll > ROLL_THRESHOLD) data.accelScale = ACCEL_SCALE_SEVERE;
  else if (absPitch > PITCH_THRESHOLD_MED) data.accelScale = ACCEL_SCALE_MODERATE;
  else if (absPitch > PITCH_THRESHOLD_LOW) data.accelScale = ACCEL_SCALE_SLIGHT;
  else data.accelScale = ACCEL_SCALE_NORMAL;

  return data;
}

// Apply anti-tip ONLY to forward drive, never to turning
inline double applyAntiTipForward(double fwdCmd, const InertialData& data) {
  double absPitch = absD(data.pitch);

  if (data.tippingForward && fwdCmd > 0) {
    double reduction = 1.0 - (absPitch - PITCH_THRESHOLD_LOW) /
                             (PITCH_THRESHOLD_HIGH - PITCH_THRESHOLD_LOW);
    reduction = std::max(ANTI_TIP_DAMPING, std::min(1.0, reduction));
    return fwdCmd * reduction;
  }

  if (data.tippingBackward && fwdCmd < 0) {
    double reduction = 1.0 - (absPitch - PITCH_THRESHOLD_LOW) /
                             (PITCH_THRESHOLD_HIGH - PITCH_THRESHOLD_LOW);
    reduction = std::max(ANTI_TIP_DAMPING, std::min(1.0, reduction));
    return fwdCmd * reduction;
  }

  return fwdCmd;
}

inline double dynamicSlewStep(double current, double target, double accelScale) {
  target = clampPctD(target);

  double step = (target == 0 || absD(target) < absD(current))
                ? DRIVE_DECEL_STEP
                : DRIVE_ACCEL_STEP * accelScale;

  if (current < target) current = (current + step > target) ? target : (current + step);
  else if (current > target) current = (current - step < target) ? target : (current - step);

  return clampPctD(current);
}

// NEW: turning is direct from driver, slew/anti-tip only affects forward
inline void arcadeDrive(controller& ctrl, motor_group& left, motor_group& right) {
  int fwdIn  = ctrl.Axis3.position(percent);
  int turnIn = ctrl.Axis1.position(percent);

  if (std::abs(fwdIn)  < ARCADE_DEADBAND) fwdIn = 0;
  if (std::abs(turnIn) < ARCADE_DEADBAND) turnIn = 0;

  double fwdCmd  = clampPctD(fwdIn  * ARCADE_SCALE) * _driveSpeedScale;
  double turnCmd = clampPctD(turnIn * ARCADE_SCALE) * _driveSpeedScale;

  InertialData imuData = getInertialData();

  // Only forward gets scaled by anti-tip + slew
  double targetFwd = applyAntiTipForward(fwdCmd, imuData);
  _slewFwd = dynamicSlewStep(_slewFwd, targetFwd, imuData.accelScale);

  double l = clampPctD(_slewFwd + turnCmd);
  double r = clampPctD(_slewFwd - turnCmd);

  left.spin(directionType::fwd, l, percent);
  right.spin(directionType::fwd, r, percent);
}

inline void displayDriveMode(controller& ctrl, bool sportMode) {
  static int displayCounter = 0;
  displayCounter++;

  if (displayCounter >= 25) {
    displayCounter = 0;

    ctrl.Screen.clearScreen();
    ctrl.Screen.setCursor(1, 1);

    if (sportMode) {
      ctrl.Screen.print("SPORT MODE");
      ctrl.Screen.setCursor(2, 1);
      ctrl.Screen.print("Speed: 100%%");
    } else {
      ctrl.Screen.print("SCORING MODE");
      ctrl.Screen.setCursor(2, 1);
      ctrl.Screen.print("Speed: 15%%");
    }

    ctrl.Screen.setCursor(3, 1);
    ctrl.Screen.print("Wings: %s", pneumaticWings.getStatusString());
  }
}

inline void updateIntakeControls(controller& ctrl) {
  bool l1 = ctrl.ButtonL1.pressing();
  bool l2 = ctrl.ButtonL2.pressing();
  bool r2 = ctrl.ButtonR2.pressing();

  const double intakeSpeed = 100.0;
  const double outakeSpeed = pneumaticWings.isExtended() ? 100.0 : 50.0;

  if (r2) {
    reverseIntake(intakeSpeed);
    reverseOutake(outakeSpeed);
  } else if (l2) {
    runIntake(intakeSpeed);
    runOutake(outakeSpeed);
  } else if (l1) {
    runIntake(intakeSpeed);
    stopOutake();
  } else {
    stopIntake();
    stopOutake();
  }
}

inline void usercontrol(void) {
  bool prevUp = false;
  bool prevR1 = false;
  bool prevB  = false;
  bool prevX  = false;

  bool sportMode = true;
  bool showOdomScreen = true;
  int brainPrintCounter = 0;

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("SPORT MODE");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Speed: 100%%");

  _driveSpeedScale = 1.0;
  _slewFwd = 0.0;

  while (true) {
    arcadeDrive(Controller1, LeftMotorGroup, RightMotorGroup);

    bool r1 = Controller1.ButtonR1.pressing();
    if (edgePressed(prevR1, r1)) {
      sportMode = !sportMode;
      _driveSpeedScale = sportMode ? 1.0 : 0.15;
      Controller1.rumble(sportMode ? ".." : ".");
    }

    bool upButton = Controller1.ButtonUp.pressing();
    if (edgePressed(prevUp, upButton)) {
      pneumaticWings.toggle();
      Controller1.rumble(".");
    }

    bool bButton = Controller1.ButtonB.pressing();
    if (edgePressed(prevB, bButton)) {
      showOdomScreen = !showOdomScreen;
      Brain.Screen.clearScreen();
      Controller1.rumble("-");
    }

    // Button X: reset total distance AND reset both rotation sensors
    bool xButton = Controller1.ButtonX.pressing();
    if (edgePressed(prevX, xButton)) {
      resetTotalDistance();
      verticalRot.resetPosition();
      horizontalRot.resetPosition();
      Controller1.rumble(".");
    }

    updateIntakeControls(Controller1);
    displayDriveMode(Controller1, sportMode);

    brainPrintCounter++;
    if (brainPrintCounter >= 5) {
      brainPrintCounter = 0;
      if (showOdomScreen) printOdomDebug(Brain);
      else printImuDebug(Brain);
    }

    wait(20, msec);
  }
}

#endif
