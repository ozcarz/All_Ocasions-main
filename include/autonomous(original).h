#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "vex.h"
#include "motor.h"
#include "sensors.h"
#include "odom_task.h"
#include "motion.h"
#include "intake.h"
#include "drive_helpers.h"
#include "trajectory.h"
#include <cmath>
#include "pneumatic_devices.h"

using namespace vex;

// -------------------------------------------------------------
// Shared auton initialization
// -------------------------------------------------------------
inline task initAutonomous(brain& Brain, controller& ctrl) {
    initSensors(Brain, ctrl);
    resetOdometry();

    task odomTask(odomTaskFn, nullptr);
    wait(100, msec);
    return odomTask;
}

// -------------------------------------------------------------
// Simple retry wrapper for driveDistance
// -------------------------------------------------------------
inline bool executeWithRetry(motor_group& left, motor_group& right,
                             double distance, DriveGains& G, int maxRetries = 1) {
    for (int attempt = 0; attempt <= maxRetries; attempt++) {
        bool success = driveDistance(left, right, distance, G, 3000, 0.02, 2.0);
        if (success) return true;

        if (attempt < maxRetries) {
            driveDistance(left, right, -0.10, G, 1000);
            wait(100, msec);
        }
    }
    return false;
}

// Helper: mm -> m
inline double mmToM(double mm) { return mm / 1000.0; }


inline void turn90() {
    DriveGains G;
//              P           D
G.dist.setGains(0.1, 0.0, 0.0);     // drive distance PID
G.head.setGains(0.1, 0.0, 0.0);   // heading hold while driving
G.turn.setGains(0.1, 0.00, 0.0);   // NEW: turning PID (turnTo/turnBy)
G.vMax = 70.0;
G.wMax = 60.0;
G.kMix = 0.6;
// Milliseconds first is the distance 
    driveDistance(LeftMotorGroup, RightMotorGroup, 1.328, G, 6000);
    turnBy(LeftMotorGroup, RightMotorGroup, 89.8, G, 2000, 1.5);
    wait(200, msec);

    pneumaticWings.toggle();
    wait(200, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.68, G, 6000);
    runOutake(100);
    wait(2.5, sec);
    stopOutake();
    wait(200, msec);


    driveDistance(LeftMotorGroup, RightMotorGroup, -0.5, G, 6000);
    wait(250, msec);
    turnBy(LeftMotorGroup, RightMotorGroup, 186, G, 2000, 1.5);
    wait(200, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.62, G, 6000);
    wait(250, msec);

    runIntake(100);
    wait(500, msec);
    stopIntake();

    driveDistance(LeftMotorGroup, RightMotorGroup, -0.5, G, 6000);
    wait(250, msec);
    turnBy(LeftMotorGroup, RightMotorGroup, 182, G, 2000, 1.5);
    wait(200, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.5, G, 6000);
  
}

inline void turn180(){
   DriveGains G;
G.dist.setGains(0.1, 0.0, 0.0);     // drive distance PID
G.head.setGains(0.1, 0.0, 0.0);   // heading hold while driving
G.turn.setGains(0.1, 0.00, 0.0);   // NEW: turning PID (turnTo/turnBy)
G.vMax = 70.0;
G.wMax = 60.0;
G.kMix = 0.6;

    turnBy(LeftMotorGroup, RightMotorGroup, 180, G, 5000);
    wait(250, msec);
    turnBy(LeftMotorGroup, RightMotorGroup, 90, G, 5000);
    

   
}




inline void autonomousMain() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 70;
    G.wMax = 60;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    runIntake(100);
    wait(200, msec);

    executeWithRetry(LeftMotorGroup, RightMotorGroup, 0.50, G);

    reverseIntake(80);
    wait(600, msec);
    stopIntake();

    driveDistance(LeftMotorGroup, RightMotorGroup, -0.30, G, 2500);
    turnBy(LeftMotorGroup, RightMotorGroup, 45.0, G, 2000, 1.5);

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void autonomousSafe() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 65;
    G.wMax = 55;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    runIntake(100);
    wait(200, msec);
    driveDistance(LeftMotorGroup, RightMotorGroup, 0.45, G, 2500);

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void autonomousSkills() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 75;
    G.wMax = 65;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    for (int cycle = 0; cycle < 5; cycle++) {
        runIntake(100);
        driveDistance(LeftMotorGroup, RightMotorGroup, 0.65, G, 3000);
        wait(350, msec);

        turnBy(LeftMotorGroup, RightMotorGroup, 72.0, G, 2000);

        driveDistance(LeftMotorGroup, RightMotorGroup, 0.50, G, 2500);

        reverseIntake(100);
        wait(600, msec);
        stopIntake();

        driveDistance(LeftMotorGroup, RightMotorGroup, -0.45, G, 2500);
        turnBy(LeftMotorGroup, RightMotorGroup, 36.0, G, 2000);
    }

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void autonomousRed() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 70;
    G.wMax = 60;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    runIntake(100);
    wait(200, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.55, G);
    reverseIntake(80);
    wait(500, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, -0.30, G);
    turnBy(LeftMotorGroup, RightMotorGroup, -45.0, G);

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void autonomousBlue() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 70;
    G.wMax = 60;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    runIntake(100);
    wait(200, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.55, G);
    reverseIntake(80);
    wait(500, msec);

    driveDistance(LeftMotorGroup, RightMotorGroup, -0.30, G);
    turnBy(LeftMotorGroup, RightMotorGroup, 45.0, G);

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void autonomousTest() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    driveDistance(LeftMotorGroup, RightMotorGroup, 0.50, G);
    wait(500, msec);

    turnBy(LeftMotorGroup, RightMotorGroup, 90.0, G);
    wait(500, msec);

    stopDrive(LeftMotorGroup, RightMotorGroup);
}

// Example trajectory auton (uses followWaypointPath)
inline void autonomousTrajectory() {
    task odomTask = initAutonomous(Brain, Controller1);

    DriveGains G;
    G.vMax = 60;
    G.wMax = 60;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    const double waypoints[][2] = {
        {0.50,  0.00},
        {0.50, -0.50},
        {1.00, -0.50},
        {1.00,  0.00},
        {0.00,  0.00}
    };
    const int n = sizeof(waypoints) / sizeof(waypoints[0]);

    followWaypointPath(LeftMotorGroup, RightMotorGroup,
                       waypoints, n, G,
                       0.3, 2.0, 0.02);

    stopDrive(LeftMotorGroup, RightMotorGroup);
    stopIntake();
}

inline void autonomousBlocksLoaderGoals() {
    task odomTask = initAutonomous(Brain, Controller1);

    setOdometry(mmToM(1700), mmToM(0), 0.0);

    DriveGains G;
    G.vMax = 60;
    G.wMax = 60;
    G.kMix = 0.6;
    G.dist.setGains(1.5, 0.02, 0.05);
    G.head.setGains(2.0, 0.0, 0.08);

    runIntake(100);

    const double waypoints_m[][2] = {
        { mmToM(1700),  mmToM(600) },
        { mmToM(1725),  mmToM(1200) },
        { mmToM(600),   mmToM(0)   }
    };
    const int n = sizeof(waypoints_m) / sizeof(waypoints_m[0]);

    followWaypointPath(LeftMotorGroup, RightMotorGroup,
                       waypoints_m, n, G,
                       0.35, 1.5, 0.02);

    stopIntake();
    stopDrive(LeftMotorGroup, RightMotorGroup);
}

inline void encoderTest() {
    initSensors(Brain, Controller1);
    resetOdometry();
    task odomTask(odomTaskFn, nullptr);
    wait(100, msec);
}



#endif // AUTONOMOUS_H

