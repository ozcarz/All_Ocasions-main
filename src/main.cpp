#include "vex.h"
#include "robot_config.h"
#include "motor.h"
#include "sensors.h"
#include "pre_auton.h"
#include "autonomous(original).h"
#include "manual.h"

using namespace vex;

// -------------------------------------------------------------
// SELECT AUTONOMOUS ROUTINE HERE:
//
// 0 = autonomousMain
// 1 = autonomousSafe
// 2 = autonomousSkills
// 3 = autonomousRed
// 4 = autonomousBlue
// 5 = autonomousTest
// 6 = autonomousTrajectory
// 7 = autonomousBlocksLoaderGoals
// 8 = turn90
// 9 = encoderTest
// -------------------------------------------------------------
constexpr int AUTON_SELECTION = 8;   // <--- CHANGE THIS NUMBER ONLY
// -------------------------------------------------------------

competition Competition;

void autonomous(void) {
    switch (AUTON_SELECTION) {
        case 0: autonomousMain(); break;
        case 1: autonomousSafe(); break;
        case 2: autonomousSkills(); break;
        case 3: autonomousRed(); break;
        case 4: autonomousBlue(); break;
        case 5: autonomousTest(); break;
        case 6: autonomousTrajectory(); break;
        case 7: autonomousBlocksLoaderGoals(); break;
        case 8: turn90(); break;
        case 9: encoderTest(); break;
        case 10: turn180(); break;
        default: autonomousSafe(); break;
    }
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}

///sdfjksldjflksjdfklsjlkdf
