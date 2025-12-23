/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Aden                                             */
/*    Created:      Mon May 16 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "../include/main.h"

void test_aivision();
void test_vision();

int main() {
    vex::competition Competition;

    Competition.autonomous(autonomous);
    Competition.drivercontrol(opcontrol);
    pre_auton();


    // imu.calibrate();
    // master.ButtonLeft.pressed(tune_fast_pid);
    // master.ButtonRight.pressed(autonomous);

    while (true) {
        wait(20, vex::msec);
    }
}

void test_aivision() {
    vis.colorDetection(false);
    vis.modelDetection(true);
    while (1) {
        vis.takeSnapshot(vex::aivision::ALL_AIOBJS);
        B_SCRN.clearScreen();
        B_SCRN.printAt(20, 20, "%d", vis.largestObject.centerX);

        wait(20, vex::msec);
    }
}

void test_vision() {
    vis.colorDetection(false);
    vis.modelDetection(true);
    while (1) {
        vis.takeSnapshot(vex::aivision::ALL_AIOBJS);
        B_SCRN.clearScreen();
        B_SCRN.printAt(20, 20, "%d", vis.largestObject.centerX);

        wait(20, vex::msec);
    }
}
