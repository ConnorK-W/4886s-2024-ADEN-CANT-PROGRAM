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

    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }

    // master.ButtonLeft.pressed(tune_goal_pid);
    // master.ButtonRight.pressed(autonomous);
    // tune_biggoal_pid();

    // imu.calibrate();
    // drive_straight_toward_goal(2000, 1);

    while (true) {
        wait(20, vex::msec);
    }
}

void test_aivision() {
    // Init
    // How colors are specified: https://www.vexforum.com/t/ai-vision-on-visual-studio-code/125085/9
    // vex::aivision::colordesc red(1, 237, 61, 74, 10.00, 0.20);
    // vex::aivision::colordesc green(2, 10, 143, 45, 10.00, 0.20);
    // vex::aivision::colordesc blue(3, 13, 114, 227, 10.00, 0.20);
    vex::aivision::colordesc yellow(1, 255, 175, 83, 20.00, 0.30);
    vex::aivision test_aivis = vex::aivision(PORT4);

    // Configure
    test_aivis.set(yellow);

    test_aivis.colorDetection(true);
    test_aivis.modelDetection(false);

    // Logic
    test_aivis.startAwb();
    wait(2, vex::sec);

    while (1) {
        test_aivis.takeSnapshot(yellow);

        B_SCRN.clearScreen();

        if (test_aivis.largestObject.exists) {
            B_SCRN.printAt(20, 20, "X: %d   W: %d", test_aivis.largestObject.centerX, test_aivis.largestObject.width);
            B_SCRN.printAt(20, 40, "Y: %d   H: %d", test_aivis.largestObject.centerY, test_aivis.largestObject.height);
            B_SCRN.printAt(20, 60, "Count: %d", (int)test_aivis.objectCount);
        } else {
            B_SCRN.printAt(20, 20, "No Yellow Found");
        }

        wait(20, vex::msec);
    }
}

void test_vision() {
    // Init
    vex::vision::signature test_yellow = vex::vision::signature(1, 2159, 3375, 2766, -4481, -4079, -4280, 7.4, 0);
    vex::vision test_vis = vex::vision(PORT4);

    // Configure
    test_vis.setBrightness(50);
    test_vis.setSignature(test_yellow);

    // Logic
    while (1) {
        test_vis.takeSnapshot(test_yellow);

        B_SCRN.clearScreen();

        if (test_vis.largestObject.exists) {
            B_SCRN.printAt(20, 20, "X: %d   W: %d", test_vis.largestObject.centerX, test_vis.largestObject.width);
            B_SCRN.printAt(20, 40, "Y: %d   H: %d", test_vis.largestObject.centerY, test_vis.largestObject.height);
            B_SCRN.printAt(20, 60, "Count: %d", (int)test_vis.objectCount);
        } else {
            B_SCRN.printAt(20, 20, "No Yellow Found");
        }

        wait(20, vex::msec);
    }
}
