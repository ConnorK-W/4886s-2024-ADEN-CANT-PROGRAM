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
void test_distance();
void test_potentiometer();

int main() {
    // NORMAL
    vex::competition Competition;
     
    // AUTON
    Competition.autonomous(autonomous);
    Competition.drivercontrol(opcontrol);
    
    pre_auton();
    // NORMAL


    // // CALIBRATE FOR TESTING FXNs
    // imu.calibrate();
    // while (imu.isCalibrating()) {
    //     wait(20, vex::msec);
    // }
    
    // Start vision processing task for goal detection
    
    
    // TEST FXNs HERE
    // Tune drive_straight_toward_goal k values
    // aivis.startAwb();
    // wait(500, vex::msec); // Let AWB initialize
    // vex::thread vision_thread(vision_processing_task);
    // tune_drive_toward_goal();
    // TEST FXNs HERE


    // test_potentiometer();

    // master.ButtonLeft.pressed([]() { arm.rotate_pid(50.0); });

    // while (true) {
    //     B_SCRN.clearScreen();
    //     B_SCRN.setCursor(1, 1);
    //     B_SCRN.print("Pot Value: %.2f", get_pot_value());
    //     wait(20, vex::msec);
    // }

    // test_distance();

}

void test_distance() {
    intakeLow.spin(DIR_FWD, 100, VLT_VLT);
    while (1) {
        double distancefront = distance_front.objectDistance(vex::distanceUnits::in);
        double distanceleft = distance_left.objectDistance(vex::distanceUnits::in);
        B_SCRN.printAt(100, 240, "Front dist: %.2f", distancefront);
        B_SCRN.printAt(100, 220, "Left dist: %.2f", distanceleft);
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
