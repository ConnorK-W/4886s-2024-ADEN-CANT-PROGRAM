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
void test_drive_straight_with_dist();
void test_potentiometer();
void test_drive_to_dist_value();
void test_wall_follow();
void test_distance_display();
void drive_toward_goal_1500();

//#define TEST_FUNCS
#ifdef TEST_FUNCS
const bool run_main = false;
#else
const bool run_main = true;
#endif

void test_potentiometer_raw() {
    B_SCRN.clearScreen();
    while (true) {
        double raw = lever_pot.angle(vex::percentUnits::pct);
        double norm = get_pot_value();

        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Pct: %.2f   ", raw);
        B_SCRN.setCursor(2, 1);
        B_SCRN.print("Norm: %.2f   ", norm);

        printf("pct: %.2f  norm: %.2f\n", raw, norm);

        wait(100, vex::msec);
    }
}

int main() {
    test_potentiometer_raw();

    // imu.calibrate();
    // while (imu.isCalibrating()) {
    //     wait(20, vex::msec);
    // }
    // reset_imu_rotation();

    // vex::thread vision_thread(vision_processing_task);

    // master.ButtonA.pressed(drive_toward_goal_1500);

    // while (true) {
    //     wait(20, TIME_MSEC);
    // }
}

void drive_toward_goal_1500() {
    drive_straight_toward_goal(1500, false, true);
}

void test_drive_to_dist_value() {
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }
    reset_imu_rotation();
    
    // Drive to where front distance sensor reads 18.2 inches
    drive_straight_to_dist_value(18.2, 50, 20, distance_front);
}

void test_wall_follow() {
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }
    reset_imu_rotation();
    target_heading = imu_rotation();

    // Drive 24 inches while maintaining 21.5 inches from left wall
    // Parameters: distance(in), speed(ips), accel(ipss), target_wall_dist(in)
    drive_straight_wall_follow(24, 60, 30, 21.5);
}

void test_distance_display() {
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }
    reset_imu_rotation();
    target_heading = imu_rotation();

    B_SCRN.clearScreen();

    // Display distance readings continuously
    while (true) {
        float current_heading = imu_rotation();
        float measured_dist = distance_left.objectDistance(vex::distanceUnits::in);

        // Trig correction
        float heading_error = current_heading - target_heading;
        float angle_rad = heading_error * (3.14159265359 / 180.0);
        float cos_angle = std::cos(angle_rad);
        if (std::abs(cos_angle) < 0.1) cos_angle = 0.1;
        float corrected_dist = measured_dist / cos_angle;

        // Display on brain
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Distance Test");
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("Heading: %.1f deg  ", heading_error);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("Raw: %.2f in       ", measured_dist);
        B_SCRN.setCursor(5, 1);
        B_SCRN.print("Corrected: %.2f in ", corrected_dist);
        B_SCRN.setCursor(7, 1);
        B_SCRN.print("Rotate robot to test");

        // Print to terminal
        printf("Hdg:%.1f  Raw:%.2f  Corrected:%.2f\n",
               heading_error, measured_dist, corrected_dist);

        wait(100, vex::msec);
    }
}

void test_drive_straight_with_dist() {
    finger.set(1);
    drive_straight_with_dist(80, 60, 70, true, 0, 0);
    turn_pid(90 + offsettheta, -1, 1);
    offsettheta = 0;
}

void test_distance() {
    // intakeLow.spin(DIR_FWD, 100, VLT_VLT);
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
