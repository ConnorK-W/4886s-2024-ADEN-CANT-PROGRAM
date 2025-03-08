#include "main.h"
#include "stddefs.h"
#include <thread>

// Tunes side to side for drive straight
void tune_dir_pid() {
    imu.calibrate();
    while (imu.isCalibrating())
        vex::wait(20, vex::msec);
    master.rumble(".");

    const float TUNER = 0.025;

    while (true) {
        opdrive(TSA, 1, SENSITIVITY);
        if (BTN_Y.PRESSED) {
            target_heading = imu_rotation();
            drive_straight(80, 65, 256);
        }
        B_SCRN.printAt(8, 8, "\n%f\n\n", imu_rotation());
        wait(20, vex::msec);
    }
}

// Tunes acceleration for drive straight
void tune_accel_pid() {
    imu.calibrate();
    while (imu.isCalibrating())
        vex::wait(20, vex::msec);
    master.rumble(".");
    const float TUNER = 0.025;
    // Make PID objects
    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    while (true) {
        // Enable opcontrol
        opdrive(TSA, 1, SENSITIVITY);
        // Toggle pid movement on y press
        if (BTN_Y.PRESSED) {
            target_heading = imu_rotation();
            // Go back to opcontrol if y pressed again
            while (!BTN_Y.PRESSED) {
                // Drive forward  300 rpm
                drive_r.spin(DIR_FWD, 300 + rd.adjust(300, drive_r.velocity(VEL_RPM)) - dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                drive_l.spin(DIR_FWD, 300 + ld.adjust(300, drive_l.velocity(VEL_RPM)) + dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                wait(20, vex::msec);
            }
        }
        // Enable pid tuning
        ld.tune_kP(btn_up() - btn_down(), TUNER);
        ld.tune_kD(btn_right() - btn_left(), TUNER);
        rd.tune_kP(btn_up() - btn_down(), TUNER);
        rd.tune_kI(btn_x() - btn_b(), TUNER);
        rd.tune_kD(btn_right() - btn_left(), TUNER);

        wait(20, vex::msec);
    }
}

/*
R1 and R2 are for tuning turn in place
L1 and L2 are for tuning arc function

Need to change the PID drive_pid in movement.cpp to move_kp...
When done we need to swich it back to TURN_PID_KP...
*/
void tune_fast_pid() {
    move_kp = TURN_PID_KP;
    move_ki = TURN_PID_KI;
    move_kd = TURN_PID_KD;
    const float TUNER = 0.025;

    while (true) {
        // drive_r.setStopping(vex::coast);
        // drive_l.setStopping(vex::coast);
        master.rumble(".");
        opdrive(TSA, 1, SENSITIVITY);
        if (BTN_R1.PRESSED) {
            target_heading = imu_rotation();
            vex::thread t(graph_pid);
            turn_pid(90, -1, 1);
            t.interrupt();
        }
        if (BTN_R2.PRESSED) {
            target_heading = imu_rotation();
            vex::thread t(graph_pid);
            turn_pid(-90, -1, 1);
            t.interrupt();
        }
        if (BTN_L1.PRESSED) {
            //target_heading = imu_rotation();
            vex::thread t(graph_pid);
            drive_turn(-90, WHEEL_TO_WHEEL_DIST *2, 60, 60, true);
            // drive_straight(36, 66, 512);
            t.interrupt();
        }
        if (BTN_L2.PRESSED) {
            //target_heading = imu_rotation();
            vex::thread t(graph_pid);
            drive_turn(90, WHEEL_TO_WHEEL_DIST *2, 60, 60, true);
            // drive_straight(108, 72, 72);
            t.interrupt();
        }
        move_kp += (btn_up() - btn_down()) * TUNER;
        move_kd += (btn_x() - btn_b()) * TUNER;

        printf("\nkP: %f\nkI: %f\nkD: %f\n", move_kp, move_ki, move_kd);

        B_SCRN.clearScreen();
        B_SCRN.printAt(0, 20, "kP: %.3f", move_kp);
        B_SCRN.printAt(0, 40, "kI: %.3f", move_ki);
        B_SCRN.printAt(0, 60, "kD: %.3f", move_kd);
        B_SCRN.printAt(0, 100, "imu: %.3f", imu_rotation());

        wait(20, vex::msec);
    }
}
