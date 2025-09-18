#include "../include/main.h"
#include "stddefs.h"
#include "vex_global.h"
#include "vex_units.h"

void intake(void);

void autonomous(void) {
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }

reset_imu_rotation();

drive_full.setStopping(vex::hold);
lift.setStopping(vex::hold);
lift.resetPosition();

    switch (auton_mode) {
    case AWP: {

        break;
    }

    case RightComplex: {

vex::thread t1 (intake);
drive_straight(28, 50, 70);
turn_pid(45, -1, 1);
drive_turn(-30, -52, 10, 75, false);
turn_pid(50, -1, 1);
drive_straight(16, 15, 100);
tounge.set(1);
wait(500, TIME_MSEC);
drive_straight(-20, 30, 100);
turn_pid(73, -1, 1);
drive_straight(39, 30, 100);
turn_pid(43, -1, 1);
drive_straight(-10, 30, 100);
drive_full.spinFor(DIR_REV, 300, TIME_MSEC, 40, VEL_PCT);
intakeFull.spin(DIR_FWD, 100, VEL_PCT);
drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 40, VEL_PCT);
intakeHigh.stop();
drive_straight(33, 50, 100);
drive_full.spin(DIR_FWD, 1, VLT_VLT);
wait(1000, TIME_MSEC);
drive_straight(-33, 50, 100);
intakeFull.spin(DIR_FWD, 100, VEL_PCT);
drive_full.spinFor(DIR_REV, 2000, TIME_MSEC, 40, VEL_PCT);




t1.interrupt();

        break;
    }

    case SKILLS: {

        break;
    }
}
}


void intake() {
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;
    while (true){
        if (intakeLow.velocity(VEL_PCT) < 10)
            antiJamTime ++;
        if (intakeLow.velocity(VEL_PCT) > 10)
            antiJamTime = 0;
        if (antiJamTime == 10){
            intakeLow.spinFor(DIR_REV, 200, TIME_MSEC, 100, VEL_PCT);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            wait(200, TIME_MSEC);
            antiJamTime = 0;
        }
        wait (20, TIME_MSEC);
    }
}