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

    switch (auton_mode) {
    case AWP: {
drive_straight(43, 70, 100);
lift.set(1);
finger.set(1);
tounge.set(1);
turn_pid(90, -1, 1);
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1400, TIME_MSEC);
// long goal
drive_straight(-30, 75, 50);
intakeLow.spin(DIR_REV, 12, VLT_VLT);
tounge.set(0);
arm.spin(DIR_FWD, 12, VLT_VLT);
wait(600, TIME_MSEC);
arm.spin(DIR_REV, 12, VLT_VLT);
// middle goal
drive_turn(135, 11, 40, 75, false);
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
arm.spin(DIR_REV, 4, VLT_VLT);
drive_straight(35, 40, 100);
intakeLow.spin(DIR_REV, 12, VLT_VLT);
wait(500, TIME_MSEC);
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(-18, 75, 100);
turn_pid(-45, -1, 1);
drive_straight(37, 75, 100);
tounge.set(1);
drive_turn(45, 17, 50, 100, false);
turn_pid(-90, -1, 1);
drive_straight(32, 75, 100);
turn_pid(-45, -1, 1);
drive_straight(-20, 50, 100);
arm.resetPosition();
arm.spin(DIR_FWD, 12, VLT_VLT);
wait(500, TIME_MSEC);
arm.spin(DIR_REV, 12, VLT_VLT);
drive_straight(10, 50, 100);


        break;
    }



    case RightComplex: {
finger.set(1);
vex::thread t1 (intake);
drive_straight(28, 50, 70);
turn_pid(40, -1, 1);
drive_straight(10, 10, 100);
drive_straight(16.5, 40, 100);
drive_turn(25, 22, 20, 75, false);
drive_straight(3, 15, 100);
wait(200, TIME_MSEC);
drive_straight(-20, 50, 100);
turn_pid(73, -1, 1);
drive_straight(36, 50, 100);
turn_pid(45, -1, 1);
drive_straight(-13, 50, 100);
tounge.set(1);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
drive_full.spinFor(DIR_REV, 1400, TIME_MSEC, 30, VEL_PCT);
intakeHigh.stop();
drive_straight(31, 50, 100);
drive_full.spin(DIR_FWD, 2, VLT_VLT);
wait(500, TIME_MSEC);
tounge.set(0);
drive_straight(-33, 40, 100);
t1.interrupt();
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
drive_full.spin(DIR_REV, 2, VLT_VLT);
wait(1000, TIME_MSEC);
intakeFull.spin(DIR_REV, 12, VLT_VLT);
wait(100, TIME_MSEC);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(500, TIME_MSEC);
drive_straight(10, 50, 100);

        break;
    }


    case RightSimple: {
finger.set(1);
lift.set(1);
arm.spin(DIR_REV, 4, VLT_VLT);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(29, 50, 70);
turn_pid(47, -1, 1);
drive_straight(10, 5, 100);
turn_pid(-90, -1, 1);
drive_straight(15, 20, 100);
intakeFull.spin(DIR_REV, 12, VLT_VLT);
wait(1100, TIME_MSEC);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
tounge.set(1);
drive_straight(-46, 40, 100);
turn_pid(-135, -1, 1);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1800, TIME_MSEC);
// long goal
drive_straight(-30, 75, 50);
wait(1500, TIME_MSEC);
arm.spin(DIR_FWD, 12, VLT_VLT);
wait(700, TIME_MSEC);
intakeLow.spin(DIR_REV, 12, VLT_VLT);
arm.spin(DIR_REV, 12, VLT_VLT);
drive_straight(10, 75, 50);


        break;
    }


    case LeftSimple: {

vex::thread t1 (intake);
drive_straight(28, 50, 70);
turn_pid(-40, -1, 1);
drive_straight(12, 8, 100);
tounge.set(1);
turn_pid(-95, -1, 1);
drive_straight(-15, 30, 100);
intakeFull.spin(DIR_FWD, 8, VLT_VLT);
wait(650, TIME_MSEC);
intakeHigh.spinFor(DIR_REV, 200, ROT_DEG, false);
intakeHigh.stop();
drive_straight(49, 50, 100);
turn_pid(-45, -1, 1);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1400, TIME_MSEC);
drive_straight(-35, 30, 100);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(2000, TIME_MSEC);
drive_straight(10, 50, 100);
drive_full.spinFor(DIR_REV, 500, TIME_MSEC, 100, VEL_PCT);


        break;
    }

    case SKILLS: {

intakeLow.spin(DIR_FWD, 12, VLT_VLT);



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