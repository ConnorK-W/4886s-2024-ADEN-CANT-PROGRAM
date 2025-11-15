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
drive_straight(28.5, 40, 100);
finger.set(1);
tounge.set(1);
turn_pid(92, -1, 1);
vex::thread t1 (intake);
drive_full.spin(DIR_FWD, 3, VLT_VLT);
wait(1400, TIME_MSEC);
// long goal
drive_straight(-33, 50, 50);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(700, TIME_MSEC);
tounge.set(0);
// middle balls
drive_turn(150, 12, 50, 75, false);
intakeHigh.stop();
drive_turn(-60, -15, 50, 75, false);

drive_straight(37, 65, 100);
drive_turn(45, 14.5, 50, 75, false);
tounge.set(1);
turn_pid(-90, -1, 1);
drive_straight(-11.5, 50, 100);
intakeHigh.spin(DIR_FWD, 7, VLT_VLT);
wait(450, TIME_MSEC);
intakeHigh.stop();
drive_straight(48, 65, 100);
tounge.set(0);
turn_pid(-45, -1, 1);
drive_straight(-18.5, 65, 100);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(1200, TIME_MSEC);
drive_straight(6, 65, 100);



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

vex::thread t1 (intake);
drive_straight(25, 50, 70);
turn_pid(-40, -1, 1);
drive_straight(12, 15, 100);
tounge.set(1);
turn_pid(-96, -1, 1);
drive_straight(33.5, 30, 100);
turn_pid(-45, -1, 1);
drive_straight(-22, 30, 100);
drive_full.spin(DIR_REV, 4, VLT_VLT);
intakeHigh.spin(DIR_FWD, 12, VLT_VLT);
wait(1200, TIME_MSEC);
tounge.set(1);
drive_straight(32, 30, 100);
intakeHigh.stop();
drive_full.spin(DIR_FWD, 2, VLT_VLT);
wait(2000, TIME_MSEC);
drive_straight(-25, 30, 100);
drive_turn(-180, -15, 50, 75, false);
intakeLow.stop();
t1.interrupt();
intakeLow.stop();
drive_straight(77, 60, 100);
turn_pid(-90, -1, 1);
tounge.set(0);
drive_full.spin(DIR_FWD, 9, VLT_VLT);
wait(1000, TIME_MSEC);

// first goal
drive_straight(-9.5, 30, 100);
turn_pid(92, -1, 1);
drive_straight(-13, 30, 100);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(2500, TIME_MSEC);
// second match load
tounge.set(1);
drive_straight(33, 30, 100);
intakeHigh.stop();
drive_full.spin(DIR_FWD, 2, VLT_VLT);
wait(2500, TIME_MSEC);
drive_straight(-35, 30, 100);
tounge.set(0);
intakeHigh.spin(DIR_FWD, 12, VLT_VLT);
wait(2500, TIME_MSEC);
// second half
drive_turn(87, 15, 50, 75, false);
drive_straight(105, 60, 100);
drive_full.spin(DIR_FWD, 6, VLT_VLT);
wait(1000, TIME_MSEC);

drive_straight(-11.5, 30, 100);
turn_pid(-90, -1, 1);
tounge.set(1);
intakeHigh.stop();
drive_straight(14, 50, 100);
drive_full.spin(DIR_FWD, 2, VLT_VLT);
wait(2500, TIME_MSEC);
drive_straight(-25, 30, 100);
drive_turn(-180, -15, 50, 75, false);
t1.interrupt();
intakeLow.stop();
drive_straight(77, 60, 100);
tounge.set(0);
turn_pid(-90, -1, 1);
drive_full.spin(DIR_FWD, 6, VLT_VLT);
wait(1400, TIME_MSEC);

// last goal
drive_straight(-10, 30, 100);
turn_pid(90, -1, 1);
drive_straight(-13, 30, 100);
vex::thread t2 (intake);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
wait(2500, TIME_MSEC);
// last match load
tounge.set(1);
drive_straight(33, 30, 100);
intakeHigh.stop();
drive_full.spin(DIR_FWD, 2, VLT_VLT);
wait(3000, TIME_MSEC);
drive_straight(-35, 30, 100);
intakeHigh.spin(DIR_FWD, 12, VLT_VLT);
wait(2500, TIME_MSEC);
tounge.set(0);
drive_straight(1, 30, 100);
drive_turn(80, 38, 50, 75, false);
tounge.set(1);
drive_full.spinFor(DIR_FWD, 1800, TIME_MSEC, 50, VEL_PCT);


t2.interrupt();
intakeLow.spin(DIR_REV, 100, VEL_PCT);

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