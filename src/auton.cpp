#include "../include/main.h"
#include "stddefs.h"
#include "vex_global.h"
#include "vex_units.h"

void red_sort(void);
void blue_sort(void);

void autonomous(void) {
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }

drive_full.setStopping(vex::hold);
lift.setStopping(vex::hold);
lift.resetPosition();
intake_lift.set(0);
    switch (auton_mode) {
    case FULL_AWP: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-35.5, 50, 50);
lift.spinToPosition(80 * 4, ROT_DEG, 150, VEL_RPM, false);
wait(100, TIME_MSEC);
mogo_clamp.set(1);
// quad stack
turn_pid(167, -1, 1);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechL.set(1);
drive_straight(18, 35, 100);
wait(200, TIME_MSEC);
drive_straight(-18, 35, 100);
vex::thread t1 (red_sort);
Smith_MechL.set(0);
drive_turn(-225, -19, 50, 75, false);
intake_lift.set(1);
drive_straight(48, 20, 100);
t1.interrupt();
intake.stop();
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
intake_lift.set(0);
mogo_clamp.set(0);
wait(400, TIME_MSEC);
turn_pid(115, -1, 1);
lift.spinToPosition(115 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(-35, 40, 100);
mogo_clamp.set(1);
intake.spin(DIR_FWD, 100, PCT_PCT);
drive_turn(105, 12, 20, 75, false);
lift.spinToPosition(150 * 4, ROT_DEG, 150, VEL_RPM, false);


        break;
    }
    case BLUE_QUAD: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-35.5, 50, 50);
lift.spinToPosition(70 * 4, ROT_DEG, 150, VEL_RPM, false);
wait(100, TIME_MSEC);
mogo_clamp.set(1);
vex::thread t1 (blue_sort);
// quad stack
turn_pid(-84, -1, 1);
turn_pid(-84, -1, 1);
Smith_MechR.set(1);
drive_straight(18, 35, 100);
wait(200, TIME_MSEC);
turn_pid(-33, -1, 1);
turn_pid(40, -1, 1);
drive_straight(-15, 35, 100);
Smith_MechR.set(0);
turn_pid(35, -1, 1);
drive_straight(16, 30, 100);
drive_turn(172, 13, 40, 75, false);
intake_lift.set(1);
drive_straight(35, 75, 100);
drive_straight(5, 30, 100);
intake_lift.set(0);
turn_pid(-30, -1, 1);
drive_straight(-30, 30, 100);
turn_pid(90, -1, 1);
lift.spinToPosition(120 * 4, ROT_DEG, 200, VEL_RPM, false);
drive_straight(20, 30, 100);
t1.interrupt();

        break;
    }


    case BLUE_RUSH: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-33.5, 50, 50);
lift.spinToPosition(-25 * 4, ROT_DEG, 150, VEL_RPM, false);
mogo_clamp.set(1);
turn_pid(-104, -1, 1);
Smith_MechL.set(1);
drive_straight(19, 70, 100);
wait(100, TIME_MSEC);
drive_straight(-10, 75, 100);
drive_turn(-90, 19, 65, 75, true);
vex::thread t1 (blue_sort);
Smith_MechL.set(0);
wait(200, TIME_MSEC);
turn_pid(-30, -1, 1);
drive_straight(30, 30, 50);
turn_pid(-172, -1, 1);
intake_lift.set(1);
drive_straight(39, 75, 100);
drive_straight(5, 20, 100);
intake_lift.set(0);
drive_straight(-5, 50, 100);
turn_pid(134, -1, 1);
Smith_MechL.set(1);
drive_straight(40, 70, 100);
turn_pid(140, -1, 1);
Smith_MechL.set(0);
drive_turn(-20, -20, 75, 75, false);



        break;
    }


    case RED_RUSH: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-33.5, 50, 50);
lift.spinToPosition(-25 * 4, ROT_DEG, 150, VEL_RPM, false);
mogo_clamp.set(1);
turn_pid(104, -1, 1);
Smith_MechR.set(1);
drive_straight(19, 70, 100);
wait(100, TIME_MSEC);
drive_straight(-10, 75, 100);
drive_turn(90, -19, 65, 75, true);
vex::thread t1 (red_sort);
Smith_MechR.set(0);
wait(200, TIME_MSEC);
turn_pid(-30, -1, 1);
drive_straight(30, 30, 50);
turn_pid(172, -1, 1);
intake_lift.set(1);
drive_straight(39, 75, 100);
drive_straight(5, 20, 100);
intake_lift.set(0);
drive_straight(-5, 50, 100);
turn_pid(-134, -1, 1);
Smith_MechL.set(1);
drive_straight(40, 70, 100);
turn_pid(-140, -1, 1);
Smith_MechL.set(0);
drive_turn(20, 20, 75, 75, false);
        break;
    }


    case RED_QUAD: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-35.5, 50, 50);
lift.spinToPosition(80 * 4, ROT_DEG, 150, VEL_RPM, false);
wait(100, TIME_MSEC);
mogo_clamp.set(1);
vex::thread t1 (red_sort);
// quad stack
turn_pid(167, -1, 1);
Smith_MechL.set(1);
drive_straight(15.5, 35, 100);
wait(200, TIME_MSEC);
turn_pid(33, -1, 1);
turn_pid(-35, -1, 1);
drive_straight(-17, 35, 100);
Smith_MechL.set(0);
wait(300, TIME_MSEC);
turn_pid(-35, -1, 1);
drive_straight(14, 30, 100);
drive_turn(-180, -13, 40, 75, false);
intake_lift.set(1);
drive_straight(30, 75, 100);
drive_straight(5, 30, 100);
intake_lift.set(0);
turn_pid(55, -1, 1);
drive_straight(-20, 30, 100);
turn_pid(-90, -1, 1);
lift.spinToPosition(200 * 4, ROT_DEG, 200, VEL_RPM, false);
drive_straight(4, 30, 100);

        break;
    }


    case ELIMS_RED: {


lift.spin(DIR_REV, 20, PCT_PCT);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechL.set(1);
drive_straight(43, 70, 100);
turn_pid(-40, -1, 1);
drive_straight(-25, 50, 50);
mogo_clamp.set(1);
vex::thread t1 (red_sort);
Smith_MechL.set(0);
wait(200, TIME_MSEC);
turn_pid(-30, -1, 1);
drive_straight(25, 13, 100);
drive_straight(-10, 75, 100);
turn_pid(-87, -1, 1);
reset_imu_rotation();
drive_straight(30, 75, 200);
turn_pid(-90, -1, 1);
drive_straight(19, 75, 100);
intake_lift.set(1);
lift.resetPosition();
lift.spinToPosition(37 * 4, ROT_DEG, 200, VEL_RPM);
drive_straight(12, 15, 100);
intake_lift.set(0);
turn_pid(90, -1, 1);
drive_straight(2, 30, 100);
wait(500, TIME_MSEC);
t1.interrupt();
intake.spinFor(-180, ROT_DEG, 100, VEL_PCT, false);
lift.spinToPosition(200 * 4, ROT_DEG, 150, VEL_RPM);


// ring rush no wallstake
/*
lift.spin(DIR_REV, 20, PCT_PCT);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechL.set(1);
drive_straight(43, 70, 100);
turn_pid(-40, -1, 1);
drive_straight(-25, 50, 50);
mogo_clamp.set(1);
vex::thread t1 (red_sort);
Smith_MechL.set(0);
wait(200, TIME_MSEC);
turn_pid(-30, -1, 1);
drive_straight(29, 13, 100);
drive_straight(-13, 75, 100);
turn_pid(-85, -1, 1);
drive_straight(24, 75, 100);
turn_pid(-90, -1, 1);
drive_straight(15, 75, 100);
intake_lift.set(1);
drive_straight(25, 30, 100);
intake_lift.set(0);
turn_pid(-90, -1, 1);
turn_pid(105, -1, 1);
Smith_MechR.set(1);
drive_straight(41, 75, 100);
turn_pid(-90, -1, 1);
*/



        break;
    }

    case ELIMS_BLUE: {

lift.spin(DIR_REV, 20, PCT_PCT);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechR.set(1);
drive_straight(43, 70, 100);
turn_pid(40, -1, 1);
drive_straight(-25, 50, 50);
mogo_clamp.set(1);
vex::thread t1 (blue_sort);
Smith_MechR.set(0);
wait(200, TIME_MSEC);
turn_pid(30, -1, 1);
drive_straight(25, 13, 100);
drive_straight(-10, 75, 100);
turn_pid(85, -1, 1);
drive_straight(22, 75, 100);
wait(200, TIME_MSEC);
turn_pid(90, -1, 1);
drive_straight(15, 75, 100);
intake_lift.set(1);
lift.resetPosition();
lift.spinToPosition(37 * 4, ROT_DEG, 200, VEL_RPM);
drive_straight(19, 15, 100);
intake_lift.set(0);
turn_pid(-90, -1, 1);
drive_straight(2, 30, 100);
wait(500, TIME_MSEC);
t1.interrupt();
intake.spinFor(-180, ROT_DEG, 100, VEL_PCT, false);
lift.spinToPosition(200 * 4, ROT_DEG, 150, VEL_RPM);

//ring rush no wall stake
/*
lift.spin(DIR_REV, 20, PCT_PCT);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechR.set(1);
drive_straight(43, 70, 100);
turn_pid(40, -1, 1);
drive_straight(-25, 50, 50);
mogo_clamp.set(1);
vex::thread t1 (blue_sort);
Smith_MechR.set(0);
wait(200, TIME_MSEC);
turn_pid(30, -1, 1);
drive_straight(29, 13, 100);
drive_straight(-13, 75, 100);
turn_pid(-85, -1, 1);
drive_straight(24, 75, 100);
turn_pid(90, -1, 1);
drive_straight(15, 75, 100);
intake_lift.set(1);
drive_straight(25, 30, 100);
intake_lift.set(0);
turn_pid(90, -1, 1);
turn_pid(105, -1, 1);
Smith_MechL.set(1);
drive_straight(41, 75, 100);
turn_pid(90, -1, 1);
*/

        break;
    }


case SKILLS: {

// first ring
lift.spinToPosition(162 * 4, ROT_DEG, 150, VEL_RPM);
drive_straight(-16, 50, 50);
mogo_clamp.set(1);
wait(200, TIME_MSEC);
lift.spinToPosition(70 * 4, ROT_DEG, 200, VEL_RPM, false);
turn_pid(-137, -1, 1);
intake.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(28, 75, 75);
// NWS
turn_pid(-58, -1, 1);
drive_straight(29, 50, 75);
lift.spinToPosition(7 * 4, ROT_DEG, 200, VEL_RPM, false);
turn_pid(-30, -1, 1);
drive_straight(4, 20, 50);
drive_full.spinFor(DIR_FWD, 600, TIME_MSEC, 15, VEL_PCT);
lift.spinToPosition(130 * 4, ROT_DEG, 200, VEL_RPM, false);
intake.spinFor(-30, ROT_DEG, 100, VEL_PCT, false);
wait(200, TIME_MSEC);
drive_straight(-7, 30, 75);
wait(200, TIME_MSEC);
intake.spin(DIR_FWD, 12, VLT_VLT);
// Rings
drive_straight(-3, 75, 75);
turn_pid(-86, -1, 1);
drive_straight(65, 20, 75);
drive_straight(-4, 75, 75);
wait(250, TIME_MSEC);
drive_turn(90, -15, 30, 75, true);
drive_straight(23, 75, 75);
drive_full.spinFor(DIR_FWD, 250, TIME_MSEC, 30, VEL_PCT);
wait(200, TIME_MSEC);
reset_imu_rotation();
drive_straight(-15, 75, 75);
turn_pid(135, -1, 1);
drive_straight(-22, 75, 75);
mogo_clamp.set(0);
intake.spinFor(-90, ROT_DEG, 100, VEL_PCT);

// Goal 2
drive_straight(15, 75, 75);
turn_pid(-135, -1, 1);
lift.spinToPosition(50 * 4, ROT_DEG, 200, VEL_RPM, false);
drive_straight(-78, 75, 75);
mogo_clamp.set(1);
wait(200, TIME_MSEC);
drive_straight(1, 20, 75);
turn_pid(90, -1, 1);
// rings
intake.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(26, 75, 75);
// NWS
turn_pid(68, -1, 1);
drive_straight(35, 50, 75);
lift.spinToPosition(7 * 4, ROT_DEG, 200, VEL_RPM, false);
turn_pid(25, -1, 1);
drive_straight(5, 20, 50);
drive_full.spinFor(DIR_FWD, 600, TIME_MSEC, 15, VEL_PCT);
lift.spinToPosition(130 * 4, ROT_DEG, 200, VEL_RPM, false);
intake.spinFor(-30, ROT_DEG, 100, VEL_PCT, false);
wait(300, TIME_MSEC);
drive_straight(-7, 30, 75);
wait(200, TIME_MSEC);
intake.spin(DIR_FWD, 12, VLT_VLT);
// Rings
drive_straight(-5, 75, 75);
turn_pid(88, -1, 1);
drive_straight(65, 20, 75);
drive_straight(-4, 75, 75);
wait(250, TIME_MSEC);
drive_turn(-90, 15, 30, 75, true);
drive_straight(23, 75, 75);
drive_full.spinFor(DIR_FWD, 250, TIME_MSEC, 30, VEL_PCT);
wait(200, TIME_MSEC);
reset_imu_rotation();
drive_straight(-15, 75, 75);
turn_pid(-134, -1, 1);
drive_straight(-22, 75, 75);
mogo_clamp.set(0);
intake.spinFor(-90, ROT_DEG, 100, VEL_PCT);

// Goal 3
drive_straight(14, 50, 75);
turn_pid(46, -1, 1);
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(69, 75, 75);
turn_pid(-90, -1, 1);
intakeHigh.spinFor(600, ROT_DEG, 100, VEL_PCT, false);
drive_straight(25, 50, 75);
turn_pid(-135, -1, 1);
drive_straight(-34, 50, 75);
mogo_clamp.set(1);
intake.spin(DIR_FWD, 12, VLT_VLT);
drive_straight(-1, 50, 75);
turn_pid(150, -1, 1);
wait(1000, TIME_MSEC);
intake_lift.set(1);
drive_full.spinFor(DIR_FWD, 200, TIME_MSEC, 100, VEL_PCT);
mogo_clamp.set(0);
drive_full.spinFor(DIR_FWD, 1100, TIME_MSEC, 100, VEL_PCT);
intake.spin(DIR_REV, 12, VLT_VLT);
drive_straight(-65, 75, 300);
turn_pid(153, -1, 1);
lift.spinToPosition(60 * 4, ROT_DEG, 200, VEL_RPM, false);
drive_full.spinFor(DIR_FWD, 1300, TIME_MSEC, 80, VEL_PCT);
turn_pid(-0, -1, 1);
drive_straight(-1000, 50, 100000);


        break;
    }
    }
}

void blue_sort() {
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    while (true){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        if (colorSort.isNearObject() && colorSort.color() == vex::color::red) {
            intake.spinFor(DIR_REV, 1000, TIME_MSEC, 100, VEL_PCT);
        }
    }
}

void red_sort() {
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    while (true){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        if (colorSort.isNearObject() && colorSort.color() == vex::color::blue){
            intake.spinFor(DIR_REV, 1000, TIME_MSEC, 100, VEL_PCT);

        }
    }
}