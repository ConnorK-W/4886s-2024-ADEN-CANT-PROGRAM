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

    switch (auton_mode) {
    case AWP_Red: {

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

    case AWP_Blue: {

lift.spinToPosition(164 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-35.5, 50, 50);
lift.spinToPosition(80 * 4, ROT_DEG, 150, VEL_RPM, false);
wait(100, TIME_MSEC);
mogo_clamp.set(1);
// quad stack
turn_pid(-167, -1, 1);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechR.set(1);
drive_straight(16.5, 35, 100);
wait(200, TIME_MSEC);
drive_straight(-18, 35, 100);
vex::thread t1 (blue_sort);
Smith_MechR.set(0);
drive_turn(225, 19, 50, 75, false);
intake_lift.set(1);
drive_straight(48, 20, 100);
t1.interrupt();
intake.stop();
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
intake_lift.set(0);
mogo_clamp.set(0);
wait(400, TIME_MSEC);
turn_pid(-115, -1, 1);
lift.spinToPosition(115 * 4, ROT_DEG, 150, VEL_RPM, false);
drive_straight(-35, 40, 100);
mogo_clamp.set(1);
intake.spin(DIR_FWD, 100, PCT_PCT);
drive_turn(-105, -12, 20, 75, false);
lift.spinToPosition(150 * 4, ROT_DEG, 150, VEL_RPM, false);

        break;
    }

    case Red_Rush_Rings: {

lift.spinToPosition(80 * 3, ROT_DEG, 150, VEL_RPM, false);
intakeLow.spin(DIR_FWD, 100, PCT_PCT);
Smith_MechL.set(1);
drive_straight(43, 70, 100);
turn_pid(-40, -1, 1);
drive_straight(-28, 50, 50);
mogo_clamp.set(1);
vex::thread t1 (red_sort);
Smith_MechL.set(0);
wait(200, TIME_MSEC);
turn_pid(-30, -1, 1);
drive_straight(35, 25, 100);
drive_straight(-5, 75, 100);
turn_pid(-60, -1, 1);
drive_straight(35, 75, 100);
drive_full.spinFor(DIR_FWD, 500, TIME_MSEC, 40, VEL_PCT);
drive_straight(-6, 75, 100);
drive_turn(60, -10, 40, 75);
turn_pid(-180, -1, 1);
drive_straight(35, 75, 100);
intake_lift.set(1);
drive_straight(25, 75, 100);
intake_lift.set(0);
drive_straight(-3, 75, 100);
turn_pid(-90, -1, 1);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(8, 75, 100);
t1.interrupt();

        break;
    }

    case Blue_Rush_Rings: {

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

    case Blue_Rush_Goal: {

// 157.5
drive_straight(-46, 75, 120);
mogo_clamp.set(1);
wait(150, TIME_MSEC);
drive_straight(3, 75, 100);
turn_pid(50, -1, 1);
intake.spin(DIR_FWD, 100, VEL_PCT);
drive_straight(7, 75, 100);
intakeHigh.stop();
turn_pid(-115, -1, 1);
drive_straight(-5, 75, 100);
mogo_clamp.set(0);
drive_straight(5, 75, 100);
turn_pid(180, -1, 1);
drive_straight(-23, 75, 100);
mogo_clamp.set(1);
wait(150, TIME_MSEC);
turn_pid(-80, -1, 1);
vex::thread t1 (blue_sort);
drive_straight(33, 75, 100);
turn_pid(60, -1, 1);
drive_full.spinFor(DIR_FWD, 1000, TIME_MSEC, 400, VEL_RPM);
drive_straight(-17, 75, 100);
Smith_MechL.set(1);
intakeLow.stop();
drive_straight(5, 75, 100);
drive_turn(150, 10, 40, 75, false);
t1.interrupt();
intake.stop();
lift.spinToPosition(164 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(50, 40, 100);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);


        break;
    }

    case Red_Rush_Goal: {
        
drive_straight(-46, 75, 120);
mogo_clamp.set(1);
wait(150, TIME_MSEC);
drive_straight(3, 75, 100);
turn_pid(-50, -1, 1);
intake.spin(DIR_FWD, 100, VEL_PCT);
drive_straight(7, 75, 100);
intakeHigh.stop();
turn_pid(115, -1, 1);
drive_straight(-5, 75, 100);
mogo_clamp.set(0);
drive_straight(5, 75, 100);
turn_pid(-180, -1, 1);
drive_straight(-23, 75, 100);
mogo_clamp.set(1);
wait(150, TIME_MSEC);
turn_pid(80, -1, 1);
vex::thread t1 (red_sort);
drive_straight(33, 75, 100);
turn_pid(-50, -1, 1);
drive_full.spinFor(DIR_FWD, 1000, TIME_MSEC, 400, VEL_RPM);
drive_straight(-17, 75, 100);
Smith_MechR.set(1);
intakeLow.stop();
drive_straight(5, 75, 100);
drive_turn(-150, 10, 40, 75, false);
t1.interrupt();
intake.stop();
lift.spinToPosition(164 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(50, 40, 100);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);

        break;
    }

    case Red_Quad: {
// spin to 332.5
lift.spinToPosition(164 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-42, 40, 100);
lift.spinToPosition(70 * 3, ROT_DEG, 150, VEL_RPM, false);
mogo_clamp.set(1);
wait(300, TIME_MSEC);
vex::thread t1 (red_sort);
// quad stack
turn_pid(145, -1, 1);
drive_straight(13, 60, 100);
turn_pid(-28, -1, 1);
drive_straight(16, 60, 100);
drive_straight(-2, 60, 100);
// Single ring
turn_pid(-110, -1, 1);
drive_straight(15, 40, 100);
turn_pid(40, -1, 1);
drive_straight(35, 75, 100);
drive_full.spinFor(DIR_FWD, 700, TIME_MSEC, 400, VEL_RPM);
drive_straight(-10, 75, 100);
turn_pid(-110, -1, 1);
intake_lift.set(1);
drive_straight(54, 75, 100);
intake_lift.set(0);
turn_pid(-90, -1, 1);
lift.spinToPosition(120 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_full.spinFor(DIR_FWD, 800, TIME_MSEC, 20, VEL_PCT);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);
t1.interrupt();

        break;
    }

    case Blue_Quad: {

lift.spinToPosition(164 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-44, 40, 100);
lift.spinToPosition(70 * 3, ROT_DEG, 150, VEL_RPM, false);
mogo_clamp.set(1);
wait(300, TIME_MSEC);
vex::thread t1 (blue_sort);
// quad stack
turn_pid(-153, -1, 1);
drive_straight(11.5, 60, 100);
turn_pid(35, -1, 1);
drive_straight(16, 60, 100);
drive_straight(-4, 60, 100);
// Single ring
turn_pid(110, -1, 1);
drive_straight(15, 40, 100);
turn_pid(-40, -1, 1);
drive_straight(35, 75, 100);
drive_full.spinFor(DIR_FWD, 700, TIME_MSEC, 400, VEL_RPM);
drive_straight(-10, 75, 100);
turn_pid(110, -1, 1);
drive_straight(35, 75, 100);
intake_lift.set(1);
drive_straight(5, 30, 100);
intake_lift.set(0);
turn_pid(90, -1, 1);
lift.spinToPosition(120 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_full.spinFor(DIR_FWD, 700, TIME_MSEC, 30, VEL_PCT);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);
t1.interrupt();

        /*
// spin to 26.5
lift.spinToPosition(164 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_straight(4, 50, 50);
wait(250, TIME_MSEC);
// pickup ring 2 with smith mech
drive_straight(-40, 40, 100);
lift.spinToPosition(70 * 3, ROT_DEG, 150, VEL_RPM, false);
mogo_clamp.set(1);
wait(300, TIME_MSEC);
vex::thread t1 (blue_sort);
// quad stack
turn_pid(-155, -1, 1);
drive_straight(12, 60, 100);
turn_pid(45, -1, 1);
drive_straight(16, 60, 100);
drive_straight(-2, 60, 100);
// Single ring
turn_pid(110, -1, 1);
drive_straight(15, 40, 100);
turn_pid(-50, -1, 1);
drive_straight(30, 75, 100);
drive_full.spinFor(DIR_FWD, 700, TIME_MSEC, 400, VEL_RPM);
drive_straight(-10, 75, 100);
turn_pid(120, -1, 1);
intake_lift.set(1);
drive_straight(54, 75, 100);
intake_lift.set(0);
turn_pid(90, -1, 1);
lift.spinToPosition(120 * 3, ROT_DEG, 150, VEL_RPM, false);
drive_full.spinFor(DIR_FWD, 500, TIME_MSEC, 30, VEL_PCT);
lift.spinToPosition(180 * 3, ROT_DEG, 150, VEL_RPM, false);
t1.interrupt();
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
turn_pid(10, -1, 1);
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
    int antiJamTime = 0;
    while (true){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        if (intakeHigh.velocity(VEL_PCT) < 1)
            antiJamTime ++;
        if (intakeHigh.velocity(VEL_PCT) > 1)
            antiJamTime = 0;
        if (antiJamTime == 10){
            intakeHigh.spinFor(DIR_REV, 200, TIME_MSEC, 100, VEL_PCT);
            intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
            wait(200, TIME_MSEC);
            antiJamTime = 0;            
        }

        if (colorSort.isNearObject() && colorSort.color() == vex::color::red) {
            intake.spinFor(DIR_REV, 150, TIME_MSEC, 100, VEL_PCT);
        }
    }
}

void red_sort() {
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;
    while (true){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        if (intakeHigh.velocity(VEL_PCT) < 1)
            antiJamTime ++;
        if (intakeHigh.velocity(VEL_PCT) > 1)
            antiJamTime = 0;
        if (antiJamTime == 10){
            intakeHigh.spinFor(DIR_REV, 200, TIME_MSEC, 100, VEL_PCT);
            intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
            wait(200, TIME_MSEC);
            antiJamTime = 0;            
        }

        if (colorSort.isNearObject() && colorSort.color() == vex::color::blue){
            intake.spinFor(DIR_REV, 150, TIME_MSEC, 100, VEL_PCT);

        }
        wait(20, TIME_MSEC);
    }
}