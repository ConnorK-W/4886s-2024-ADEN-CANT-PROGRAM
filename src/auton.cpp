#include "../include/main.h"
#include "stddefs.h"
#include "vex_global.h"
#include "vex_units.h"
int scoring = 1;

void intake(void);

void autonomous(void) {
    while (imu.isCalibrating()) {
        wait(20, vex::msec);
    }

reset_imu_rotation();
drive_full.setStopping(vex::hold);

    switch (auton_mode) {
    case AWP: {
vex::thread t1 (intake);
lift.set(1);
drive_straight(28, 50, 70);
drive_turn(45, 9.5, 30, 75, false);
tounge.set(1);
drive_turn(-90, -11, 30, 75, false);
tounge.set(0);
drive_straight(6, 50, 100);
scoring = 2;
wait(500, TIME_MSEC);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
scoring = 1;
drive_straight(-10, 75, 100);
turn_pid(-45, -1, 1);
drive_straight(37.5, 70, 100);
tounge.set(1);
drive_turn(45, 11, 20, 75, false);
turn_pid(-90, -1, 1);
lift.set(0);
drive_straight(-11.5, 40, 100);
scoring = 3;
wait(800, TIME_MSEC);
scoring = 1;
lift.set(1);
drive_straight(44, 75, 100);
lift.set(1);
turn_pid(-45, -1, 1);
drive_straight(5, 75, 100);
t1.interrupt();
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1600, TIME_MSEC);
drive_straight(-30, 75, 100);
wait(300, TIME_MSEC);
arm.spin(DIR_FWD, 12, VLT_VLT);
        break;
    }



    case RightComplex: {
vex::thread t1 (intake);
drive_straight(29, 50, 70);
turn_pid(52, -1, 1);
drive_straight(35, 15, 70);
drive_straight(-25, 50, 70);
turn_pid(-95, -1, 1);
drive_straight(20, 50, 70);
drive_straight(-5, 15, 70);
scoring = 2;
wait(800, TIME_MSEC);
scoring = 1;
drive_straight(-43, 60, 70);
tounge.set(1);
turn_pid(-135, -1, 1);
t1.interrupt();
lift.set(1);
intakeLow.spin(DIR_FWD, 12, VLT_VLT);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1600, TIME_MSEC);
drive_straight(-30, 75, 100);
wait(300, TIME_MSEC);
arm.spinFor(DIR_FWD, 500, TIME_MSEC, 100, VEL_PCT);
arm.spinFor(DIR_REV, 300, TIME_MSEC, 100, VEL_PCT);
arm.spinFor(DIR_FWD, 500, TIME_MSEC, 100, VEL_PCT);

        break;
    }


    case RightSimple: {


vex::thread t1 (intake);
lift.set(1);
drive_straight(28, 50, 70);
drive_turn(45, 9.5, 30, 75, false);
tounge.set(1);
drive_turn(-90, -10, 30, 75, false);
tounge.set(0);
drive_straight(7, 50, 100);
scoring = 2;
wait(700, TIME_MSEC);
intakeFull.spin(DIR_FWD, 12, VLT_VLT);
scoring = 1;
drive_straight(-42, 75, 100);
tounge.set(1);

// long goal
turn_pid(-135, -1, 1);
drive_straight(10, 75, 100);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(700, TIME_MSEC);
// long goal
drive_straight(-30, 75, 50);
scoring = 4;
drive_full.spinFor(DIR_REV, 650, TIME_MSEC, 50, VEL_PCT);
tounge.set(0);
scoring = 1;
drive_straight(2, 50, 100);
drive_turn(-90, -12.5, 50, 75, false);
t1.interrupt();
turn_pid(90, -1, 1);
drive_straight(-27, 75, 50);

        break;
    }


    case LeftSimple: {

vex::thread t1 (intake);
lift.set(0);
drive_straight(28, 50, 70);
turn_pid(-40, -1, 1);
drive_straight(9, 8, 100);
tounge.set(1);
turn_pid(-95, -1, 1);
drive_straight(-12.5, 30, 100);
scoring = 3;
wait(1000, TIME_MSEC);
scoring = 1;
drive_straight(46, 50, 100);
lift.set(1);
turn_pid(-45, -1, 1);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(1400, TIME_MSEC);
drive_straight(-35, 30, 100);
scoring = 4;
wait(1000, TIME_MSEC);
scoring = 1;
drive_straight(10, 40, 100);
t1.interrupt();

        break;
    }

    case SKILLS: {

vex::thread t1 (intake);
lift.set(1);
drive_straight(25, 50, 70);
turn_pid(-40, -1, 1);
drive_straight(12, 15, 100);
tounge.set(1);
turn_pid(-96, -1, 1);
drive_straight(29, 30, 100);
turn_pid(-45, -1, 1);
drive_straight(-15, 30, 100);
drive_full.spin(DIR_REV, 4, VLT_VLT);
scoring = 4;
drive_full.spinFor(DIR_REV, 600, TIME_MSEC, 50, VEL_PCT);
scoring = 1;
tounge.set(1);
drive_straight(28, 30, 100);
intakeHigh.stop();
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(2000, TIME_MSEC);
scoring = 0;
drive_straight(-25, 30, 100);
drive_turn(-180, -15, 50, 75, false);
drive_straight(77, 60, 100);
turn_pid(-90, -1, 1);
tounge.set(0);
drive_full.spin(DIR_FWD, 6, VLT_VLT);
wait(1300, TIME_MSEC);

// first goal
drive_straight(-8.5, 30, 100);
turn_pid(92, -1, 1);
drive_straight(-13, 30, 100);
scoring = 5;
drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
scoring = 1;
// second match load
tounge.set(1);
drive_straight(26, 30, 100);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(2500, TIME_MSEC);
drive_straight(-35, 30, 100);
scoring = 6;
drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
scoring = 1;

// second half
drive_turn(90, 15, 50, 75, false);
tounge.set(0);
drive_straight(90, 60, 100);
drive_full.spin(DIR_FWD, 6, VLT_VLT);
wait(1000, TIME_MSEC);
drive_straight(-15, 30, 100);
turn_pid(-90, -1, 1);
tounge.set(1);
drive_straight(8, 50, 100);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(2500, TIME_MSEC);
scoring = 0;
drive_straight(-25, 30, 100);
drive_turn(-180, -15, 50, 75, false);
drive_straight(77, 60, 100);
tounge.set(0);
turn_pid(-90, -1, 1);
drive_full.spin(DIR_FWD, 6, VLT_VLT);
wait(1400, TIME_MSEC);

// last goal
drive_straight(-9, 30, 100);
turn_pid(90, -1, 1);
drive_straight(-11.5, 30, 100);
scoring = 5;
drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
scoring = 1;
// last match load
tounge.set(1);
drive_straight(26, 30, 100);
drive_full.spin(DIR_FWD, 4, VLT_VLT);
wait(2500, TIME_MSEC);
drive_straight(-35, 30, 100);
scoring = 6;
drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
tounge.set(0);
scoring = 7;
drive_straight(1, 30, 100);
drive_turn(80, 38, 50, 75, false);
drive_full.spinFor(DIR_FWD, 1100, TIME_MSEC, 50, VEL_PCT);
intakeLow.spin(DIR_REV, 100, VEL_PCT);


        break;
    }
}
}


void intake() {
    arm.spinFor(DIR_REV, 500, TIME_MSEC, 100, VEL_PCT);
    arm.resetPosition();
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;

    while (true){

    switch (scoring) {
    case 0:{
        intakeLow.stop();
        if (arm.position(ROT_DEG) > 40){
            arm.spinToPosition(8 * 3, ROT_DEG, 100, VEL_PCT, false);
            intakeLow.spin(DIR_REV, 100, VEL_PCT);
        }
        break;

    }
    case 1:{
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        if (arm.position(ROT_DEG) > 40){
            arm.spinToPosition(8 * 3, ROT_DEG, 100, VEL_PCT, false);
            intakeLow.spin(DIR_REV, 100, VEL_PCT);
        }
        break;
    }        
    case 2:{
        intakeLow.spin(DIR_REV, 40, VEL_PCT);
        break;
    }
    case 3:{
        lift.set(0);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        arm.spin(DIR_FWD, 30, VEL_PCT);        
        break;
    }
    case 4:{
        lift.set(1);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        arm.spin(DIR_FWD, 100, VEL_PCT);        
        break;
    }
    case 5:{
        lift.set(1);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        arm.spin(DIR_FWD, 50, VEL_PCT);        
        break;
    }
    case 6:{
        lift.set(1);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        arm.spin(DIR_FWD, 25, VEL_PCT);        
        break;
    }
    case 7:{
        lift.set(1);
        intakeLow.spin(DIR_REV, 100, VEL_PCT);
        break;
    }
    }
}
wait(50, TIME_MSEC);
}