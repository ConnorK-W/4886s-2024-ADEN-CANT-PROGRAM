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
        vex::thread t1(intake);
        lift.set(0);
        tounge.set(1);
        drive_straight((41.5-17+9.5), 70, 100);



        // turn_pid(90, -1, 1);
        // drive_straight(10, 75, 50);

        // vex::thread turnTask([](){
        //     finger.set(1);
        //     tounge.set(1);
        //     turn_pid(90, -1, 1);
        // });
        // wait(600, TIME_MSEC);
        // turnTask.interrupt();

        finger.set(1);
        drive_turn(90, 14, 40, 75, false);

        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(600, TIME_MSEC);
        // long goal
        drive_straight_toward_goal(1000, 0);
        tounge.set(0);
        scoring = 4;
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        wait(600, TIME_MSEC);

        // middle goal
        scoring = 1;
        drive_double_turn(93, 5, 22.5, 75, -16.5, -105, 30, 75, false);
        tounge.set(1);
        // turn_pid(-25, -1, 1);
        drive_turn(-30, -19 , 30, 50);
        drive_straight_toward_goal(1000, 1);
        scoring = 3;
        wait(730, TIME_MSEC);
        scoring = 1;
        hood.set(0);
        lift.set(0);
        t1.interrupt();
        arm.spin(DIR_REV, 100, VEL_PCT);
        drive_straight(39, 75, 130);
        drive_turn(-45, -25, 50, 75);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(600, TIME_MSEC);
        // long goal
        drive_straight_toward_goal(1000, 0);
        tounge.set(0);
        hood.set(1);
        arm.spin(DIR_FWD, 100, VEL_PCT);
        drive_full.spin(DIR_REV, 50, VEL_PCT);

        break;
    }


    case RightComplex: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(41.5, 70, 100);
        tounge.set(1);
        turn_pid(90, -1, 1);
        drive_straight(10, 75, 50);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        // long goal
        drive_straight(-28, 75, 100);
        tounge.set(0);
        scoring = 4;
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        wait(500, TIME_MSEC);
        tounge.set(0);
        scoring = 1;
        drive_straight(2, 75, 100);
        drive_turn(-90, -12.5, 50, 75, false);
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-27, 75, 50);

        break;
    }


    case RightSimple: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(20, 50, 70);
        drive_turn(45, 9.5, 30, 75, false);
        tounge.set(1);
        drive_turn(-90, -11, 30, 75, false);
        tounge.set(0);
        drive_straight(9, 50, 100);
        scoring = 2;
        wait(700, TIME_MSEC);
        intakeFull.spin(DIR_FWD, 12, VLT_VLT);
        scoring = 1;
        drive_straight(-47.5, 75, 100);
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
        drive_turn(-90, -13, 50, 75, false);
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-27, 75, 50);

        break;
    }


    case LeftSimple: {
        vex::thread t1(intake);
        lift.set(0);
        drive_straight(16, 75, 100);
        drive_turn(-55, -21, 15, 75, false);
        tounge.set(1);
        turn_pid(-80, -1, 1);
        drive_straight(-14, 30, 100);
        scoring = 3;
        wait(800, TIME_MSEC);
        lift.set(1);
        scoring = 1;
        lift.set(1);
        drive_straight(44, 50, 100);
        lift.set(1);
        turn_pid(-45, -1, 1);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1400, TIME_MSEC);
        drive_straight(-35, 30, 100);
        scoring = 4;
        wait(600, TIME_MSEC);
        scoring = 1;
        //drive_turn(-90, -13.5, 50, 75, false);
        /*
t1.interrupt();
turn_pid(90, -1, 1);
drive_straight(-29, 75, 50);
*/
        break;
    }

    case SKILLS: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(41, 70, 100);
        finger.set(1);
        tounge.set(1);
        turn_pid(-90, -1, 1);
        drive_straight(10, 75, 50);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(800, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        // long goal
        drive_straight(-25, 30, 100);
        drive_turn(-180, -12, 50, 75, false);
        drive_straight(77, 60, 100);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(1300, TIME_MSEC);

        // first goal
        drive_straight(9, 30, 100);
        turn_pid(-90, -1, 1);
        drive_straight(-13, 30, 100);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // second match load
        tounge.set(1);
        drive_straight(20, 30, 100);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(800, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_straight(-30, 30, 100);
        scoring = 6;
        drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        drive_straight(5, 30, 100);
        scoring = 0;
        drive_straight(-5, 10, 50);

        // second half
        drive_turn(90, 15, 50, 75, false);
        drive_straight(88, 60, 100);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(1000, TIME_MSEC);
        drive_straight(-15, 30, 100);
        tounge.set(1);
        scoring = 1;
        turn_pid(-90, -1, 1);
        drive_straight(8, 50, 100);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(800, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_straight(-25, 30, 100);
        drive_turn(-182, -12, 50, 75, false);
        drive_straight(77, 60, 100);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(1400, TIME_MSEC);

        // last goal
        drive_straight(9, 30, 100);
        turn_pid(-90, -1, 1);
        drive_straight(-11.5, 30, 100);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // last match load
        tounge.set(1);
        drive_straight(20, 30, 100);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(800, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_straight(-35, 30, 100);
        scoring = 6;
        drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        drive_straight(5, 30, 100);
        scoring = 8;
        drive_straight(-5, 10, 50);
        drive_straight(1, 30, 100);
        drive_turn(80, 38, 50, 75, false);
        drive_full.spinFor(DIR_FWD, 1050, TIME_MSEC, 50, VEL_PCT);
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

    while (true) {
        switch (scoring) {
        case 0: {
            intakeLow.stop();
            hood.set(0);
            if (arm.position(ROT_DEG) > 40) {
                arm.spinToPosition(8 * 3, ROT_DEG, 100, VEL_PCT, false);
                intakeLow.spin(DIR_REV, 100, VEL_PCT);
            }
            break;
        }
        case 1: {
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            hood.set(0);
            if (arm.position(ROT_DEG) > 40) {
                arm.spinToPosition(8 * 3, ROT_DEG, 100, VEL_PCT, false);
                intakeLow.spin(DIR_REV, 100, VEL_PCT);
            }
            break;
        }
        case 2: {
            intakeLow.spin(DIR_REV, 40, VEL_PCT);
            hood.set(1);
            break;
        }
        case 3: {
            lift.set(1);
            hood.set(1);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            arm.spin(DIR_FWD, 30, VEL_PCT);
            break;
        }
        case 4: {
            lift.set(0);
            hood.set(1);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            arm.spin(DIR_FWD, 100, VEL_PCT);
            break;
        }
        case 5: {
            lift.set(0);
            hood.set(1);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            arm.spin(DIR_FWD, 50, VEL_PCT);
            break;
        }
        case 6: {
            lift.set(0);
            hood.set(1);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            arm.spin(DIR_FWD, 25, VEL_PCT);
            break;
        }
        case 7: {
            hood.set(0);
            arm.spin(DIR_REV, 12, VLT_VLT);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            drive_full.spin(DIR_FWD, 4, VLT_VLT);
            wait(700, TIME_MSEC);
            drive_full.spin(DIR_REV, 5, VLT_VLT);
            wait(100, TIME_MSEC);
            drive_full.spin(DIR_FWD, 4, VLT_VLT);
            wait(300, TIME_MSEC);
            drive_full.spin(DIR_REV, 5, VLT_VLT);
            wait(100, TIME_MSEC);
            drive_full.spin(DIR_FWD, 4, VLT_VLT);
            wait(500, TIME_MSEC);
            drive_full.spin(DIR_REV, 5, VLT_VLT);
            wait(100, TIME_MSEC);
            drive_full.spin(DIR_FWD, 6, VLT_VLT);
            wait(800, TIME_MSEC);
            break;
        }
        case 8: {
            lift.set(0);
            hood.set(0);
            intakeLow.spin(DIR_REV, 100, VEL_PCT);
            break;
        }
        }
        wait(50, TIME_MSEC);
    }
}
