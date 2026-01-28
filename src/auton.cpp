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
        drive_straight((41.5-17+9.5-3+1), 70, 100, true, 0, 20);
        finger.set(1);
        drive_turn(90, 14, 40, 75, false, 20, 0);

        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(625, TIME_MSEC);
        // long goal
        tounge.set(0);
        drive_straight_toward_goal(800, 0);
        scoring = 4;
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        wait(700, TIME_MSEC);

        // middle goal
        scoring = 1;
        drive_turn(91, 5, 25, 70, false);
        target_heading = 180;   
        drive_straight(52,45,45, true, 0, 20);
        drive_straight(11, 12, 50, true, 20, 6);
        tounge.set(1);
        turn_pid(-45, -1, 1);
        vex::thread t2([](){
            wait(600, TIME_MSEC);
            scoring = 3;
        });
        drive_straight_toward_goal(1500, 1);
        wait(700, TIME_MSEC);
        t2.interrupt();
        scoring = 1;
        hood.set(0);
        lift.set(0);
        t1.interrupt();
        arm.spin(DIR_REV, 100, VEL_PCT);
        drive_straight(46, 75, 100, true, 0, 0);
        // tounge.set(1);

        turn_pid(-44, -1, 1);
        drive_straight(7, 35, 40, true, 0, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(625, TIME_MSEC);
        // long goal
        vex::thread t3([](){
            wait(400, TIME_MSEC);
            arm.spin(DIR_FWD, 100, VEL_PCT);
        });
        drive_straight_toward_goal(1000, 0);
        
        tounge.set(0);
        hood.set(1);
        drive_full.spin(DIR_REV, 50, VEL_PCT);

        break;
    }


    case AWPPush: {
        vex::thread t1(intake);
        drive_straight(-5, 40, 100);
        lift.set(0);
        tounge.set(1);
        drive_straight((41.5-17+9.5-3+1+3), 70, 100, true, 0, 20);
        finger.set(1);
        drive_turn(90, 14, 40, 75, false, 20, 10);
        drive_straight(1.5, 10, 30, true, 10, 5);

        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(650, TIME_MSEC);
        // long goal
        tounge.set(0);
        drive_straight_toward_goal(800, 0);
        scoring = 4;
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        wait(700, TIME_MSEC);

        // middle goal
        scoring = 1;
        drive_turn(90, 5, 25, 70, false);
        target_heading = 180;   
        drive_straight(54,35,50, true, 0, 10);
        drive_straight(10, 6, 0, true, 12, 3);

        turn_pid(-45, -1, 1);
        drive_straight_toward_goal(1200, 1);
        scoring = 3;
        wait(800, TIME_MSEC);
        scoring = 1;
        hood.set(0);
        lift.set(0);
        t1.interrupt();
        arm.spin(DIR_REV, 100, VEL_PCT);
        drive_straight(51, 75, 130, true, 0, 0);
        tounge.set(1);
        turn_pid(-44, -1, 1);
        drive_straight_toward_goal(800, 0);
        hood.set(1);
        arm.spin(DIR_FWD, 100, VEL_PCT);
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        
        break;

        
        // GRAB TEAMMATE BALL
        // vex::thread t1(intake);
        // lift.set(0);
        // drive_straight(5, 30, 50);
        // tounge.set(1);
        // drive_straight(-51, 70, 100, true, 0, 0);



        // // turn_pid(90, -1, 1);
        // // drive_straight(10, 75, 50);

        // // vex::thread turnTask([](){
        // //     finger.set(1);
        // //     tounge.set(1);
        // //     turn_pid(90, -1, 1);
        // // });
        // // wait(600, TIME_MSEC);
        // // turnTask.interrupt();

        // finger.set(1);
        // turn_pid(-90, -1, 1);
        // drive_straight(7, 35, 40, true, 0, 15);
        // drive_full.spin(DIR_FWD, 4, VLT_VLT);
        // wait(700, TIME_MSEC);
        // // long goal
        // tounge.set(0);
        // drive_straight_toward_goal(1000, 0);
        // scoring = 4;
        // drive_full.spin(DIR_REV, 50, VEL_PCT);
        // wait(700, TIME_MSEC);

        // // middle goal
        // scoring = 1;
        // drive_turn(90, 5, 25, 70, false);
        // target_heading = 0;   
        // drive_straight(54,35,50, true, 0, 12);
        // drive_straight(8, 9, 50, true, 12, 6);

        // turn_pid(-45, -1, 1);
        // drive_straight_toward_goal(1200, 1);
        // scoring = 3;
        // wait(800, TIME_MSEC);
        // scoring = 1;
        // hood.set(0);
        // lift.set(0);
        // t1.interrupt();
        // arm.spin(DIR_REV, 100, VEL_PCT);
        // drive_straight(47.5, 75, 130, true, 0, 0);
        // tounge.set(1);

        // turn_pid(-44, -1, 1);
        // drive_straight_toward_goal(600, 0);
        // hood.set(1);
        // arm.spin(DIR_FWD, 100, VEL_PCT);
        // drive_full.spin(DIR_REV, 50, VEL_PCT);

        // break;
    }



    case RightSimple: {
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


    case RightComplex: {
        vex::thread t1(intake);
        drive_straight(20, 50, 70);
        drive_turn(50, 9.5, 30, 75, false, 0, 0);
        tounge.set(1);
        drive_turn(-95, -11, 30, 75, false, 0, 0);
        tounge.set(0);
        drive_straight(11, 50, 100, true, 0, 0);
        scoring = 2;
        wait(900, TIME_MSEC);
        scoring = 1;
        drive_straight(-49.5, 75, 100, true, 0, 0);
        tounge.set(1);

        // long goal
        turn_pid(-135, -1, 1);
        drive_straight(7, 35, 40, true, 0, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(650, TIME_MSEC);
        // long goal

        tounge.set(0);
        drive_straight_toward_goal(1100, 0);
        scoring = 4;
        drive_full.spinFor(DIR_REV, 700, TIME_MSEC, 50, VEL_PCT);
        scoring = 1;
        drive_turn(-90, -13, 50, 75, false, 0, 0);
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-27, 75, 50, true, 0, 0);
        turn_pid(30, -1, 1);

        break;
    }


    case LeftComplex: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(16, 75, 100, true, 0, 0);
        drive_turn(-55, -21, 15, 75, false, 0, 0);
        tounge.set(1);
        turn_pid(-80, -1, 1);
        drive_straight_toward_goal(1000, 1);
        scoring = 3;
        wait(800, TIME_MSEC);
        lift.set(0);
        scoring = 1;
        lift.set(0);
        drive_straight(46, 50, 100, true, 0, 0);
        lift.set(0);
        turn_pid(-45, -1, 1);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1200, TIME_MSEC);
        drive_straight_toward_goal(1000, false);
        scoring = 4;
        drive_full.spinFor(DIR_REV, 700, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        scoring = 1;
        drive_turn(-90, -13, 50, 75, false, 0, 0);
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-27, 75, 50, true, 0, 0);
        turn_pid(30, -1, 1);
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
        finger.set(1); // get finger out of way 
        tounge.set(1); // open tounge
        drive_straight((41.5-17+9.5-0.5-1), 70, 100); // straight
        drive_turn(-90, -13, 40, 75, false); // arc toward goal


        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);

        // long goal
        drive_straight_toward_goal(1300, false); // try lowering
        drive_straight(5,30,75, true, 0, 10);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false, 10, 0);
        drive_straight(88, 75, 100, true, 0, 10);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(1000, TIME_MSEC);

        // first goal
        drive_straight(9, 30, 50, true, 0, 0);
        turn_pid(-90, -1, 1); 
        tounge.set(1);
        drive_straight_toward_goal(700, false);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // second match load
        drive_straight(10, 30, 100, false);
        scoring = 1;
        drive_full.spin(DIR_FWD, 3, VLT_VLT);
        wait(2000, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        // score second goal
        drive_straight_toward_goal(750, false); // try lowering 
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
        scoring = 1;
        wait(400, TIME_MSEC);
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2000, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        drive_straight(5, 10, 50, false);
        scoring = 0;
        drive_straight(-7, 10, 50, false);


        // second half legacy going over
        drive_turn(89, 15, 50, 75, false);
        drive_straight(88, 75, 125, false);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(1000, TIME_MSEC);
        drive_straight(-11, 30, 70, false);
        tounge.set(1);
        scoring = 1;
        turn_pid(-90, -1, 1);
        drive_straight(8, 20, 70, false);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);

        // long goal
        drive_straight_toward_goal(1000, false); // try lowering
        drive_straight(5,30,75, false);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false);
        drive_straight(77, 50, 75, false);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(1000, TIME_MSEC);

        // first goal
        drive_straight(9, 30, 100, false);
        turn_pid(-90, -1, 1); 
        tounge.set(1);
        drive_straight_toward_goal(700, false);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // second match load
        drive_straight(8, 20, 75, false);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(300, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(100, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(500, TIME_MSEC);
        // score second goal
        drive_straight_toward_goal(750, false); // try lowering 
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
        scoring = 1;
        wait(400, TIME_MSEC);
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2000, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        drive_straight(5, 10, 50, false);
        scoring = 0;
        drive_straight(-7, 10, 50, false);
        drive_turn(80, 38, 50, 75, false);
        drive_full.spinFor(DIR_FWD, 1300, TIME_MSEC, 50, VEL_PCT);
        intakeLow.spin(DIR_REV, 100, VEL_PCT);
        drive_full.spinFor(DIR_REV, 200, TIME_MSEC, 50, VEL_PCT);

        break;
    }
    }
}

void intake() {
    arm.spinFor(DIR_REV, 500, TIME_MSEC, 100, VEL_PCT);
    arm.resetPosition();
    arm.stop();
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;

    while (true) {
        switch (scoring) {
        case 0: {
            intakeLow.stop();
            hood.set(0);
            printf("%f\n", arm.position(ROT_DEG));
            if ((get_pot_value() > 12)) {
                arm.pid_step(9);
                intakeLow.spin(DIR_REV, 100, VEL_PCT);
            }

            break;
        }
        case 1: {
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            hood.set(0);
            printf("%f\n", arm.position(ROT_DEG));
            if ((get_pot_value() > 12)) {
                arm.pid_step(9);
                intakeLow.spin(DIR_REV, 100, VEL_PCT);
            }
            break;
        }
        case 2: {
            intakeLow.spin(DIR_REV, 30, VEL_PCT);
            hood.set(1);
            break;
        }
        case 3: {
            lift.set(1);
            hood.set(1);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            arm.spin(DIR_FWD, 35, VEL_PCT);
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