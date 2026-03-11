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
        drive_straight(28, 70, 100, true, 0, 40);
        finger.set(1);
        drive_turn(80, 15, 55, 75, false, 40, 0);
        target_heading = 90;   
        drive_full.spin(DIR_FWD, 3, VLT_VLT);
        wait(750, TIME_MSEC);
        hood.set(1);
        // long goal
        vex::thread t4([](){
            wait(300, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(1000, 0);
        tounge.set(0);
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        t4.interrupt();
        wait(400, TIME_MSEC);
        
        // turning to middle goal
        drive_straight(5, 40, 120, true, 0, 15);
        scoring = 6;
        intake2.stop();
        drive_turn(135, 9.5, 30, 70, false, 15, 10);
        drive_straight(40,35,70, true, 10, 20);
        scoring = 2;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(550, TIME_MSEC);
        
        drive_straight(-3.5, 30, 50, true, 20, 10);
        scoring = 1;
        drive_turn(-55, 16, 60, 70, true, 10, 0);
        t1.interrupt();
        drive_straight(36.5, 75, 100, true, 0, 20);
        tounge.set(1);
        drive_turn(-80, -29, 60, 100, false, 20, 2);
        drive_straight(5, 30, 70, true, 2, 10);

        drive_full.spin(DIR_FWD, 3, VLT_VLT);
        wait(900, TIME_MSEC);
        // long goal
        tounge.set(0);
        drive_straight_toward_goal(800, 0);
        hood.set(0);
        intakeFull.spin(DIR_FWD, 100, VEL_PCT);
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
        drive_straight(15, 75, 100, true, 0, 15);
        drive_turn(150, 15, 30, 75, false, 15, 5);
        tounge.set(1);
        drive_straight(20, 75, 100, true, 5, 15);
        drive_turn(30, 20, 30, 75, false, 15, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(850, TIME_MSEC);
        // long goal

        tounge.set(0);
        vex::thread t4([](){
            wait(600, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(2200, 0);
        drive_turn(-75, -13, 50, 75, false, 0, 0);
        t1.interrupt();
        turn_pid(75, -1, 1);
        drive_straight(-27, 75, 50, true, 0, 2);
        turn_pid(30, -1, 1);
        drive_l.stop(vex::brakeType::hold);
        drive_r.stop(vex::brakeType::hold);
        drive_turn(30, 5, 10, 75, true, 0, 0);


        break;
    }


    case RightComplex: {
        vex::thread t1(intake);
        drive_turn(70, 50, 40, 75, false, 0, 0);
        tounge.set(1);
        wait(500, TIME_MSEC);
        drive_turn(-25, 35, 40, 75, true, 0, 0);
        tounge.set(0);
        turn_pid(-90, -1, 1);
        drive_straight(6, 75, 50, true, 0, 20);
        scoring = 2;
        wait(1200, TIME_MSEC);
        scoring = 1;
        drive_straight(-49, 75, 100, true, 0, 5);
        tounge.set(1);

        // long goal
        turn_pid(-135, -1, 1);
        drive_straight(5, 20, 120, true, 0, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(850, TIME_MSEC);
        // long goal

        tounge.set(0);
        vex::thread t4([](){
            wait(600, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(2200, 0);
        drive_turn(-75, -13, 50, 75, false, 0, 0);
        t1.interrupt();
        turn_pid(75, -1, 1);
        drive_straight(-27, 75, 50, true, 0, 2);
        turn_pid(30, -1, 1);
        drive_l.stop(vex::brakeType::hold);
        drive_r.stop(vex::brakeType::hold);
        drive_turn(30, 5, 10, 75, true, 0, 0);


        break;
    }

    case LeftSimple: {
        turn_pid(30, -1, 1);
        target_heading = 0;   
        drive_straight_toward_goal(1000, 0, false);

        break;
    }

    case LeftComplex: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(9, 75, 100, true, 0, 30);
        vex::thread t2([](){
            wait(500, TIME_MSEC);
            tounge.set(1);
        });
        drive_turn(-55, -17, 15, 75, false, 30, 0);
        t2.interrupt();
        turn_pid(-80, -1, 1);
        drive_straight(-12, 35, 40, true, 0, 10);
        scoring = 3;
        wait(700, TIME_MSEC);
        scoring = 1;
        lift.set(0);
        drive_straight(36, 75, 100, true, 0, 30);
        // tounge.set(1);

        drive_turn(-45, -25, 40, 75, false, 30, 0);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(700, TIME_MSEC);
        // long goal
        tounge.set(0);
        vex::thread t3([](){
            wait(500, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(1500, 0, false);

        drive_turn(-90, -15, 30, 50, false, 0, 0);
        scoring = 1;
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-30, 75, 100, true, 0, 0);
        drive_l.stop(vex::brakeType::hold);
        drive_r.stop(vex::brakeType::hold);
        drive_turn(30, 5, 10, 75, true, 0, 0);
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
        drive_straight(29, 70, 100, true, 0, 10);
        drive_turn(-90, -15, 40, 75, false, 10, 10);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);

        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1300, false, false); // try lowering
        drive_straight(5,30,75, true, 0, 10);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false, 10, 0);
        drive_straight(78, 70, 70, true, 0, 2);
        turn_pid(-30, -1, 1);
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        // long goal
        vex::thread t2([](){
            wait(1200, TIME_MSEC);
            scoring = 4;
            target_heading = 90;   

        });
        drive_straight_toward_goal(3900, false, false);
        // second match load
        t2.interrupt();
        drive_straight(20, 40, 100, false, 0, 15);
        scoring = 1;
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);
        // score second goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight(-25, 70, 100, true, 0, 10);
        scoring = 4;
        drive_straight_toward_goal(2100, false, false);
        drive_straight(5, 10, 50, false);
        scoring = 1;
        drive_straight(-7, 10, 50, false);


        // second half legacy going over
        drive_turn(89, 15, 50, 70, false);
        drive_straight(65, 60, 70, true, 0, 15);
        drive_turn(-90, -19, 40, 75, false, 10, 10);
        scoring = 1;
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);

        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1200, false, true); // try lowering
        drive_straight(5,30,75, true, 0, 5);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false);
        drive_straight(78, 70, 70, true, 0, 2);
        turn_pid(-30, -1, 1);
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        // long goal
        vex::thread t4([](){
            wait(1200, TIME_MSEC);
            scoring = 4;
            target_heading = 2700;   

        });
        drive_straight_toward_goal(3900, false, false);
        t4.interrupt();
        drive_straight(20, 40, 75, true, 0, 20);
        scoring = 1;
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);
        // score second goal

        // drive_straight(-35, 50, 70); // straight
        drive_straight(-25, 70, 100, true, 0, 10);
        scoring = 4;
        drive_straight_toward_goal(2100, false, false);
        tounge.set(0);
        scoring = 1;
        drive_turn(80, 38, 50, 75, false);
        vex::thread t6([](){
            wait(800, TIME_MSEC);
            tounge.set(1);
        });
        drive_full.spinFor(DIR_FWD, 950, TIME_MSEC, 50, VEL_PCT);

        break;
    }

    }
}

void intake() {
    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;

    while (true) {
        switch (scoring) {
        case 0: {
            intakeFull.stop();
            hood.set(1);
            break;
        }
        case 1: {
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            intakeHigh.spin(DIR_FWD, 1, VLT_VLT);
            hood.set(1);
            lift.set(0);
            break;
        }
        case 2: {
            lift.set(1);
            intakeLow.spin(DIR_REV, 30, VEL_PCT);
            hood.set(1);
            break;
        }
        case 3: {
            lift.set(1);
            hood.set(1);
            intakeHigh.spin(DIR_REV, 60, VEL_PCT);
            intake1.spin(DIR_FWD, 100, VEL_PCT);
            intake2.spin(DIR_FWD, 50, VEL_PCT);
            break;
        }
        case 4: {
            lift.set(0);
            hood.set(0);
            intakeFull.spin(DIR_FWD, 100, VEL_PCT);
            break;
        }
        case 5: {
            lift.set(0);
            hood.set(0);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            intakeHigh.spin(DIR_FWD, 60, VEL_PCT);
            break;
        }
        case 6: {
            intake1.spin(DIR_FWD, 100, VEL_PCT);
            hood.set(1);
            lift.set(0);
            break;        }
        case 7: {
            lift.set(0);
            hood.set(0);
            wait(200, TIME_MSEC);
            intakeLow.spin(DIR_FWD, 100, VEL_PCT);
            intakeHigh.spin(DIR_FWD, 60, VEL_PCT);
            break;
            break;
        }
        case 8: {
            intake1.spin(DIR_REV, 100, VEL_PCT);
            break;
        }
        }
        wait(50, TIME_MSEC);
    }
}