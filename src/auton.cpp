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
        drive_straight(33, 70, 100, true, 0, 20);
        finger.set(1);
        drive_turn(90, 14, 40, 75, false, 20, 0);

        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(750, TIME_MSEC);
        hood.set(1);
        tounge.set(0);
        // long goal
        vex::thread t4([](){
            wait(600, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(1000, 0);
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        t4.interrupt();
        wait(500, TIME_MSEC);

        // middle goal
        vex::thread t5([](){
            wait(500, TIME_MSEC);
            scoring = 1;
        });
        drive_turn(90, 5, 30, 70, false);
        target_heading = 181;   
        drive_straight(52,30,30, true, 0, 10);
        drive_straight(12, 10, 50, true, 10, 5);
        turn_pid(-45, -1, 1);
        vex::thread t2([](){
            wait(500, TIME_MSEC);
            scoring = 3;
        });
        drive_straight_toward_goal(1500, 1);
        wait(500, TIME_MSEC);
        t2.interrupt();
        scoring = 1;
        hood.set(0);
        lift.set(0);
        t1.interrupt();
        arm.spin(DIR_REV, 100, VEL_PCT);
        drive_straight(31.5, 75, 100, true, 0, 30);
        tounge.set(1);
        // tounge.set(1);

        drive_turn(-45, -30, 40, 75, false, 30, 0);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(800, TIME_MSEC);
        // long goal
        tounge.set(0);
        hood.set(1);
        vex::thread t3([](){
            wait(500, TIME_MSEC);
            arm.spin(DIR_FWD, 100, VEL_PCT);
        });
        drive_straight_toward_goal(1500, 0);
        tounge.set(0);
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
        drive_straight(20, 50, 70);
        drive_turn(50, 9.5, 30, 75, false, 0, 0);
        tounge.set(1);
        drive_turn(-95, -11, 30, 75, false, 0, 0);
        tounge.set(0);
        drive_straight(12, 50, 100, true, 0, 20);
        scoring = 2;
        wait(900, TIME_MSEC);
        scoring = 1;
        drive_straight(-49, 75, 100, true, 0, 0);
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
        drive_l.stop(vex::brakeType::hold);
        drive_r.stop(vex::brakeType::hold);
        drive_turn(30, 5, 10, 75, true, 0, 0);
        break;
    }


    case RightComplex: {
        vex::thread t1(intake);
        drive_straight(10, 70, 120, true, 0, 15);
        drive_turn(50, 11, 20, 75, false, 30, 5);
        tounge.set(1);
        drive_turn(-95, -11, 30, 75, false, 5, 5);
        tounge.set(0);
        drive_straight(12, 50, 100, true, 5, 20);
        scoring = 2;
        wait(900, TIME_MSEC);
        scoring = 1;
        drive_straight(-51, 75, 100, true, 0, 5);
        tounge.set(1);

        // long goal
        turn_pid(-135, -1, 1);
        drive_straight(12, 20, 120, true, 0, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(600, TIME_MSEC);
        // long goal

        tounge.set(0);
                tounge.set(0);
        vex::thread t3([](){
            wait(600, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(1500, 0);
        drive_turn(-90, -13, 50, 75, false, 0, 0);
        t1.interrupt();
        turn_pid(90, -1, 1);
        drive_straight(-27, 75, 50, true, 0, 2);
        turn_pid(30, -1, 1);
        drive_l.stop(vex::brakeType::hold);
        drive_r.stop(vex::brakeType::hold);
        drive_turn(30, 5, 10, 75, true, 0, 0);


        break;
    }

    case LeftSimple: {
        vex::thread t1(intake);
        drive_straight(15, 75, 100, true, 0, 15);
        drive_turn(-55, -28, 10, 75, false, 15, 15);
        drive_straight(17, 40, 100, true, 15, 2);   
        tounge.set(1);
        wait(500, TIME_MSEC);
        drive_straight(-18, 75, 100, true, 5, 2);   
        drive_turn(235, -18, 30, 75, true, 2, 5);
        scoring = 4; 
        drive_full.spinFor(DIR_REV, 800, TIME_MSEC, 50, VEL_PCT);
        scoring = 1;
        drive_straight(25, 75, 100, true, 0, 30);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(600, TIME_MSEC);
        lift.set(1);
        drive_turn(45, -13, 70, 75, true, 0, 15);
        drive_straight(-40, 75, 100, true, 15, 30);
        drive_straight_toward_goal(2000, 1);
        scoring = 3;
        wait(1000, TIME_MSEC);
        t1.interrupt();

        break;
    }

    case LeftComplex: {
        vex::thread t1(intake);
        lift.set(1);
        drive_straight(17, 75, 100, true, 0, 30);
        vex::thread t4([](){
            wait(500, TIME_MSEC);
            tounge.set(1);
        });
        drive_turn(-55, -17, 15, 75, false, 30, 0);
        t4.interrupt();
        turn_pid(-80, -1, 1);
        drive_straight(-12, 35, 40, true, 0, 10);
        scoring = 3;
        wait(700, TIME_MSEC);
        scoring = 1;
        lift.set(0);
        drive_straight(30, 75, 100, true, 0, 30);
        // tounge.set(1);

        drive_turn(-45, -30, 40, 75, false, 30, 0);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(800, TIME_MSEC);
        // long goal
        tounge.set(0);
        vex::thread t3([](){
            wait(500, TIME_MSEC);
            scoring = 4;
        });
        drive_straight_toward_goal(1500, 0);

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
        drive_straight(32, 70, 100, true, 0, 20);
        drive_turn(-90, -14, 40, 75, false, 20, 0);


        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(200, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1000, TIME_MSEC);

        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1300, false, true); // try lowering
        drive_straight(5,30,75, true, 0, 10);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false, 10, 0);
        drive_straight(85, 60, 70, true, 0, 5);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 4, VLT_VLT);
        wait(1000, TIME_MSEC);
        drive_full.stop();

        // first goal
        drive_straight(9, 30, 50, true, 2, 2);
        turn_pid(-90, -1, 1); 
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        drive_straight_toward_goal(700, false, true);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // second match load
        drive_straight(10, 30, 100, false);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(2000, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(200, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1000, TIME_MSEC);
        // score second goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(750, false, true); // try lowering 
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
        drive_turn(89, 15, 50, 70, false);
        drive_straight(88, 60, 70, false);
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
        wait(200, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1000, TIME_MSEC);

        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1000, false, true); // try lowering
        drive_straight(5,30,75, false);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false);
        drive_straight(85, 50, 75, false);
        turn_pid(90, -1, 1);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(1000, TIME_MSEC);

        // first goal
        drive_straight(9, 30, 100);
        turn_pid(-90, -1, 1); 
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        drive_straight_toward_goal(700, false, true);
        scoring = 5;
        drive_full.spinFor(DIR_REV, 1500, TIME_MSEC, 50, VEL_PCT);
        // second match load
        drive_straight(9, 20, 75, false);
        scoring = 1;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(200, TIME_MSEC);
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(1000, TIME_MSEC);
        // score second goal

        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(750, false, true); // try lowering 
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2500, TIME_MSEC, 50, VEL_PCT);
        scoring = 1;
        wait(400, TIME_MSEC);
        scoring = 6; 
        drive_full.spinFor(DIR_REV, 2000, TIME_MSEC, 50, VEL_PCT);
        tounge.set(0);
        scoring = 8;
        drive_turn(80, 38, 50, 75, false);
        drive_full.spinFor(DIR_FWD, 1200, TIME_MSEC, 50, VEL_PCT);

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
            arm.spin(DIR_FWD, 32.5, VEL_PCT);

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