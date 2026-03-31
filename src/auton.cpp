#include "../include/main.h"
#include "stddefs.h"
#include "vex_global.h"
#include "vex_units.h"
int scoring = 1;

void intake(void);

enum class IntakeState { OFF, INTAKE, OUTTAKE, OUTTAKE_LIFT, SCORE, SCORE_SLOW, HALF_SCORE };
IntakeState currentState = IntakeState::INTAKE;

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
        drive_turn(80, 15.5, 55, 75, false, 40, 0);
        target_heading = 90;   
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        hood.set(1);
        tounge.set(0);
        // long goal
        vex::thread t4([](){
            wait(500, TIME_MSEC);
            currentState = IntakeState::SCORE; 
        });
        drive_straight_toward_goal(1000, 0);
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        t4.interrupt();
        wait(650, TIME_MSEC);
        
        // turning to middle goal
        drive_straight(5, 40, 120, true, 0, 15);
        currentState = IntakeState::INTAKE; 
        intake2.stop();
        drive_turn(135, 9.5, 30, 70, false, 15, 10);
        drive_straight(40,35,70, true, 10, 20);
        currentState = IntakeState::OUTTAKE; 
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(700, TIME_MSEC);
        
        drive_straight(-3.5, 30, 50, true, 20, 10);
        currentState = IntakeState::INTAKE; 
        drive_turn(-55, 16, 40, 70, true, 10, 0);
        t1.interrupt();
        drive_straight(44, 75, 100, true, 0, 20);
        tounge.set(1);
        drive_turn(-80, -33.5, 50, 100, false, 20, 10);
        drive_straight(3, 30, 70, true, 10, 10);

        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(850, TIME_MSEC);
        // long goal
        tounge.set(0);
        drive_straight_toward_goal(800, 0);
        hood.set(0);
        intakeFull.spin(DIR_FWD, 100, VEL_PCT);
        drive_full.spin(DIR_REV, 50, VEL_PCT);
        break;
    }


    case RightSimple: {
        vex::thread t1(intake);
        drive_straight(10, 75, 125, true, 0, 40);
        vex::thread t2([](){
            wait(300, TIME_MSEC);
            tounge.set(1);
        });
        drive_turn(100, 13, 60, 100, false, 40, 15);
        tounge.set(1);
        drive_turn(70, 43.5, 60, 75, false, 15, 15);
        t2.interrupt();
        target_heading = 180;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(900, TIME_MSEC);
        // long goal

        tounge.set(0);
        vex::thread t4([](){
            wait(800, TIME_MSEC);
            currentState = IntakeState::SCORE; 
        });
        drive_straight_toward_goal(2400, 0, false);
        drive_straight(5,70,125, true, 0, 15);
        currentState = IntakeState::OUTTAKE; 
        t1.interrupt();
        hood.set(1);
        drive_straight_toward_goal(10000, 0, false);        

        break;
    }


    case RightComplex: {
        vex::thread t1(intake);
        drive_turn(65, 46.7, 40, 75, false, 0, 10);
        tounge.set(1);
        wait(400, TIME_MSEC);
        drive_turn(-18.75, 33, 40, 75, true, 0, 0);
        tounge.set(0);
        turn_pid(-90, -1, 1);
        drive_straight(6, 75, 50, true, 0, 20);
        currentState = IntakeState::OUTTAKE; 
        wait(1200, TIME_MSEC);
        currentState = IntakeState::INTAKE; 
        drive_straight(-49, 75, 100, true, 0, 5);
        tounge.set(1);

        // long goal
        turn_pid(-135, -1, 1);
        drive_straight(3, 20, 120, true, 0, 15);
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(800, TIME_MSEC);
        // long goal

        tounge.set(0);
        vex::thread t4([](){
            wait(600, TIME_MSEC);
            currentState = IntakeState::SCORE; 
        });
        drive_straight_toward_goal(1700, 0);
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
        vex::thread t1(intake);
        drive_straight(10, 75, 100, true, 0, 30);
        vex::thread t2([](){
            wait(300, TIME_MSEC);
            tounge.set(1);
        });
        drive_turn(-100, -13, 60, 100, false, 30, 15);
        tounge.set(1);
        drive_turn(-70, -47, 60, 75, false, 15, 15);
        t2.interrupt();
        target_heading = -180;
        drive_full.spin(DIR_FWD, 4, VLT_VLT);
        wait(900, TIME_MSEC);
        // long goal

        tounge.set(0);
        vex::thread t4([](){
            wait(800, TIME_MSEC);
            currentState = IntakeState::SCORE; 
        });
        drive_straight_toward_goal(2700, 0, false);
        drive_full.spin(DIR_FWD, 6, VLT_VLT);
        wait(300, TIME_MSEC);
        currentState = IntakeState::INTAKE; 
        t1.interrupt();
        hood.set(1);
        drive_straight_toward_goal(1000, 0, false);        

        break;
    }

    case LeftComplex: {
        vex::thread t1(intake);
        tounge.set(0);
        currentState = IntakeState::INTAKE; 
        drive_turn(85, 37, 50, 75, false, 0, 10);
        drive_full.spinFor(DIR_FWD, 2500, TIME_MSEC, 30, VEL_PCT);
        drive_full.spin(DIR_REV, 2, VLT_VLT);
        wait(1000, TIME_MSEC);
        drive_turn(70, 20, 30, 75, false);
        turn_pid(180, -1, 1);
        vex::thread t10([](){
            wait(1200, TIME_MSEC);
            currentState = IntakeState::SCORE; 
            target_heading = 360;

        });
        drive_straight_toward_goal(3900, false, false);
        t10.interrupt();
        tounge.set(1);
        
        break;
    }

    case SKILLS: {
        vex::thread t1(intake);
        tounge.set(1); // open tounge
        drive_straight(31, 70, 100, true, 0, 10);
        drive_turn(-80, -15, 60, 75, false, 10, 20);
        target_heading = -90;
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);

        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1300, false, false); // try lowering
        drive_straight(5,30,75, true, 0, 10);
        tounge.set(0);
        drive_turn(180, 12, 30, 75, false, 10, 0);
        drive_straight(85, 70, 70);
        turn_pid(-30, -1, 1);
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        // long goal
        vex::thread t2([](){
            wait(1200, TIME_MSEC);
            currentState = IntakeState::SCORE; 
            target_heading = 90;   

        });
        drive_straight_toward_goal(3900, false, false);
        // second match load
        t2.interrupt();
        drive_straight(20, 40, 75, false, 0, 20);
        currentState = IntakeState::INTAKE; 
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2500, TIME_MSEC);
        // score second goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight(-28, 70, 100, true, 0, 40);
        currentState = IntakeState::SCORE; 
        drive_straight_toward_goal(2100, false, false);

        // second half legacy going over
        tounge.set(0);
        drive_turn(85, 38.5, 50, 75, false, 0, 10);
        currentState = IntakeState::INTAKE; 
        drive_full.spinFor(DIR_FWD, 2500, TIME_MSEC, 30, VEL_PCT);
        drive_full.spin(DIR_REV, 2, VLT_VLT);
        wait(1500, TIME_MSEC);
        drive_turn(74, 20, 30, 75, false);
        turn_pid(-180, -1, 1);
        tounge.set(1);
        vex::thread t10([](){
            wait(1200, TIME_MSEC);
            currentState = IntakeState::SCORE; 
            target_heading = 90;

        });
        drive_straight_toward_goal(3900, false, false);
        t10.interrupt();
        drive_straight(20, 40, 75, true, 0, 20);
        currentState = IntakeState::INTAKE; 
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);
        // long goal
        // drive_straight(-35, 50, 70); // straight
        drive_straight_toward_goal(1200, false, true); // try lowering
        currentState = IntakeState::SCORE; 
        drive_full.spin(DIR_REV, 5, VLT_VLT);
        wait(2000, TIME_MSEC);
        drive_straight(5,30,75, true, 0, 5);
        tounge.set(0);
        currentState = IntakeState::INTAKE; 
        hood.set(1);
        drive_turn(180, 12, 30, 75, false);
        currentState = IntakeState::INTAKE; 
        hood.set(1);
        drive_straight(85, 70, 70);
        turn_pid(-30, -1, 1);
        tounge.set(1);
        // drive_straight(-15, 50, 70); // straight
        // long goal
        vex::thread t4([](){
            wait(800, TIME_MSEC);
            target_heading = 270;   

        });
        drive_straight_toward_goal(1800, false, false);
        t4.interrupt();
        drive_straight(20, 40, 75, true, 0, 20);
        currentState = IntakeState::INTAKE; 
        drive_full.spin(DIR_FWD, 5, VLT_VLT);
        wait(2000, TIME_MSEC);
        // score second goal

        // drive_straight(-35, 50, 70); // straight
        drive_straight(-28, 70, 100, true, 0, 10);
        currentState = IntakeState::SCORE; 
        drive_straight_toward_goal(2000, false, false);
        tounge.set(0);
        drive_turn(85, 38.5, 50, 75, false);
        currentState = IntakeState::INTAKE; 
        vex::thread t6([](){
            wait(800, TIME_MSEC);
            tounge.set(1);
        });
        drive_full.spinFor(DIR_FWD, 800, TIME_MSEC, 50, VEL_PCT);

        break;
    }

    }
}
void intake() {
    intakeFull.spin(DIR_FWD, 100, VEL_PCT);
    int antiJamTime = 0;
    while (true) {
        switch (currentState) {
            case IntakeState::INTAKE:
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                arm.pid_step(2);
                hood.set(1);

                if (get_pot_value() > 4) {
                    intakeFull.spin(DIR_REV, 30, VEL_PCT);
                }

                if (intake1.velocity(VEL_PCT) < 10)
                    antiJamTime ++;
                if (intake1.velocity(VEL_PCT) > 10)
                    antiJamTime = 0;
                if (antiJamTime == 10){
                    intakeLow.spinFor(DIR_REV, 200, TIME_MSEC, 100, VEL_PCT);
                    intakeLow.spin(DIR_FWD, 100, VEL_PCT);
                    wait(200, TIME_MSEC);
                    antiJamTime = 0;
                }
            wait (20, TIME_MSEC);
                break;

            case IntakeState::OUTTAKE:
                intakeFull.spin(DIR_REV, 100, VEL_PCT);
                arm.pid_step(0);
                hood.set(0);
                break;

            case IntakeState::OUTTAKE_LIFT:
                intakeFull.spin(DIR_REV, 15, VEL_PCT);
                arm.pid_step(0);
                intakeLift.set(1);
                hood.set(1);
                break;

            case IntakeState::SCORE:
                hood.set(0);
                arm.pid_step(99);
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                break;

            case IntakeState::SCORE_SLOW:
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                arm.spin(DIR_FWD, 30, VEL_PCT);
                hood.set(0);
                break;

            case IntakeState::HALF_SCORE:
                hood.set(0);
                arm.pid_step(50);
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                break;

            case IntakeState::OFF:
                intakeFull.stop();
                intakeLift.set(0);
                arm.pid_step(2);
                hood.set(1);
                if (get_pot_value() > 4) {
                    intakeFull.spin(DIR_REV, 30, VEL_PCT);
                }
                break;
        }
    }
}