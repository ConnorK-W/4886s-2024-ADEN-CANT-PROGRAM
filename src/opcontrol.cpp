#include "../include/main.h"
#include "stddefs.h"
#include "vex_thread.h"

// Driver macros
void opcontrol(void) {
    
    drive_l.stop(vex::brakeType::coast);
    drive_r.stop(vex::brakeType::coast);
    colorSort.setLightPower(100, PCT_PCT);
    colorSort.setLight(vex::ledState::on);
    arm.stop(vex::brakeType::coast);

    intakeLow.spin(DIR_REV, 100, VEL_PCT);
    arm.spinFor(DIR_REV, 500, TIME_MSEC, 100, VEL_PCT);
    arm.resetPosition();
    intakeLow.stop();

    bool shifted = false;
    void red_sort(void);
    void blue_sort(void);


    float spd_mod = 1.0;
    float sens_mod = 1.0;
    bool do_neutral_line_up = false;

    const int LIFT_BUFFER = 110;
    
    tounge.set(0);
    int liftHeight = 1;
    bool liftOT = 0;
    bool liftSA = 0;
    finger.set(1);
    lift.set(1);
    tounge.set(0);
    bool sort = 1;




    while (1) {
        // master.rumble(".");
        // Drive control
        opdrive(TSA, spd_mod, SENSITIVITY * sens_mod);

        Brain.Screen.drawImageFromFile("Graduation.png", 0, 0);


        // tounge
        if (BTN_Y.PRESSED) {
            tounge.set(!tounge.value());
        }
        if (BTN_L2.PRESSED) {
            finger.set(!finger.value());
        }
        if (BTN_A.PRESSED) {
        lift.set(!lift.value());
        }
        if (BTN_RIGHT.PRESSED) {
            while (arm.current(PCT_PCT) < 95) {
                arm.spin(DIR_REV, 85, PCT_PCT);
                wait(100, TIME_MSEC);
            }
            arm.resetPosition();

        }
        
        // Toggles chase neutral post
        //if (BTN_RIGHT.PRESSED)
        //do_neutral_line_up = !do_neutral_line_up;

        if (do_neutral_line_up) {
            // neutral_line_up();
        }
        liftSA = 0;
        wait(20, vex::msec);

    // Intake

    if (BTN_R1.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    }
    if (BTN_R2.pressing()){
        intakeLow.spin(DIR_REV, 100, VEL_PCT);
    }
    if (BTN_L1.pressing()){
        arm.spinToPosition(140 * 3, ROT_DEG, 100, VEL_PCT, false);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    }
    if (BTN_X.pressing()){
        arm.spinToPosition(140 * 3, ROT_DEG, 35, VEL_PCT, false);
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
    }
    if (!BTN_L1.pressing() && !BTN_X.pressing() && arm.position(ROT_DEG) > 40){
        arm.spinToPosition(8 * 3, ROT_DEG, 100, VEL_PCT, false);
        intakeLow.spin(DIR_REV, 100, VEL_PCT);
    }
    if (!BTN_R1.pressing() && !BTN_R2.pressing() && !BTN_L1.pressing() && !BTN_X.pressing()){
        intakeLow.stop();
    }


    }
}



void opdrive(int control_mode, float drive_mod, float turn_mod) {
    switch (control_mode) {
    case TNK:
        drive_r.spin(DIR_FWD, RIGHT_STICK_Y, VEL_PCT);
        drive_l.spin(DIR_FWD, LEFT_STICK_Y, VEL_PCT);
        break;
    case OSA:
        drive_r.spin(DIR_FWD, (LEFT_STICK_Y - LEFT_STICK_X * turn_mod) * drive_mod, VEL_PCT);
        drive_l.spin(DIR_FWD, (LEFT_STICK_Y + LEFT_STICK_X * turn_mod) * drive_mod, VEL_PCT);
        break;
    case TSA:
        float lspeed = LEFT_STICK_Y;
        float rspeed = (RIGHT_STICK_X * turn_mod);
        drive_r.spin(DIR_FWD, (lspeed - rspeed) * drive_mod / 8, VLT_VLT);
        drive_l.spin(DIR_FWD, (lspeed + rspeed) * drive_mod / 8, VLT_VLT);
        break;
    }
}

// loat rspeed = (RIGHT_STICK_X * ((LEFT_STICK_Y) + 50) / 200);

/**
 * turnGoal is the desired rotation in degrees
 */
/*
void imuTurn(int turnGoal){
    printf(".\n");
    while (imu.rotation() <= turnGoal) {
        drive_l.spin(DIR_FWD, -20, VEL_PCT);
        drive_r.spin(DIR_FWD, 20, VEL_PCT);
    }
    drive_full.stop();
}
*/
