#include "../include/main.h"
#include "stddefs.h"
#include "vex_thread.h"

// Driver macros
void opcontrol(void) {
    
    drive_l.stop(vex::brakeType::coast);
    drive_r.stop(vex::brakeType::coast);
    colorSort.setLightPower(100, PCT_PCT);
    colorSort.setLight(vex::ledState::on);
    Brain.Screen.setFont(vex::fontType::mono30);

    bool shifted = false;
    bool scoring = 0;
    void red_sort(void);
    void blue_sort(void);


    float spd_mod = 1.0;
    float sens_mod = 1.0;
    bool do_neutral_line_up = false;

    const int LIFT_BUFFER = 110;
    
    tounge.set(0);
    finger.set(0);
    lift.set(0);
    tounge.set(0);
    bool sort = 1;
    Brain.Screen.drawImageFromFile("Xavier.png", 0, 0);

    while (1) {
        // master.rumble(".");
        // Drive control
        opdrive(TSA, spd_mod, SENSITIVITY * sens_mod);


        // tounge
        if (BTN_Y.PRESSED) {
            tounge.set(!tounge.value());
        }
        if (BTN_L2.pressing()){
        hood.set(0 );
        }
        if (BTN_LEFT.PRESSED){
            Brain.Screen.drawImageFromFile("Xavier.png", 0, 0);

        }
        
        // Toggles chase neutral post
        //if (BTN_RIGHT.PRESSED)
        //do_neutral_line_up = !do_neutral_line_up;

        if (do_neutral_line_up) {
            // neutral_line_up();
        }
        wait(20, vex::msec);

    // Intake
    if (BTN_R1.pressing()){
        intake1.spin(DIR_FWD, 100, VEL_PCT);
        intake2.spin(DIR_FWD, 30, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 1, VLT_VLT);
    }
    else if (BTN_R2.pressing()){
        intakeFull.spin(DIR_REV, 100, VEL_PCT);
    }
    else if (BTN_L1.pressing()){
        intakeFull.spin(DIR_FWD, 100, VEL_PCT);       
        hood.set(0);
    }
    else if (BTN_B.pressing()){
        intake1.spin(DIR_REV, 15, VEL_PCT);
        intake2.spin(DIR_REV, 100, VEL_PCT);
        intakeHigh.spin(DIR_REV, 30, VEL_PCT);
        lift.set(1);
    }
    else if (BTN_A.pressing()){
        intake1.spin(DIR_FWD, 100, VEL_PCT);
        intake2.spin(DIR_FWD, 50, VEL_PCT);
        intakeHigh.spin(DIR_REV, 100, VEL_PCT);
    }
    else if (BTN_X.pressing()){
        intake1.spin(DIR_FWD, 100, VEL_PCT);
        intake2.spin(DIR_FWD, 30, VEL_PCT);
        intakeHigh.spin(DIR_REV, 30, VEL_PCT);
    }
    else {
        lift.set(0);
        scoring = 0;
    }
    if (!BTN_L1.pressing() && !BTN_L2.pressing() && !BTN_X.pressing() && !BTN_B.pressing() && !BTN_A.pressing()){
        hood.set(1);
    }
    if (!BTN_R1.pressing() && !BTN_R2.pressing() && !BTN_L1.pressing() && !BTN_B.pressing() && !BTN_X.pressing() && !BTN_A.pressing()){
        intakeFull.stop();
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