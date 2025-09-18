#include "../include/main.h"
#include "stddefs.h"
#include "vex_thread.h"

// Driver macros
void opcontrol(void) {
    
    lift.setStopping(vex::brakeType::coast);
    drive_l.stop(vex::brakeType::coast);
    drive_r.stop(vex::brakeType::coast);
    colorSort.setLightPower(100, PCT_PCT);
    colorSort.setLight(vex::ledState::on);


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
    stopper.set(0);
    bool sort = 1;




    while (1) {
        // master.rumble(".");
        // Drive control
        opdrive(TSA, spd_mod, SENSITIVITY * sens_mod);

        Brain.Screen.drawImageFromFile("Graduation.png", 0, 0);

        if (BTN_L2.pressing()) {
            lift.spinToPosition(265 * 3, ROT_DEG, 100, VEL_PCT, false);
            liftSA = 1;
        }
        if (BTN_L1.PRESSED)
            liftHeight = liftHeight + 1;
        if (BTN_L2.PRESSED)
            liftHeight = 1;
        if (BTN_LEFT.PRESSED)
            sort = !sort;



        // tounge
        if (BTN_Y.PRESSED) {
            tounge.set(!tounge.value());
        }
        
        // stopper
        if (BTN_B.PRESSED)
            stopper.set(!stopper.value());
        
        // Toggles chase neutral post
        //if (BTN_RIGHT.PRESSED)
        //do_neutral_line_up = !do_neutral_line_up;

        if (do_neutral_line_up) {
            // neutral_line_up();
        }
        liftSA = 0;
        wait(20, vex::msec);

    // Intake
    intakeLow.spin(DIR_FWD, (btn_r1() - btn_r2()) * BTN_TO_PCT, VEL_PCT);


    if (BTN_L1.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
    }
    if (BTN_L2.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        stopper.set(1);
    }
    else {
        stopper.set(0);
    }
    if (!BTN_L1.pressing() && !BTN_L2.pressing()){
        intakeHigh.stop();
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
        drive_r.spin(DIR_FWD, (lspeed - rspeed) * drive_mod / 10, VLT_VLT);
        drive_l.spin(DIR_FWD, (lspeed + rspeed) * drive_mod / 10, VLT_VLT);
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
