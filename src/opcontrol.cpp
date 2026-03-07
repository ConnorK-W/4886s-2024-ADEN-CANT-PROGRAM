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
    bool intaking = 1;
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
    finger.set(0);
    lift.set(0);
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
        if (BTN_L2.pressing()){
        hood.set(0 );
    }

        B_SCRN.printAt(100, 200, "Arm position: %f", lever_pot.value(ROT_DEG));
        B_SCRN.printAt(100, 220, "Arm current: %f", arm.current(PCT_PCT));

        
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
        intakeHigh.spin(DIR_FWD, 3, VLT_VLT);
    }
    if (BTN_R2.pressing()){
        intakeFull.spin(DIR_REV, 100, VEL_PCT);
    }
    if (BTN_L1.pressing()){
        intakeFull.spin(DIR_FWD, 100, VEL_PCT);       
        hood.set(0);
    }
    if (BTN_X.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 50, VEL_PCT);
    }
    if (BTN_A.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 100, VEL_PCT);
        lift.set(1);
    }
    else if (BTN_B.pressing()){
        intakeLow.spin(DIR_FWD, 100, VEL_PCT);
        intakeHigh.spin(DIR_FWD, 15, VEL_PCT);
        lift.set(1);
    }
    else{
        lift.set(0);
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