#include "../include/main.h"
#include "stddefs.h"
#include "vex_thread.h"

void opcontrol(void) {
    drive_l.stop(vex::brakeType::coast);
    drive_r.stop(vex::brakeType::coast);
    arm.stop(vex::brakeType::brake);
    colorSort.setLightPower(100, PCT_PCT);
    colorSort.setLight(vex::ledState::on);
    Brain.Screen.setFont(vex::fontType::mono30);

    enum class IntakeState { OFF, INTAKE, OUTTAKE, OUTTAKE_LIFT, SCORE, SCORE_SLOW, HALF_SCORE };

    float spd_mod = 1.0;
    float sens_mod = 1.0;
    
    lift.set(0);
    intakeLift.set(0);
    Brain.Screen.drawImageFromFile("Xavier.png", 0, 0);

    while (1) {
        opdrive(TSA, spd_mod, SENSITIVITY * sens_mod);

        // --- STEP 1: INPUT LOGIC ---
        IntakeState currentState = IntakeState::OFF;

        if (BTN_L1.pressing() && BTN_L2.pressing()) currentState = IntakeState::HALF_SCORE;
        else if (BTN_L1.pressing())                 currentState = IntakeState::SCORE;
        else if (BTN_X.pressing())                  currentState = IntakeState::SCORE_SLOW;
        else if (BTN_R1.pressing())                 currentState = IntakeState::INTAKE;
        else if (BTN_R2.pressing())                 currentState = IntakeState::OUTTAKE;
        else if (BTN_B.pressing())                  currentState = IntakeState::OUTTAKE_LIFT;
        else                                        currentState = IntakeState::OFF;

        // --- STEP 2: OUTPUT LOGIC ---
        switch (currentState) {
            case IntakeState::INTAKE:
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                arm.pid_step(0);
                hood.set(1);
                break;

            case IntakeState::OUTTAKE:
                intakeFull.spin(DIR_REV, 100, VEL_PCT);
                arm.pid_step(0);
                hood.set(0); // Set to 0 here for R2
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

        // --- STEP 3: INDEPENDENT SUBSYSTEMS & OVERRIDES ---
        
        // Lift Logic
        if (BTN_A.pressing()) {
            lift.set(1);
            hood.set(0); // Dropping hood while lifting
        } else {
            lift.set(0);
        }

        // HARD OVERRIDE: R2 always sets hood to 0
        if (BTN_R2.pressing()) {
            hood.set(0);
        }

        // Tounge Toggle
        if (BTN_Y.PRESSED) tounge.set(!tounge.value());

        wait(20, vex::msec);
    }
}

// Drive function stays the same
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