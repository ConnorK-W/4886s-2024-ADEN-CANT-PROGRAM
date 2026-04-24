#include "../include/main.h"
#include "stddefs.h"
#include "vex_thread.h"

static float slew_drive(float current, float target,
                        float forward_accel_step,
                        float reverse_accel_step,
                        float forward_decel_step,
                        float reverse_decel_step) {
    float delta = target - current;
    float step = std::fabs(delta);

    if (current > 0.0f && target > 0.0f) {
        step = (std::fabs(target) < std::fabs(current)) ? forward_decel_step : std::fabs(delta);
    } else if (current < 0.0f && target < 0.0f) {
        step = (std::fabs(target) < std::fabs(current)) ? reverse_decel_step : std::fabs(delta);
    } else if (current > 0.0f && target < 0.0f) {
        step = forward_decel_step;
    } else if (current < 0.0f && target > 0.0f) {
        step = reverse_decel_step;
    }

    if (delta > step) return current + step;
    if (delta < -step) return current - step;
    return target;
}

static float clamp_voltage(float value) {
    if (value > 12.0f) return 12.0f;
    if (value < -12.0f) return -12.0f;
    return value;
}

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
    int hood_hold_until = 0;
    int ignore_lift_toggle_until = 0;
    constexpr int HOOD_HOLD_MSEC = 500;
    bool skills_mode = false;
    bool prev_auton_combo_pressed = false;
    bool prev_middle_high_pressed = false;
    
    lift.set(0);
    intakeLift.set(0);
    Brain.Screen.drawImageFromFile("Xavier.png", 0, 0);

    while (1) {
        bool auton_combo_pressed = BTN_RIGHT.pressing() && BTN_X.pressing();
        bool middle_high_pressed = BTN_RIGHT.pressing() && BTN_A.pressing();
        bool auton_combo_started = auton_combo_pressed && !prev_auton_combo_pressed;
        bool middle_high_started = middle_high_pressed && !prev_middle_high_pressed;
        prev_auton_combo_pressed = auton_combo_pressed;
        prev_middle_high_pressed = middle_high_pressed;

        if (auton_combo_started) {
            drive_l.stop(vex::brakeType::coast);
            drive_r.stop(vex::brakeType::coast);
            intakeFull.stop();
            arm.stop(vex::brakeType::brake);

            imu.calibrate();
            while (imu.isCalibrating()) {
                wait(20, vex::msec);
            }
            wait(700, vex::msec);
            autonomous();

            drive_l.stop(vex::brakeType::coast);
            drive_r.stop(vex::brakeType::coast);
            intakeFull.stop();
            arm.stop(vex::brakeType::brake);
            continue;
        }

        if (middle_high_started && skills_mode == true) {
            tounge.set(1);
            intakeFull.spin(DIR_REV, 20, VEL_PCT);
            lift.set(1);
            hood.set(0);
            drive_full.spin(DIR_REV, 3, VLT_VLT);
            wait(500, TIME_MSEC);
            drive_full.stop();
            intakeFull.spin(DIR_FWD, 100, VEL_PCT);
            wait(1000, TIME_MSEC);
            arm.pid_step(99, 15);
            wait(2000, TIME_MSEC);
            tounge.set(0);
            drive_full.spin(DIR_FWD, 5, VLT_VLT);
            wait(500, TIME_MSEC);
            drive_full.stop();
            lift.set(1);
            continue;
        }


        opdrive(TSA, spd_mod, SENSITIVITY * sens_mod);

        // --- STEP 1: INPUT LOGIC ---
        IntakeState currentState = IntakeState::OFF;

        if (BTN_L1.pressing() && BTN_L2.pressing()) currentState = IntakeState::HALF_SCORE;
        else if (BTN_L1.pressing())                 currentState = IntakeState::SCORE;
        else if (BTN_X.pressing() && !BTN_RIGHT.pressing()) currentState = IntakeState::SCORE_SLOW;
        else if (BTN_R1.pressing())                 currentState = IntakeState::INTAKE;
        else if (BTN_R2.pressing())                 currentState = IntakeState::OUTTAKE;
        else if (BTN_B.pressing())                  currentState = IntakeState::OUTTAKE_LIFT;
        else                                        currentState = IntakeState::OFF;

        bool hood_score_active =
            currentState == IntakeState::SCORE ||
            currentState == IntakeState::SCORE_SLOW ||
            currentState == IntakeState::HALF_SCORE;

        if (hood_score_active) {
            hood_hold_until = totalTime.time(vex::msec) + HOOD_HOLD_MSEC;
        }

        // --- STEP 2: OUTPUT LOGIC ---
        switch (currentState) {
            case IntakeState::INTAKE:
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                intakeLift.set(0);
                arm.pid_step(0);
                hood.set(1);
                break;

            case IntakeState::OUTTAKE:
                intakeFull.spin(DIR_REV, 100, VEL_PCT);
                arm.pid_step(0);
                hood.set(1);
                break;

            case IntakeState::OUTTAKE_LIFT:
                intakeFull.spin(DIR_REV, 50, VEL_PCT);
                arm.pid_step(0);
                intakeLift.set(1);
                hood.set(1);
                break;

            case IntakeState::SCORE:
                hood.set(0);
                arm.pid_step(99, skills_mode ? 25 : 100);
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                break;

            case IntakeState::SCORE_SLOW:
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                arm.pid_step(99, 15);
                hood.set(0);
                break;

            case IntakeState::HALF_SCORE:
                hood.set(0);
                arm.pid_step(50);
                intakeFull.spin(DIR_FWD, 100, VEL_PCT);
                break;

            case IntakeState::OFF:
                intakeFull.stop();
                arm.pid_step(2);
                hood.set(1);
                if (get_pot_value() > 4) {
                    intakeFull.spin(DIR_REV, 45, VEL_PCT);
                }
                break;
        }

        // --- STEP 3: INDEPENDENT SUBSYSTEMS & OVERRIDES ---
        
        // Lift Logic
        if (BTN_A.PRESSED &&
            !BTN_RIGHT.pressing() &&
            totalTime.time(vex::msec) >= ignore_lift_toggle_until) {
            lift.set(!lift.value());
        }

        if (BTN_LEFT.PRESSED && !BTN_X.pressing()) skills_mode = !skills_mode;

        // L2 should move the hood immediately without extending score hold timing.
        if (BTN_L2.pressing()) {
            hood.set(0);
            hood_hold_until = 0;
        } else if (totalTime.time(vex::msec) < hood_hold_until) {
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
        float target_forward = lspeed * drive_mod / 8;
        float turn_voltage = rspeed * drive_mod / 8;

        static float applied_forward = 0.0f;
        constexpr float FORWARD_ACCEL_STEP = 12.0f;
        constexpr float REVERSE_ACCEL_STEP = 12.0f;
        constexpr float FORWARD_DECEL_STEP = 1.00f;
        constexpr float REVERSE_DECEL_STEP = 2.40f;

        applied_forward = slew_drive(
            applied_forward,
            target_forward,
            FORWARD_ACCEL_STEP,
            REVERSE_ACCEL_STEP,
            FORWARD_DECEL_STEP,
            REVERSE_DECEL_STEP
        );

        drive_r.spin(DIR_FWD, clamp_voltage(applied_forward - turn_voltage), VLT_VLT);
        drive_l.spin(DIR_FWD, clamp_voltage(applied_forward + turn_voltage), VLT_VLT);
        break;
    }
}
