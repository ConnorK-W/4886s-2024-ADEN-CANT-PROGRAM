#include "main.h"
#include "stddefs.h"
#include <thread>

// Tunes side to side for drive straight
void tune_dir_pid() {
    imu.calibrate();
    while (imu.isCalibrating())
        vex::wait(20, vex::msec);
    master.rumble(".");
    const float TUNER = 0.025;
    // Make PID objects
    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    while (true) {
        // Enable opcontrol
        opdrive(TSA, 1, SENSITIVITY);
        // Toggle pid movement on y press
        if (BTN_Y.PRESSED) {
            target_heading = imu_rotation();
            // Go back to opcontrol if y pressed again
            while (!BTN_Y.PRESSED) {
                // Drive forward  300 rpm
                drive_r.spin(DIR_FWD, 300 + rd.adjust(300, drive_r.velocity(VEL_RPM)) - dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                drive_l.spin(DIR_FWD, 300 + ld.adjust(300, drive_l.velocity(VEL_RPM)) + dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                wait(20, vex::msec);
            }
        }
        // Enable pid tuning
        dir.tune_kP(btn_up() - btn_down(), TUNER);
        dir.tune_kI(btn_x() - btn_b(), TUNER);
        dir.tune_kD(btn_right() - btn_left(), TUNER);

        wait(20, vex::msec);
    }
}

// Tunes side to side for drive straight towards goal
void tune_goal_pid() {
    lift.set(1);
    master.rumble(".");
    const float TUNER = 0.025;

    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_TOWARD_GOAL_KP, DRIVE_STRAIGHT_TOWARD_GOAL_KI, DRIVE_STRAIGHT_TOWARD_GOAL_KD);

    // --- CONFIGURATION ---
    float final_max_rpm = -100.0;
    float accel_base = 2.0; 
    
    // SMOOTHING VARS
    int last_known_x = 160;   // Remember where the goal was
    int lost_frames = 0;      // specific counter for safety
    // ---------------------

    while (true) {
        // Background processing
        aivis.takeSnapshot(yellow);
        
        opdrive(TSA, 1, SENSITIVITY);

        if (BTN_Y.PRESSED) {
            float current_vel = 0; 
            
            while (!BTN_Y.PRESSED) {
                aivis.takeSnapshot(yellow);

                int goal_x = 160;
                bool has_target = aivis.largestObject.exists;

                if (has_target) {
                    goal_x = aivis.largestObject.centerX;
                    last_known_x = goal_x; // Update memory
                    lost_frames = 0;
                } else {
                    // FIX 1: PERSISTENCE
                    // If we blink for less than 10 frames (0.2s), keep aiming at the last spot
                    if (lost_frames < 10) {
                        goal_x = last_known_x;
                        lost_frames++;
                    } else {
                        // If lost for too long, default to center (or stop turning)
                        goal_x = 160; 
                    }
                }

                // FIX 2: SMOOTH SPEED SCALING (No jerky thresholds)
                // Calculate error magnitude
                float error = std::abs(160 - goal_x);
                
                // Map error to speed percentage:
                // If error is 0 -> 100% speed
                // If error is 80 (far off) -> 20% speed
                float speed_factor = 1.0 - (error / 100.0); 
                
                // Clamp factor between 0.2 and 1.0
                if (speed_factor < 0.2) speed_factor = 0.2;
                if (speed_factor > 1.0) speed_factor = 1.0;

                // Calculate dynamic speed limit
                float active_speed_limit = final_max_rpm * speed_factor;

                // --- ACCELERATION LOGIC ---
                // Smoothly ramp current_vel towards active_speed_limit
                // (Using negative logic: -100 is "less than" -20)
                
                if (current_vel > active_speed_limit) { 
                    // Accelerating (going more negative)
                    current_vel -= accel_base; 
                } else if (current_vel < active_speed_limit) { 
                    // Decelerating (we are going too fast for our current turn angle)
                    // We use a gentle brake (1.5x accel) rather than a hard slam
                    current_vel += (accel_base * 1.5); 
                }

                // FIX 3: DEADBAND
                // If error is tiny (within 5 pixels), don't turn at all to prevent wiggles
                double dir_adj = 0;
                if (error > 5) {
                    dir_adj = dir.adjust(160, goal_x);
                }

                drive_r.spin(DIR_FWD, current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) - dir_adj, VEL_RPM);
                drive_l.spin(DIR_FWD, current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) + dir_adj, VEL_RPM);

                wait(20, vex::msec);
            }
            
            // drive_l.stop(vex::brakeType::brake);
            // drive_r.stop(vex::brakeType::brake);
            wait(200, vex::msec);
        }

        dir.tune_kP(btn_up() - btn_down(), TUNER);
        dir.tune_kI(btn_x() - btn_b(), TUNER);
        dir.tune_kD(btn_right() - btn_left(), TUNER);

        wait(20, vex::msec);
    }
}

// Tunes acceleration for drive straight
void tune_accel_pid() {
    imu.calibrate();
    while (imu.isCalibrating())
        vex::wait(20, vex::msec);
    master.rumble(".");
    const float TUNER = 0.025;
    // Make PID objects
    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    while (true) {
        // Enable opcontrol
        opdrive(TSA, 1, SENSITIVITY);
        // Toggle pid movement on y press
        if (BTN_Y.PRESSED) {
            target_heading = imu_rotation();
            // Go back to opcontrol if y pressed again
            while (!BTN_Y.PRESSED) {
                // Drive forward  300 rpm
                drive_r.spin(DIR_FWD, 300 + rd.adjust(300, drive_r.velocity(VEL_RPM)) - dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                drive_l.spin(DIR_FWD, 300 + ld.adjust(300, drive_l.velocity(VEL_RPM)) + dir.adjust(target_heading, imu_rotation()), VEL_RPM);
                wait(20, vex::msec);
            }
        }
        // Enable pid tuning
        ld.tune_kP(btn_up() - btn_down(), TUNER);
        ld.tune_kI(btn_x() - btn_b(), TUNER);
        ld.tune_kD(btn_right() - btn_left(), TUNER);
        rd.tune_kP(btn_up() - btn_down(), TUNER);
        rd.tune_kI(btn_x() - btn_b(), TUNER);
        rd.tune_kD(btn_right() - btn_left(), TUNER);

        wait(20, vex::msec);
    }
}

/*
R1 and R2 are for tuning turn in place
L1 and L2 are for tuning arc function

Need to change the PID drive_pid in movement.cpp to move_kp...
When done we need to swich it back to TURN_PID_KP...
*/
void tune_fast_pid() {
    move_kp = TURN_PID_KP;
    move_ki = TURN_PID_KI;
    move_kd = TURN_PID_KD;
    const float TUNER = 0.025;

    while (true) {
        // drive_r.setStopping(vex::coast);
        // drive_l.setStopping(vex::coast);
        opdrive(TSA, 1, SENSITIVITY);
        if (BTN_R1.PRESSED) {
            target_heading = imu_rotation();
            vex::thread t(graph_pid);
            turn_pid(90, -1, 1);
            t.interrupt();
        }
        if (BTN_R2.PRESSED) {
            target_heading = imu_rotation();
            vex::thread t(graph_pid);
            turn_pid(-90, -1, 1);
            t.interrupt();
        }
        if (BTN_L1.PRESSED) {
            //target_heading = imu_rotation();
            vex::thread t(graph_pid);
            drive_turn(-90, WHEEL_TO_WHEEL_DIST * 2, 60, 60, true);
            // drive_straight(36, 66, 512);
            t.interrupt();
        }
        if (BTN_L2.PRESSED) {
            //target_heading = imu_rotation();
            vex::thread t(graph_pid);
            drive_turn(90, WHEEL_TO_WHEEL_DIST * 2, 60, 60, true);
            // drive_straight(108, 72, 72);
            t.interrupt();
        }
        move_kp += (btn_up() - btn_down()) * TUNER;
        move_kd += (btn_x() - btn_b()) * TUNER;

        printf("\nkP: %f\nkI: %f\nkD: %f\n", move_kp, move_ki, move_kd);

        B_SCRN.clearScreen();
        B_SCRN.printAt(0, 20, "kP: %.3f", move_kp);
        B_SCRN.printAt(0, 40, "kI: %.3f", move_ki);
        B_SCRN.printAt(0, 60, "kD: %.3f", move_kd);
        B_SCRN.printAt(0, 100, "imu: %.3f", imu_rotation());

        wait(20, vex::msec);
    }
}
