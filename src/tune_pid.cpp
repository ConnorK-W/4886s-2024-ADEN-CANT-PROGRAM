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
    master.rumble("-"); // Changed rumble to single dash for "Fast"
    const float TUNER = 0.025;

    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_TOWARD_GOAL_KP, DRIVE_STRAIGHT_TOWARD_GOAL_KI, DRIVE_STRAIGHT_TOWARD_GOAL_KD);

    // --- AGGRESSIVE CONFIGURATION ---
    float final_max_rpm = -300.0; // Increase this if your gear ratio allows (e.g. -200 or -600)
    float accel_base = 8.0;       // INCREASED: 4x faster acceleration (was 2.0)
    
    // SMOOTHING VARS
    int last_known_x = 160;   
    int lost_frames = 0;      
    int loop_counter = 0;
    // ---------------------

    while (true) {
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
                    last_known_x = goal_x; 
                    lost_frames = 0;
                } else {
                    // Persistence logic
                    if (lost_frames < 10) {
                        goal_x = last_known_x;
                        lost_frames++;
                    } else {
                        goal_x = 160; 
                    }
                }

                // --- AGGRESSIVE SPEED SCALING ---
                float error = std::abs(160 - goal_x);
                
                // CHANGE 1: Tolerance
                // Old: (error / 100.0).  50px error = 50% speed (Slow)
                // New: (error / 300.0).  50px error = 83% speed (Fast)
                float speed_factor = 1.0 - (error / 300.0); 
                
                // CHANGE 2: Floor
                // Never drop below 60% speed due to alignment. 
                // We trust the PID to correct the heading while moving.
                if (speed_factor < 0.6) speed_factor = 0.6;
                if (speed_factor > 1.0) speed_factor = 1.0;

                float active_speed_limit = 2 * final_max_rpm * speed_factor;

                // --- ACCELERATION LOGIC ---
                // Negative values: -100 < -50
                if (current_vel > active_speed_limit) { 
                    // Accelerate hard
                    current_vel -= accel_base; 
                } else if (current_vel < active_speed_limit) { 
                    // Brake hard (snap back to valid speed)
                    current_vel += (accel_base * 2.0); 
                }

                if (loop_counter % 5 == 0) {
                    printf("Err: %.1f | Vel: %.1f | Limit: %.1f\n", error, current_vel, active_speed_limit);
                }
                loop_counter++;

                // PID Calc
                double dir_adj = 0;
                if (error > 5) {
                    dir_adj = dir.adjust(160, goal_x);
                }

                // Apply
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
