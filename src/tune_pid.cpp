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

// Tunes side to side for drive straight towards small goal
void tune_smallgoal_pid() {
    lift.set(1);
    master.rumble("-"); // Changed rumble to single dash for "Fast"
    const float TUNER = 0.025;

    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KP, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KI, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KD);

    // --- AGGRESSIVE CONFIGURATION ---
    float final_max_rpm = -200.0; // 300 for large goal
    float accel_base = 9.0;       // INCREASED: 4x faster acceleration (was 2.0)
    
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
            last_known_x = 160;
            lost_frames = 100;
            int goal_x = 160;
            int factor = 0;
            
            while (!BTN_Y.PRESSED) {
                if (imu.roll() < 6) {
                    aivis.takeSnapshot(yellow);

                    bool has_target = aivis.largestObject.exists;

                    if (has_target) {
                        goal_x = aivis.largestObject.centerX;
                        last_known_x = goal_x; 
                        lost_frames = 0;
                        factor = 1;
                    } else {
                        if (lost_frames < 10) {
                            goal_x = last_known_x;
                            lost_frames++;
                            factor = 1; // KEEP FACTOR 1 while in "lost frames" buffer so it keeps turning briefly
                        } else {
                            goal_x = 160; 
                            factor = 0;
                        }
                    }

                    float error = std::abs(160 - goal_x);

                    printf("error: %f", error);

                    if (factor == 0) {
                        error = 300.0; // Force high error for speed calculation
                    }
                    float speed_factor = 1.0 - (error / 300.0); 
                    
                    if (speed_factor < 0.6) speed_factor = 0.6;
                    if (speed_factor > 1.0) speed_factor = 1.0;

                    float active_speed_limit = 2 * final_max_rpm * speed_factor;

                    if (factor == 0) {
                        active_speed_limit = 0; 
                    }

                    if (current_vel > active_speed_limit) { 
                        current_vel -= accel_base; 
                    } else if (current_vel < active_speed_limit) { 
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
                    drive_r.spin(DIR_FWD, current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) - factor*dir_adj, VEL_RPM);
                    drive_l.spin(DIR_FWD, current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) + factor*dir_adj, VEL_RPM);

                    wait(20, vex::msec);
                } else {
                    drive_r.stop(vex::brakeType::brake);
                    drive_l.stop(vex::brakeType::brake);
                    break;
                }
            }
            
            wait(200, vex::msec);
        }

        dir.tune_kP(btn_up() - btn_down(), TUNER);
        dir.tune_kI(btn_x() - btn_b(), TUNER);
        dir.tune_kD(btn_right() - btn_left(), TUNER);

        wait(20, vex::msec);
    }
}

// Tunes smooth proportional wall following
void tune_wall_follow_pid() {
    master.rumble("...");
    const float INTERVAL_TUNER = 0.25;  // Adjust base interval by 0.25 inches
    const float GAIN_TUNER = 0.5;       // Adjust correction gain by 0.5 deg/inch
    const float MAX_ANGLE_TUNER = 1.0;  // Adjust max angle by 1 degree
    const float THRESHOLD_TUNER = 0.1;  // Adjust threshold by 0.1 inches

    // Proportional controller tuning values
    float base_interval = WALL_BASE_CHECK_INTERVAL;   // Base distance between checks
    float correction_gain = WALL_CORRECTION_GAIN;     // Degrees per inch of error
    float max_angle = WALL_MAX_CORRECTION_ANGLE;      // Maximum correction angle
    float error_threshold = WALL_ERROR_THRESHOLD;     // Ignore errors below this

    // Test parameters
    float test_distance = 24.0;       // Test drive distance (positive = forward)
    float target_wall_dist = 21.0;    // Target distance from wall
    float target_speed = 20;           // Test speed in IPS (slow for testing)
    float accel = 15;                  // Acceleration (lower for smoother)

    B_SCRN.clearScreen();

    // Tune mode: 0=base_interval, 1=correction_gain, 2=max_angle, 3=error_threshold, 4=test_distance
    static int tune_mode = 0;
    const int NUM_MODES = 5;

    while (true) {
        // Run test on Y press
        if (BTN_Y.PRESSED) {
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }

            // Put down the tongue
            tounge.set(1);
            wait(200, vex::msec);

            imu.calibrate();
            while (imu.isCalibrating()) { wait(20, vex::msec); }
            reset_imu_rotation();
            target_heading = imu_rotation();

            // Temporarily override constants for testing
            #undef WALL_BASE_CHECK_INTERVAL
            #undef WALL_CORRECTION_GAIN
            #undef WALL_MAX_CORRECTION_ANGLE
            #undef WALL_ERROR_THRESHOLD
            #define WALL_BASE_CHECK_INTERVAL base_interval
            #define WALL_CORRECTION_GAIN correction_gain
            #define WALL_MAX_CORRECTION_ANGLE max_angle
            #define WALL_ERROR_THRESHOLD error_threshold

            printf("\n=== Smooth Wall Follow Test ===\n");
            printf("BaseInt:%.1f  Gain:%.1f  MaxAng:%.1f  Thr:%.1f  Target:%.1f\n\n",
                   base_interval, correction_gain, max_angle, error_threshold, target_wall_dist);

            // Run the smooth proportional wall follow function
            drive_straight_wall_follow(test_distance, target_speed, accel, target_wall_dist, true, 0, 0);

            printf("\n=== Test Complete ===\n\n");

            // Restore original constants
            #undef WALL_BASE_CHECK_INTERVAL
            #undef WALL_CORRECTION_GAIN
            #undef WALL_MAX_CORRECTION_ANGLE
            #undef WALL_ERROR_THRESHOLD
            #define WALL_BASE_CHECK_INTERVAL 1.5
            #define WALL_CORRECTION_GAIN 3.0
            #define WALL_MAX_CORRECTION_ANGLE 12.0
            #define WALL_ERROR_THRESHOLD 0.3

            wait(200, vex::msec);
        }

        // Change tuning parameter with A button
        if (BTN_A.PRESSED) {
            while (BTN_A.PRESSED) { wait(20, vex::msec); }
            tune_mode = (tune_mode + 1) % NUM_MODES;
        }

        // Adjust selected value
        int adjust = btn_up() - btn_down();
        switch(tune_mode) {
            case 0: // Base interval (minimum check distance)
                base_interval += adjust * INTERVAL_TUNER;
                if (base_interval < 0.5) base_interval = 0.5;
                if (base_interval > 5) base_interval = 5;
                break;
            case 1: // Correction gain (degrees per inch)
                correction_gain += adjust * GAIN_TUNER;
                if (correction_gain < 0.5) correction_gain = 0.5;
                if (correction_gain > 10) correction_gain = 10;
                break;
            case 2: // Max correction angle (degrees)
                max_angle += adjust * MAX_ANGLE_TUNER;
                if (max_angle < 5) max_angle = 5;
                if (max_angle > 25) max_angle = 25;
                break;
            case 3: // Error threshold (inches)
                error_threshold += adjust * THRESHOLD_TUNER;
                if (error_threshold < 0) error_threshold = 0;
                if (error_threshold > 2) error_threshold = 2;
                break;
            case 4: // Test distance
                test_distance += adjust * 6;
                if (test_distance < 6) test_distance = 6;
                if (test_distance > 60) test_distance = 60;
                break;
        }

        // Display current values
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Wall Smooth Tuning");

        B_SCRN.setCursor(2, 1);
        B_SCRN.print("%sBaseInt: %.2f in", (tune_mode == 0) ? "> " : "  ", base_interval);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("%sGain: %.1f d/in", (tune_mode == 1) ? "> " : "  ", correction_gain);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("%sMaxAng: %.0f deg", (tune_mode == 2) ? "> " : "  ", max_angle);

        B_SCRN.setCursor(5, 1);
        B_SCRN.print("%sThresh: %.1f in", (tune_mode == 3) ? "> " : "  ", error_threshold);
        B_SCRN.setCursor(6, 1);
        B_SCRN.print("%sTestDist: %.0f in", (tune_mode == 4) ? "> " : "  ", test_distance);

        B_SCRN.setCursor(7, 1);
        B_SCRN.print("WallTgt:%.1f  Speed:%.0f", target_wall_dist, target_speed);
        B_SCRN.setCursor(8, 1);
        B_SCRN.print("Y:Run A:Next  L:%.1f", distance_left.objectDistance(vex::distanceUnits::in));

        printf("WALL(Base:%.2f Gain:%.1f Max:%.0f Thr:%.1f) Test:%.0f\n",
               base_interval, correction_gain, max_angle, error_threshold, test_distance);

        wait(100, vex::msec);
    }
}

// Tunes side to side for drive straight towards big goal
void tune_biggoal_pid() {
    lift.set(0);
    master.rumble("-"); 
    const float TUNER = 0.025;

    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID dir = PID(DRIVE_STRAIGHT_TOWARD_BIGGOAL_KP, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KI, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KD);

    // --- AGGRESSIVE CONFIGURATION ---
    float final_max_rpm = -450.0; 
    float accel_base = 20.0;       
    
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
            last_known_x = 160;
            lost_frames = 100;
            int goal_x = 160;
            int factor = 0;
            
            while (!BTN_Y.PRESSED) {
                aivis.takeSnapshot(yellow);
                                
                bool has_target = aivis.largestObject.exists;

                if (has_target) {
                    goal_x = aivis.largestObject.centerX;
                    last_known_x = goal_x; 
                    lost_frames = 0;
                    factor = 1;
                } else {
                    if (lost_frames < 10) {
                        goal_x = last_known_x;
                        lost_frames++;
                        factor = 1; 
                    } else {
                        goal_x = 160; 
                        factor = 0;
                    }
                }

                float error = std::abs(160 - goal_x);
                if (factor == 0) {
                    error = 300.0; 
                }
                float speed_factor = 1.0 - (error / 300.0); 
                
                if (speed_factor < 0.6) speed_factor = 0.6;
                if (speed_factor > 1.0) speed_factor = 1.0;

                float active_speed_limit = 2 * final_max_rpm * speed_factor;

                if (factor == 0) {
                    active_speed_limit = 0; 
                }

                if (current_vel > active_speed_limit) { 
                    current_vel -= accel_base; 
                } else if (current_vel < active_speed_limit) { 
                    current_vel += (accel_base * 2.0); 
                }

                // --- UPDATED PRINT LOGIC ---
                if (loop_counter % 5 == 0) {
                    // Get the actual physical speed of the motors
                    double actual_L = drive_l.velocity(VEL_RPM);
                    double actual_R = drive_r.velocity(VEL_RPM);

                    // Print Target vs Actual Left vs Actual Right
                    // "Targ" is what your code requests. "Act" is what the motors are actually doing.
                    printf("Targ: %.0f | Act L: %.0f | Act R: %.0f\n", current_vel, actual_L, actual_R);
                }
                loop_counter++;
                // ---------------------------

                // PID Calc
                double dir_adj = 0;
                if (error > 5) {
                    dir_adj = dir.adjust(160, goal_x);
                }

                // Apply
                drive_r.spin(DIR_FWD, current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) - factor*dir_adj, VEL_RPM);
                drive_l.spin(DIR_FWD, current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) + factor*dir_adj, VEL_RPM);

                wait(20, vex::msec);
            }
            
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

// Tunes arm PID with buttons
void tune_arm_pid() {
    master.rumble(".");
    const float TUNER = 0.05; 

    B_SCRN.clearScreen();

    while (true) {
        // Toggle PID movement on Y press
        if (BTN_Y.PRESSED) {
            // Wait for button release
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }

            // Run PID loop until Y pressed again
            while (!BTN_Y.PRESSED) {
                 arm.rotate_pid(9);
                
                 break; 
            }
            // Wait for release
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }
        }

        // Tune Constants
        // Up/Down -> kP
        // X/B     -> kI
        // Right/Left -> kD
        if (btn_up() || btn_down()) {
            arm_kp += (btn_up() - btn_down()) * TUNER;
            if(arm_kp < 0) arm_kp = 0;
        }
        if (btn_x() || btn_b()) {
            arm_ki += (btn_x() - btn_b()) * TUNER;
             if(arm_ki < 0) arm_ki = 0;
        }
        if (btn_right() || btn_left()) {
            arm_kd += (btn_right() - btn_left()) * TUNER;
             if(arm_kd < 0) arm_kd = 0;
        }

        // Display
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Arm PID Tuning");
        B_SCRN.setCursor(2, 1);
        B_SCRN.print("P: %.3f", arm_kp);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("I: %.3f", arm_ki);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("D: %.3f", arm_kd);
        B_SCRN.setCursor(5, 1);
        B_SCRN.print("Pot: %.2f", get_pot_value());
        
        // Also print to terminal
        printf("P: %.3f I: %.3f D: %.3f Pot: %.2f\n", arm_kp, arm_ki, arm_kd, get_pot_value());

        wait(100, vex::msec); // Slower loop for tuning UI
    }
}

// Tunes drive_straight_toward_goal big goal kP
// Runs the function for 2000ms each time Y is pressed
void tune_drive_toward_goal() {
    master.rumble("--"); // Double dash for drive toward goal tuning
    const float TUNER = 0.025;

    B_SCRN.clearScreen();

    while (true) {
        // Toggle movement on Y press - runs for 2000ms
        if (BTN_Y.PRESSED) {
            // Wait for button release
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }

            // Run drive_straight_toward_goal for 2000ms (big goal mode)
            drive_straight_toward_goal(2000, false, true);
            
            // Stop motors explicitly after run
            drive_r.stop(vex::brakeType::brake);
            drive_l.stop(vex::brakeType::brake);
            
            wait(200, vex::msec);
        }

        // Tune big goal kP only
        // Up/Down -> kP
        if (btn_up() || btn_down()) {
            drive_biggoal_kp += (btn_up() - btn_down()) * TUNER;
            if (drive_biggoal_kp < 0) drive_biggoal_kp = 0;
        }

        // Display
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Big Goal kP Tuning");
        B_SCRN.setCursor(2, 1);
        B_SCRN.print("kP: %.3f", drive_biggoal_kp);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("Press Y to run 2000ms");
        
        // Print to terminal
        printf("BIG GOAL kP: %.3f\n", drive_biggoal_kp);

        wait(100, vex::msec); // Slower loop for tuning UI
    }
}

void tune_dist_sensor_pid() {
    master.rumble("...");
    const float TUNER = 0.1;

    // Distance PID tuning values
    float dist_kp = DRIVE_TO_DIST_KP;
    float dist_ki = DRIVE_TO_DIST_KI;
    float dist_kd = DRIVE_TO_DIST_KD;

    // Test parameters
    float target_dist = 18.2;
    float max_speed = 50;
    float accel = 100;

    B_SCRN.clearScreen();

    // Tune mode: 0=dist_kp, 1=dist_ki, 2=dist_kd, 3=max_speed, 4=accel
    static int tune_mode = 0;
    const int NUM_MODES = 5;

    while (true) {
        // Run test on Y press
        if (BTN_Y.PRESSED) {
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }

            imu.calibrate();
            while (imu.isCalibrating()) { wait(20, vex::msec); }
            reset_imu_rotation();
            target_heading = imu_rotation();

            drive_straight_to_dist_value(target_dist, max_speed, accel, distance_front);

            wait(200, vex::msec);
        }

        // Change tuning parameter with A button
        if (BTN_A.PRESSED) {
            while (BTN_A.PRESSED) { wait(20, vex::msec); }
            tune_mode = (tune_mode + 1) % NUM_MODES;
        }

        // Adjust selected value
        int adjust = btn_up() - btn_down();
        switch(tune_mode) {
            case 0: // Distance kP
                dist_kp += adjust * TUNER;
                if (dist_kp < 0) dist_kp = 0;
                break;
            case 1: // Distance kI
                dist_ki += adjust * TUNER;
                if (dist_ki < 0) dist_ki = 0;
                break;
            case 2: // Distance kD
                dist_kd += adjust * TUNER;
                if (dist_kd < 0) dist_kd = 0;
                break;
            case 3: // Max speed
                max_speed += adjust * 5;
                if (max_speed < 10) max_speed = 10;
                break;
            case 4: // Acceleration
                accel += adjust * 5;
                if (accel < 5) accel = 5;
                break;
        }

        // Display current values
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Dist Sensor PID Tuning");

        // Show distance PID values with selection indicator
        B_SCRN.setCursor(2, 1);
        B_SCRN.print("%sDIST kP: %.3f", (tune_mode == 0) ? "> " : "  ", dist_kp);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("%sDIST kI: %.3f", (tune_mode == 1) ? "> " : "  ", dist_ki);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("%sDIST kD: %.3f", (tune_mode == 2) ? "> " : "  ", dist_kd);

        B_SCRN.setCursor(5, 1);
        B_SCRN.print("%sMax Speed: %.1f", (tune_mode == 3) ? "> " : "  ", max_speed);
        B_SCRN.setCursor(6, 1);
        B_SCRN.print("%sAccel: %.1f", (tune_mode == 4) ? "> " : "  ", accel);

        B_SCRN.setCursor(7, 1);
        B_SCRN.print("Y: Run to %.1f in", target_dist);
        B_SCRN.setCursor(8, 1);
        B_SCRN.print("A: Next Param");
        B_SCRN.setCursor(9, 1);
        B_SCRN.print("Front: %.2f in", distance_front.objectDistance(vex::distanceUnits::in));

        printf("DIST(kP:%.3f kI:%.3f kD:%.3f) Speed:%.1f Accel:%.1f\n",
               dist_kp, dist_ki, dist_kd, max_speed, accel);

        wait(100, vex::msec);
    }
}


