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

// Tunes wall following PID
void tune_wall_follow_pid() {
    master.rumble("...");
    const float TUNER = 0.025;

    // Wall follow PID tuning values
    float wall_kp = WALL_FOLLOW_KP;
    float wall_ki = WALL_FOLLOW_KI;
    float wall_kd = WALL_FOLLOW_KD;

    // Test parameters
    float drive_dist = 24.0;      // Drive 24 inches
    float target_wall_dist = 24.0; // Target 12 inches from wall
    float target_speed = 50;
    float accel = 20;

    B_SCRN.clearScreen();

    // Tune mode: 0=wall_kp, 1=wall_ki, 2=wall_kd, 3=drive_dist, 4=target_wall_dist
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

            // Temporarily override PID constants
            #undef WALL_FOLLOW_KP
            #undef WALL_FOLLOW_KI
            #undef WALL_FOLLOW_KD
            #define WALL_FOLLOW_KP wall_kp
            #define WALL_FOLLOW_KI wall_ki
            #define WALL_FOLLOW_KD wall_kd

            // Run test for 6 seconds max to prevent motor burnout
            // Use a large distance but limit by time
            vex::timer test_timer;
            test_timer.clear();

            // Run wall follow with very large distance (will be limited by time)
            const int TICKS_PER_SEC = 50;
            const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

            PID pid_drive_l = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
            PID pid_drive_r = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
            PID pid_wall = PID(wall_kp, wall_ki, wall_kd);

            float ips = 0, pos = 0;
            float pos_start_l = pos_drive_l(), pos_start_r = pos_drive_r();
            float pos_l, pos_r;
            float dir_mod = 1;
            float vel_rpm;

            // Run for 6 seconds
            while (test_timer.time(vex::msec) < 6000) {
                // Accelerate
                if (ips < target_speed)
                    ips += accel / TICKS_PER_SEC;
                else
                    ips = target_speed;

                pos += ips / TICKS_PER_SEC * dir_mod;

                pos_l = pos_drive_l() - pos_start_l;
                pos_r = pos_drive_r() - pos_start_r;

                float pid_adjustment_l = pid_drive_l.adjust(pos, pos_l);
                float pid_adjustment_r = pid_drive_r.adjust(pos, pos_r);

                // Get current heading error for trig correction
                float heading_error = imu_rotation() - target_heading;

                // Wall following: adjust heading based on left distance sensor
                float measured_wall_dist = distance_left.objectDistance(vex::distanceUnits::in);

                // If sensor doesn't detect wall, use a large default value
                if (measured_wall_dist > 200 || measured_wall_dist < 1) {
                    measured_wall_dist = target_wall_dist; // No correction if no wall detected
                }

                float angle_rad = heading_error * (3.14159265359 / 180.0);
                float cos_angle = std::cos(angle_rad);
                if (std::abs(cos_angle) < 0.1) cos_angle = 0.1;
                float actual_wall_dist = measured_wall_dist / cos_angle;

                float pid_adjustment_wall_raw = pid_wall.adjust(target_wall_dist, actual_wall_dist);
                float pid_adjustment_wall = pid_adjustment_wall_raw;

                // Clamp wall correction to prevent spinning (max ±50 RPM correction)
                if (pid_adjustment_wall > 50) pid_adjustment_wall = 50;
                if (pid_adjustment_wall < -50) pid_adjustment_wall = -50;

                // INVERT signs - if robot keeps going wrong way, this might help
                pid_adjustment_wall = -pid_adjustment_wall;

                vel_rpm = ips / DRIVE_REV_TO_IN * 60;

                // Debug output every 5 ticks
                static int debug_counter = 0;
                if (debug_counter % 5 == 0) {
                    printf("Meas:%.1f Act:%.1f Tgt:%.1f Err:%.1f Raw:%.1f Out:%.1f L:%d R:%d\n",
                           measured_wall_dist, actual_wall_dist, target_wall_dist,
                           target_wall_dist - actual_wall_dist, pid_adjustment_wall_raw, pid_adjustment_wall,
                           (int)(dir_mod * vel_rpm + pid_adjustment_l + pid_adjustment_wall),
                           (int)(dir_mod * vel_rpm + pid_adjustment_r - pid_adjustment_wall));
                }
                debug_counter++;

                drive_l.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_l + pid_adjustment_wall, VEL_RPM);
                drive_r.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_r - pid_adjustment_wall, VEL_RPM);

                wait(MSEC_PER_TICK, vex::msec);
            }

            // Stop motors after 6 seconds
            drive_l.stop(vex::brakeType::brake);
            drive_r.stop(vex::brakeType::brake);

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
            case 0: // Wall follow kP
                wall_kp += adjust * TUNER;
                if (wall_kp < 0) wall_kp = 0;
                break;
            case 1: // Wall follow kI
                wall_ki += adjust * TUNER;
                if (wall_ki < 0) wall_ki = 0;
                break;
            case 2: // Wall follow kD
                wall_kd += adjust * TUNER;
                if (wall_kd < 0) wall_kd = 0;
                break;
            case 3: // Drive distance
                drive_dist += adjust * 5;
                if (drive_dist < 10) drive_dist = 10;
                break;
            case 4: // Target wall distance
                target_wall_dist += adjust * 1;
                if (target_wall_dist < 3) target_wall_dist = 3;
                break;
        }

        // Display current values
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Wall Follow PID Tuning");

        // Show wall PID values with selection indicator
        B_SCRN.setCursor(2, 1);
        B_SCRN.print("%sWALL kP: %.3f", (tune_mode == 0) ? "> " : "  ", wall_kp);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("%sWALL kI: %.3f", (tune_mode == 1) ? "> " : "  ", wall_ki);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("%sWALL kD: %.3f", (tune_mode == 2) ? "> " : "  ", wall_kd);

        B_SCRN.setCursor(5, 1);
        B_SCRN.print("%sDrive: %.1f in", (tune_mode == 3) ? "> " : "  ", drive_dist);
        B_SCRN.setCursor(6, 1);
        B_SCRN.print("%sWallDist: %.1f in", (tune_mode == 4) ? "> " : "  ", target_wall_dist);

        B_SCRN.setCursor(7, 1);
        B_SCRN.print("Y: Run  A: Next Param");
        B_SCRN.setCursor(8, 1);
        B_SCRN.print("Left: %.2f in", distance_left.objectDistance(vex::distanceUnits::in));

        printf("WALL(kP:%.3f kI:%.3f kD:%.3f) Drive:%.1f WallDist:%.1f\n",
               wall_kp, wall_ki, wall_kd, drive_dist, target_wall_dist);

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

// Tunes distance-following with jitter approach (no PID)
void tune_distance_turn_pid() {
    master.rumble("----");

    // Jitter tuning parameters
    float jitter_mag = 30.0;        // RPM adjustment for jitter
    int jitter_period = 100;        // milliseconds per jitter cycle
    float base_speed = 100.0;       // Forward speed (RPM)
    int test_duration = 3000;       // Test duration (ms)
    float target_distance = 21.5;   // Target distance (inches)

    B_SCRN.clearScreen();

    // Tune mode: 0=jitter_mag, 1=jitter_period, 2=base_speed, 3=test_duration, 4=target_distance
    static int tune_mode = 0;
    const int NUM_MODES = 5;

    while (true) {
        // Run test on Y press
        if (BTN_Y.PRESSED) {
            while (BTN_Y.PRESSED) { wait(20, vex::msec); }

            printf("\n=== STARTING TEST ===\n");
            printf("Target: %.1f in, Speed: %.0f RPM, Jitter: %.0f RPM @ %d ms\n",
                   target_distance, base_speed, jitter_mag, jitter_period);

            // Set target heading for cosine correction
            target_heading = imu_rotation();

            // Run the jitter test
            drive_to_distance_timed(test_duration, target_distance, base_speed, jitter_mag, jitter_period);

            printf("=== TEST COMPLETE ===\n\n");
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
            case 0: // Jitter magnitude
                jitter_mag += adjust * 5;
                if (jitter_mag < 0) jitter_mag = 0;
                break;
            case 1: // Jitter period
                jitter_period += adjust * 20;
                if (jitter_period < 20) jitter_period = 20;
                break;
            case 2: // Base speed
                base_speed += adjust * 10;
                if (base_speed < 10) base_speed = 10;
                break;
            case 3: // Test duration
                test_duration += adjust * 500;
                if (test_duration < 1000) test_duration = 1000;
                break;
            case 4: // Target distance
                target_distance += adjust * 1;
                if (target_distance < 3) target_distance = 3;
                break;
        }

        // Display current values
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("Jitter Distance Tuning");

        B_SCRN.setCursor(2, 1);
        B_SCRN.print("%sJitter Mag: %.0f RPM", (tune_mode == 0) ? "> " : "  ", jitter_mag);
        B_SCRN.setCursor(3, 1);
        B_SCRN.print("%sJitter Per: %d ms", (tune_mode == 1) ? "> " : "  ", jitter_period);
        B_SCRN.setCursor(4, 1);
        B_SCRN.print("%sBase Speed: %.0f RPM", (tune_mode == 2) ? "> " : "  ", base_speed);
        B_SCRN.setCursor(5, 1);
        B_SCRN.print("%sDuration: %d ms", (tune_mode == 3) ? "> " : "  ", test_duration);
        B_SCRN.setCursor(6, 1);
        B_SCRN.print("%sTarget Dist: %.1f in", (tune_mode == 4) ? "> " : "  ", target_distance);

        B_SCRN.setCursor(7, 1);
        B_SCRN.print("Y: Run Test  A: Cycle");
        B_SCRN.setCursor(8, 1);
        B_SCRN.print("Left Sensor: %.1f in", distance_left.objectDistance(vex::distanceUnits::in));

        printf("Jitter(Mag:%.0f Per:%dms) Speed:%.0f Dur:%d Tgt:%.1f\n",
               jitter_mag, jitter_period, base_speed, test_duration, target_distance);

        wait(100, vex::msec);
    }
}

