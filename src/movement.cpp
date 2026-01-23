#include "../include/main.h"
#include "stddefs.h"
#include <cmath>

// Oooh, this is a change!

// Use to drive straight
void drive_straight(float inches, float target_ips, float ipss, bool do_decel, float start_ips, float end_ips) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    drive_r.stop(vex::brakeType::coast);
    drive_l.stop(vex::brakeType::coast);

    PID pid_drive_l = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID pid_drive_r = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID pid_dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    float ips = start_ips, pos = 0;
    float pos_start_l = pos_drive_l(), pos_start_r = pos_drive_r();
    float pos_l, pos_r;

    // adjusts velocity for positive/negative distances
    float dir_mod = (inches > 0) ? 1 : -1;

    float pid_adjustment_l;
    float pid_adjustment_r;
    float pid_adjustment_dir;

    float vel_rpm;

    while (ips >= 0 && std::abs(pos_drive_l() - pos_start_l) < std::abs(inches)) {
        if (std::abs(pos) + stop_dist(ips, ipss, end_ips) >= std::abs(inches) && do_decel) {
            if (ips > end_ips)
                ips -= ipss / TICKS_PER_SEC;
            else
                ips = end_ips;
        } else if (ips < target_ips)
            ips += ipss / TICKS_PER_SEC;
        else
            ips = target_ips;

        pos += ips / TICKS_PER_SEC * dir_mod;

        pos_l = pos_drive_l() - pos_start_l;
        pos_r = pos_drive_r() - pos_start_r;

        pid_adjustment_l = pid_drive_l.adjust(pos, pos_l);
        pid_adjustment_r = pid_drive_r.adjust(pos, pos_r);
        pid_adjustment_dir = pid_dir.adjust(target_heading, imu_rotation());

        // Clamp integral windup if needed, or just rely on pid impl

        vel_rpm = ips / DRIVE_REV_TO_IN * 60;

        drive_l.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_l + pid_adjustment_dir, VEL_RPM);
        drive_r.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_r - pid_adjustment_dir, VEL_RPM);

        wait(MSEC_PER_TICK, vex::msec);
    }
    if (do_decel && end_ips == 0) {
        drive_r.stop(vex::brakeType::brake);
        drive_l.stop(vex::brakeType::brake);
    } else {
        drive_r.stop(vex::brakeType::coast);
        drive_l.stop(vex::brakeType::coast);
    }
}

void vision_processing_task() {
    while (true) {
        aivis.takeSnapshot(yellow);
        
        vision_mutex.lock();
        latest_vision_data.centerX = aivis.largestObject.centerX;
        latest_vision_data.exists = aivis.largestObject.exists;
        latest_vision_data.width = aivis.largestObject.width;
        latest_vision_data.height = aivis.largestObject.height;
        latest_vision_data.objectCount = aivis.objectCount;

        if (latest_vision_data.exists) {
            // printf("Vision: Found! X:%d W:%d Cnt:%d\n", 
            //     latest_vision_data.centerX, latest_vision_data.width, latest_vision_data.objectCount);
        } else {
            // printf("Vision: No object found\n");
        }

        vision_mutex.unlock();
        
        wait(20, vex::msec);
    }
}

void drive_straight_toward_goal(int duration_msec, bool target_small_goal) {
    
    int CAMERA_CENTER_OFFSET;
    
    if (target_small_goal) {
        lift.set(1);
        CAMERA_CENTER_OFFSET = 0; // was -20
    } else {
        lift.set(0); 
        CAMERA_CENTER_OFFSET = 0; // was -5
    }

    int TARGET_CENTER = 160 + CAMERA_CENTER_OFFSET;

    PID dir_pid = target_small_goal ? 
        PID(DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KP, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KI, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KD) : 
        PID(DRIVE_STRAIGHT_TOWARD_BIGGOAL_KP, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KI, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KD);

    PID imu_pid = PID(5.0, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);

    vex::timer t;
    t.clear();

    float current_vel = 0;
    float target_vel = 0;
    float slew_rate = 5.0; 
    int last_known_x = 160;   
    int lost_frames = 100;
    int goal_x = 160;
    double dir_adj = 0;

    while (t.time(vex::msec) < duration_msec) {
        if (imu.roll() >= 6) {
            break; 
        }

        // Use threaded vision data
        vision_mutex.lock();
        bool has_target = latest_vision_data.exists;
        int current_center_x = latest_vision_data.centerX;
        vision_mutex.unlock();

        if (has_target) {
            goal_x = current_center_x;
            last_known_x = goal_x; 
            lost_frames = 0;
        } else {
            if (lost_frames < 10) {
                goal_x = last_known_x;
                lost_frames++;
            }
        }

        if (has_target || lost_frames < 10) {
            current_vel = target_small_goal ? -200.0 : -400.0;
            
            if (std::abs(TARGET_CENTER - goal_x) > 5) {
                dir_adj = dir_pid.adjust(TARGET_CENTER, goal_x);
            } else {
                dir_adj = 0;
            }
            printf("Error: %d, Correction: %.2f\n", TARGET_CENTER - goal_x, dir_adj);
        } else {
            current_vel = -200.0 * 0.5;
            dir_adj = imu_pid.adjust(0, imu_rotation());
        }

        if (current_vel < target_vel) {
            current_vel += slew_rate;
            if (current_vel > target_vel) current_vel = target_vel;
        } else if (current_vel > target_vel) {
            current_vel -= slew_rate;
            if (current_vel < target_vel) current_vel = target_vel;
        }

        drive_r.spin(DIR_FWD, current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) - dir_adj, VEL_RPM);
        drive_l.spin(DIR_FWD, current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) + dir_adj, VEL_RPM);
        
        wait(20, vex::msec);
    }

    drive_r.stop(vex::brakeType::brake);
    drive_l.stop(vex::brakeType::brake);
}

// TODO: get rid of reversed and just use a negative outer_radius
/* Currently bugged
 * can't arc backwards
 * to turn right, degrees > 0 and reversed = false
 * to turn left, degrees < 0 and reversed = true
 */

// Use for arc
void drive_turn(float degrees, float outer_radius, float target_ips, float ipss, bool reversed, float start_ips, float end_ips) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    target_heading += degrees; // update target heading

    // PID pid_drive_l = PID(move_kp, move_ki, move_kd);
    // PID pid_drive_r = PID(move_kp, move_ki, move_kd);

    float pid_adjustment_l;
    float pid_adjustment_r;

    float ips = start_ips;
    float outer_vel_rpm, inner_vel_rpm;
    float outer_pos = 0, inner_pos;                             // expected distance that outer side has travelled
    float pos_start_l = pos_drive_l(), pos_start_r = pos_drive_r(); // start positions
    float pos_l, pos_r;                                         // current positions for each drive side

    float degrees_remaining;

    // Does what it says. Also, implicity sets inner side negative if turning in place
    int rad_mod = (outer_radius > 0) ? -1 : 1;
    float inner_radius = outer_radius + rad_mod * WHEEL_TO_WHEEL_DIST;
    float radius_ratio = inner_radius / outer_radius;

    // adjusts for different outer wheel sides
    int dir_mod = (degrees > 0) ? 1 : -1;


    while (ips >= 0) {
        // Update values
        pos_l = pos_drive_l() - pos_start_l;
        pos_r = pos_drive_r() - pos_start_r;

        degrees_remaining = target_heading - imu_rotation();

        // Handle acceleration
        if (std::abs(degrees_remaining / RAD_TO_DEG * outer_radius) - stop_dist(ips, ipss, end_ips) <= 0) {
            if (ips > end_ips)
                ips -= ipss / TICKS_PER_SEC;
            else
                ips = end_ips;
        } else if (ips < target_ips)
            ips += ipss / TICKS_PER_SEC;
        else
            ips = target_ips;

        // Translate ips to rpm
        outer_vel_rpm = ips / DRIVE_REV_TO_IN * 60 * dir_mod * (-rad_mod);
        inner_vel_rpm = outer_vel_rpm * radius_ratio;

        // Track position
        outer_pos += ips / TICKS_PER_SEC;
        inner_pos = outer_pos * radius_ratio;

        // printf("rad ratio: %.2f\n", radius_ratio);
        // printf("inner :%.2f\nouter: %.2f\n", inner_vel_rpm, outer_vel_rpm);


        // Get PID adjustments
        if (outer_radius < 0) { // left is inner side
            // pid_adjustment_l = pid_drive_l.adjust(inner_pos, pos_l);
            // pid_adjustment_r = -1 * pid_drive_r.adjust(outer_pos, pos_r);

            // drive_l.spin(DIR_FWD, inner_vel_rpm + pid_adjustment_l, VEL_RPM);
            // drive_r.spin(DIR_FWD, outer_vel_rpm + pid_adjustment_r, VEL_RPM);

            drive_l.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
        } else { // right is inner side
            // pid_adjustment_l = -1 * pid_drive_l.adjust(outer_pos, pos_l);
            // pid_adjustment_r = pid_drive_r.adjust(inner_pos, pos_r);

            // drive_l.spin(DIR_FWD, outer_vel_rpm + pid_adjustment_l, VEL_RPM);
            // drive_r.spin(DIR_FWD, inner_vel_rpm + pid_adjustment_r, VEL_RPM);

            drive_l.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
        }

        // Exit if we're past the desired angle
        if (degrees_remaining * dir_mod < 0)
            break;

        wait(MSEC_PER_TICK, vex::msec);
    }
    
    if (end_ips == 0) {
        drive_l.stop(vex::brakeType::brake);
        drive_r.stop(vex::brakeType::brake);
    } else {
        drive_l.stop(vex::brakeType::coast);
        drive_r.stop(vex::brakeType::coast);
    }
}

// degrees, -1, 1
//Turn in place
void turn_pid(float degrees, float ratio, int direction, int waitTime) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    // float ratio = (radius - WHEEL_TO_WHEEL_DIST) / radius;

    target_heading += degrees;
    PID drive_pid = PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD);
    //PID drive_pid = PID(move_kp, move_ki, move_kd);

    float speed_l;
    float speed_r;

    int time_still = 0;
    int time = totalTime.time();
    while (time_still < 60) {
        if (within_range(imu_rotation(), target_heading, 3.0))
            time_still += MSEC_PER_TICK;
        else
            time_still = 0;
        
        if (waitTime !=0 ) {
            if (totalTime.time() - time >= waitTime)
                break;
        }

        speed_l = drive_pid.adjust(target_heading, imu_rotation()) * direction;
        speed_r = speed_l * ratio;

        // Limit to max speed
        // if (speed_l > 12 * std::abs(ratio))
        //     speed_l = 12 * std::abs(ratio);
        // else if (speed_l < -12 * std::abs(ratio))
        //     speed_l = -12 * std::abs(ratio);
        // if (speed_r > 12 * std::abs(ratio))
        //     speed_r = 12 * std::abs(ratio);
        // else if (speed_r < -12 * std::abs(ratio))
        //     speed_r = -12 * std::abs(ratio);

        drive_l.spin(DIR_FWD, speed_l, VLT_VLT);
        drive_r.spin(DIR_FWD, speed_r, VLT_VLT);

        wait(MSEC_PER_TICK, vex::msec);
    }
    drive_r.stop();
    drive_l.stop();
    wait(4, vex::msec); // without this, drive_straight() immediately after veers right
}

// Double Arc Implementation
void drive_double_turn(float degrees1, float outer_radius1, float target_ips1, float ipss1,
                       float degrees2, float outer_radius2, float target_ips2, float ipss2,
                       bool reversed, float start_ips, float end_ips) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    // --- TURN 1 ---
    target_heading += degrees1; 

    float ips = start_ips;
    float outer_vel_rpm, inner_vel_rpm;
    float outer_pos = 0, inner_pos;
    float pos_start_l = pos_drive_l(), pos_start_r = pos_drive_r();
    float pos_l, pos_r;
    float degrees_remaining;

    int rad_mod = (outer_radius1 > 0) ? -1 : 1;
    float inner_radius = outer_radius1 + rad_mod * WHEEL_TO_WHEEL_DIST;
    float radius_ratio = inner_radius / outer_radius1;
    int dir_mod = (degrees1 > 0) ? 1 : -1;
    
    int stall_timer = 0;

    // NOTE: Deceleration Logic Restored for distinct turns
    while (ips >= 0) {
        pos_l = pos_drive_l() - pos_start_l;
        pos_r = pos_drive_r() - pos_start_r;
        degrees_remaining = target_heading - imu_rotation();

        // Handle deceleration for Turn 1
        if (std::abs(degrees_remaining / RAD_TO_DEG * outer_radius1) - stop_dist(ips, ipss1) <= 0) {
            ips -= ipss1 / TICKS_PER_SEC;
        } else if (ips < target_ips1)
            ips += ipss1 / TICKS_PER_SEC;
        else
            ips = target_ips1;

        outer_vel_rpm = ips / DRIVE_REV_TO_IN * 60 * dir_mod * (-rad_mod);
        inner_vel_rpm = outer_vel_rpm * radius_ratio;

        outer_pos += ips / TICKS_PER_SEC;
        inner_pos = outer_pos * radius_ratio;

        if (outer_radius1 < 0) { // left is inner side
            drive_l.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
        } else { // right is inner side
            drive_l.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
        }

        if (degrees_remaining * dir_mod < 0)
            break;
            
        // Stall Detection Turn 1
        if (std::abs(raw_vel_dl()) < 1.0 && std::abs(raw_vel_dr()) < 1.0) {
            stall_timer += MSEC_PER_TICK;
            if (stall_timer > 250) break;
        } else {
            stall_timer = 0;
        }

        wait(MSEC_PER_TICK, vex::msec);
    }
    
    // --- TURN 2 ---
    // Update heading for second part
    target_heading += degrees2;
    
    // Recalculate geometry for Turn 2
    rad_mod = (outer_radius2 > 0) ? -1 : 1;
    inner_radius = outer_radius2 + rad_mod * WHEEL_TO_WHEEL_DIST;
    radius_ratio = inner_radius / outer_radius2;
    dir_mod = (degrees2 > 0) ? 1 : -1;
    
    // Reset start positions for Turn 2 tracking
    pos_start_l = pos_drive_l(); 
    pos_start_r = pos_drive_r();
    outer_pos = 0;
    inner_pos = 0;
    
    // Start Turn 2 from stop (or near stop) to ensure distinct motion
    ips = 0;  
    
    stall_timer = 0; // Reset timer for Turn 2

    while (ips >= 0) {
        pos_l = pos_drive_l() - pos_start_l;
        pos_r = pos_drive_r() - pos_start_r;
        degrees_remaining = target_heading - imu_rotation();

        // Handle Deceleration for end of Turn 2
        // Calculate arc length remaining: degrees_remaining / 57.3 * radius
        if (std::abs(degrees_remaining / RAD_TO_DEG * outer_radius2) - stop_dist(ips, ipss2, end_ips) <= 0) {
             if (ips > end_ips)
                ips -= ipss2 / TICKS_PER_SEC;
            else
                ips = end_ips;
        } else if (ips < target_ips2) {
             // If second turn is faster, accelerate to it
             if(ips < target_ips2) ips += ipss2 / TICKS_PER_SEC;
        } else {
             // If second turn is slower limit to it, OR if we are cruising
             if(ips > target_ips2) ips = target_ips2; // simple hard clamp for now
        }

        // Failsafe for end of motion
        if (ips < 5 && std::abs(degrees_remaining / RAD_TO_DEG * outer_radius2) < 2) break;

        outer_vel_rpm = ips / DRIVE_REV_TO_IN * 60 * dir_mod * (-rad_mod);
        inner_vel_rpm = outer_vel_rpm * radius_ratio;

        outer_pos += ips / TICKS_PER_SEC;
        inner_pos = outer_pos * radius_ratio;

        if (outer_radius2 < 0) { // left is inner side
            drive_l.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
        } else { // right is inner side
            drive_l.spin(DIR_FWD, outer_vel_rpm, VEL_RPM);
            drive_r.spin(DIR_FWD, inner_vel_rpm, VEL_RPM);
        }

        if (degrees_remaining * dir_mod < 0)
            break;
            
        // Stall Detection Turn 2
        if (std::abs(raw_vel_dl()) < 1.0 && std::abs(raw_vel_dr()) < 1.0) {
            stall_timer += MSEC_PER_TICK;
            if (stall_timer > 250) break;
        } else {
            stall_timer = 0;
        }

        wait(MSEC_PER_TICK, vex::msec);
    }

    if (end_ips == 0) {
        drive_l.stop(vex::brakeType::brake);
        drive_r.stop(vex::brakeType::brake);
    } else {
        drive_l.stop(vex::brakeType::coast);
        drive_r.stop(vex::brakeType::coast);
    }
}

void wiggle(int num_wiggles, float angle, int duration_msec) {
    for (int i = 0; i < num_wiggles; i++) {
        target_heading += angle;
        wait(duration_msec, vex::msec);
        target_heading -= 2 * angle;
        wait(duration_msec, vex::msec);
        target_heading += angle;
    }
}


void Arm::pid_step(double target_val) {
    // Update PID constants from globals
    // This allows dynamic tuning to work immediately
    pid.set_kP(arm_kp);
    pid.set_kI(arm_ki);
    pid.set_kD(arm_kd);

    double current_val = get_pot_value();
    double pid_val = pid.adjust(target_val, current_val);

    this->spin(vex::directionType::fwd, pid_val, vex::velocityUnits::pct);
}

void Arm::rotate_pid(double target_val) {
    // Re-initialize or reset PID if needed, but since we have a member PID, 
    // we can just rely on the step function updating constants.
    // However, for a *blocking* move, we might want to reset integral windup?
    // The current PID implementation doesn't expose reset(), but creating a new PID object does.
    // For now, let's just use the member PID to persist state which is generally better.
    
    double current_val = 0;
    double prev_val = 0;
    int stall_timer = 0;

    // Run loop to actively hold position
    while (true) {
        current_val = get_pot_value();
        
        // Check if controller button B is pressed to exit the loop
        if (master.ButtonB.pressing()) {
            break;
        }

        // Exit conditions
        if (current_val < target_val) {
             break;
        }

        // Stall detection
        if (std::abs(current_val - prev_val) < 0.05) {
            stall_timer += 20;
        } else {
            stall_timer = 0;
        }

        if (stall_timer > 250) {
            break;
        }

        prev_val = current_val;

        // Perform one step
        this->pid_step(target_val);

        vex::task::sleep(20);
    }
    this->stop();
}