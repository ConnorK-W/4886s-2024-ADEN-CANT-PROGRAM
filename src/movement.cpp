#include "../include/main.h"
#include "stddefs.h"

// Oooh, this is a change!

// Use to drive straight
void drive_straight(float inches, float target_ips, float ipss, bool do_decel) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    drive_r.stop(vex::brakeType::coast);
    drive_l.stop(vex::brakeType::coast);

    PID pid_drive_l = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID pid_drive_r = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID pid_dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    float ips = 0, pos = 0;
    float pos_start_l = pos_drive_l(), pos_start_r = pos_drive_r();
    float pos_l, pos_r;

    // adjusts velocity for positive/negative distances
    float dir_mod = (inches > 0) ? 1 : -1;

    float pid_adjustment_l;
    float pid_adjustment_r;
    float pid_adjustment_dir;

    float vel_rpm;

    while (ips >= 0 && std::abs(pos_drive_l() - pos_start_l) < std::abs(inches)) {
        // Handles getting to speed
        if (std::abs(pos) + stop_dist(ips, ipss) >= std::abs(inches) && do_decel)
            ips -= ipss / TICKS_PER_SEC;
        else if (ips < target_ips)
            ips += ipss / TICKS_PER_SEC;
        else
            ips = target_ips;

        // Find expected position
        pos += ips / TICKS_PER_SEC * dir_mod; // dir_mod adjusts for fwd/bwd

        // Update actual positions
        pos_l = pos_drive_l() - pos_start_l;
        pos_r = pos_drive_r() - pos_start_r;

        // Maintain speed
        pid_adjustment_l = pid_drive_l.adjust(pos, pos_l);
        pid_adjustment_r = pid_drive_r.adjust(pos, pos_r);
        pid_adjustment_dir = pid_dir.adjust(target_heading, imu_rotation());

        vel_rpm = ips / DRIVE_REV_TO_IN * 60;

        drive_l.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_l + pid_adjustment_dir, VEL_RPM);
        drive_r.spin(DIR_FWD, dir_mod * vel_rpm + pid_adjustment_r - pid_adjustment_dir, VEL_RPM);

        wait(MSEC_PER_TICK, vex::msec);
    }
    if (do_decel) {
        drive_r.stop(vex::brakeType::brake);
        drive_l.stop(vex::brakeType::brake);
    } else {
        drive_r.stop(vex::brakeType::coast);
        drive_l.stop(vex::brakeType::coast);
    }
}

void drive_straight_toward_goal(int duration_msec, bool target_small_goal) {
    // 1. SETUP CONFIGURATION BASED ON GOAL TYPE
    if (target_small_goal) {
        lift.set(1); // Small Goal: Lift Up
    } else {
        lift.set(0); // Big Goal: Lift Down
    }

    // Select PID Constants
    PID dir = target_small_goal ? 
        PID(DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KP, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KI, DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KD) : 
        PID(DRIVE_STRAIGHT_TOWARD_BIGGOAL_KP, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KI, DRIVE_STRAIGHT_TOWARD_BIGGOAL_KD);

    // Select Drive Constants (Aggressive Config)
    float final_max_rpm = target_small_goal ? -200.0 : -600.0;
    float accel_base    = target_small_goal ? 10.0 : 14.0;

    // Common Drive PIDs
    PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
    PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);

    // 2. INITIALIZE VARIABLES
    vex::timer t;
    t.clear();

    float current_vel = 0; 
    int last_known_x = 155;   
    int lost_frames = 100; // Start assumed lost to force safe initialization
    int goal_x = 155;
    int factor = 0;

    // 3. MAIN LOOP
    while (t.time(vex::msec) < duration_msec) {
        // Safety: Anti-tip check
        if (imu.roll() >= 6) {
            break; 
        }

        aivis.takeSnapshot(yellow);
        bool has_target = aivis.largestObject.exists;

        // Vision Filtering Logic
        if (has_target) {
            goal_x = aivis.largestObject.centerX;
            last_known_x = goal_x; 
            lost_frames = 0;
            factor = 1;
        } else {
            // Buffer: Keep turning briefly if frame is dropped
            if (lost_frames < 10) {
                goal_x = last_known_x;
                lost_frames++;
                factor = 1; 
            } else {
                goal_x = 155; 
                factor = 0;
            }
        }

        // Speed Calculation (Slow down if not centered)
        float error = std::abs(155 - goal_x);
        if (factor == 0) {
            error = 300.0; // Force high error -> low speed if target lost
        }
        
        float speed_factor = 1.0 - (error / 300.0); 
        
        // Clamp speed factor
        if (speed_factor < 0.6) speed_factor = 0.6;
        if (speed_factor > 1.0) speed_factor = 1.0;

        float active_speed_limit = 2 * final_max_rpm * speed_factor;

        if (factor == 0) {
            active_speed_limit = 0; 
        }

        // Acceleration Ramp
        if (current_vel > active_speed_limit) { 
            current_vel -= accel_base; 
        } else if (current_vel < active_speed_limit) { 
            current_vel += (accel_base * 2.0); 
        }

        // PID Calculation
        double dir_adj = 0;
        if (error > 5) {
            dir_adj = dir.adjust(160, goal_x);
        }

        // Motor Application
        drive_r.spin(DIR_FWD, current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) - factor*dir_adj, VEL_RPM);
        drive_l.spin(DIR_FWD, current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) + factor*dir_adj, VEL_RPM);

        // Get current actual RPM
        double actual_L = drive_l.velocity(VEL_RPM);
        double actual_R = drive_r.velocity(VEL_RPM);
        
        // Print to the debug terminal (L: <val> R: <val>)
        printf("L: %.2f  R: %.2f\n", actual_L, actual_R);
        
        wait(20, vex::msec);
    }

    // 4. STOP
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
void drive_turn(float degrees, float outer_radius, float target_ips, float ipss, bool reversed) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    target_heading += degrees; // update target heading

    // PID pid_drive_l = PID(move_kp, move_ki, move_kd);
    // PID pid_drive_r = PID(move_kp, move_ki, move_kd);

    float pid_adjustment_l;
    float pid_adjustment_r;

    float ips = 0;
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
        if (std::abs(degrees_remaining / RAD_TO_DEG * outer_radius) - stop_dist(ips, ipss) <= 0) {
            ips -= ipss / TICKS_PER_SEC;
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

        printf("rad ratio: %.2f\n", radius_ratio);
        printf("inner :%.2f\nouter: %.2f\n", inner_vel_rpm, outer_vel_rpm);


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
    drive_l.stop(vex::brakeType::brake);
    drive_r.stop(vex::brakeType::brake);
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
