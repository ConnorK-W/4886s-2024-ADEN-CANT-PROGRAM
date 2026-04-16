#include "../include/main.h"
#include "stddefs.h"
#include <cmath>


void drive_straight_with_dist(float inches, float target_ips, float ipss, bool do_decel, float start_ips, float end_ips) {

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

    double df_i = distance_front.objectDistance(vex::distanceUnits::in);
    double dl_i = distance_left.objectDistance(vex::distanceUnits::in);

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

    double df_f = distance_front.objectDistance(vex::distanceUnits::in);
    double dl_f = distance_left.objectDistance(vex::distanceUnits::in);

    double change_l = dl_f - dl_i;
    double change_r = df_f - df_i;

    double theta = std::atan(change_l / change_r) * (180.0 / 3.14159265358979323846);

    offsettheta = theta;
}

void drive_straight_to_dist_value(float target_dist_in, float target_ips, float ipss, vex::distance& dist_sensor, bool do_decel) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;
    const float STOP_VELOCITY_THRESHOLD = 2.0; // Auto-stop when decelerated to 2 ips
    const float KILLSWITCH_VELOCITY = 2.0; // Kill switch if moving less than 2 ips
    const int KILLSWITCH_TIME_MS = 500; // for 0.5 seconds
    const float MAX_VALID_DISTANCE = 100.0; // Max reasonable distance reading in inches

    drive_r.stop(vex::brakeType::coast);
    drive_l.stop(vex::brakeType::coast);

    PID pid_dist = PID(DRIVE_TO_DIST_KP, DRIVE_TO_DIST_KI, DRIVE_TO_DIST_KD);
    PID pid_dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

    float ips = 0;
    int stuck_time_ms = 0; // Track time below velocity threshold

    float pid_adjustment_dir;
    float vel_rpm;

    // Outlier filtering
    float prev_valid_dist = target_dist_in; // Initialize to target
    const float OUTLIER_THRESHOLD = 20.0; // Ignore readings that jump more than 20 inches
    bool has_valid_reading = false;

    while (true) {
        float raw_dist = dist_sensor.objectDistance(vex::distanceUnits::in);

        // Filter out outliers and invalid readings
        float current_dist;
        bool reading_is_valid = false;

        if (raw_dist >= MAX_VALID_DISTANCE) {
            // No valid reading - if we've never had one, keep driving forward
            if (!has_valid_reading) {
                current_dist = target_dist_in + 10.0; // Pretend we're 10" too far, so we keep driving forward
                reading_is_valid = false;
            } else {
                // Lost the reading after having it - use previous valid distance
                current_dist = prev_valid_dist;
                reading_is_valid = true;
            }
        } else if (has_valid_reading && std::abs(raw_dist - prev_valid_dist) > OUTLIER_THRESHOLD) {
            // Outlier spike - use previous valid reading instead
            current_dist = prev_valid_dist;
            reading_is_valid = true;
        } else {
            // Valid reading - use it and update previous
            current_dist = raw_dist;
            prev_valid_dist = raw_dist;
            reading_is_valid = true;
            has_valid_reading = true;
        }

        float dist_error = current_dist - target_dist_in;
        
        // Check if we've reached target and are slow enough to stop
        if (std::abs(dist_error) <= 0.25 && std::abs(ips) <= STOP_VELOCITY_THRESHOLD) {
            break;
        }
        
        // Kill switch: if moving less than 2 ips for 0.5 seconds, exit
        if (std::abs(ips) < KILLSWITCH_VELOCITY) {
            stuck_time_ms += MSEC_PER_TICK;
            if (stuck_time_ms >= KILLSWITCH_TIME_MS) {
                break; // Robot is stuck
            }
        } else {
            stuck_time_ms = 0; // Reset if velocity is above threshold
        }
        
        // PID control for distance - output is desired velocity
        // Negate because: if too far (current > target), we want positive velocity (forward)
        float pid_output = -pid_dist.adjust(target_dist_in, current_dist);
        
        // Limit to target_ips
        float desired_ips = std::max(-target_ips, std::min(target_ips, pid_output));
        
        // Apply acceleration limiting
        if (desired_ips > ips + ipss / TICKS_PER_SEC) {
            ips += ipss / TICKS_PER_SEC;
        } else if (desired_ips < ips - ipss / TICKS_PER_SEC) {
            ips -= ipss / TICKS_PER_SEC;
        } else {
            ips = desired_ips;
        }

        // Direction correction to keep straight
        pid_adjustment_dir = pid_dir.adjust(target_heading, imu_rotation());

        vel_rpm = ips / DRIVE_REV_TO_IN * 60;

        drive_l.spin(DIR_FWD, vel_rpm + pid_adjustment_dir, VEL_RPM);
        drive_r.spin(DIR_FWD, vel_rpm - pid_adjustment_dir, VEL_RPM);

        wait(MSEC_PER_TICK, vex::msec);
    }
    
    // Coast to stop when reaching target (don't brake)
    drive_r.stop(vex::brakeType::coast);
    drive_l.stop(vex::brakeType::coast);
}

// Pattern: 10% straight, 70% correction, 20% straight
void drive_straight_wall_follow(float inches, float target_ips, float ipss, float target_wall_dist, bool do_decel, float start_ips, float end_ips) {
    float total = std::abs(inches);
    float dir = (inches > 0) ? 1 : -1;

    // Safety timeout: exit if robot hasn't moved after 500ms
    vex::timer timeout_timer;
    timeout_timer.clear();

    // PHASE 1: Drive straight for first 10%
    float first_straight = total * 0.1;
    printf("Phase 1: Drive straight %.1f (10%%)\n", first_straight);
    drive_straight(dir * first_straight, target_ips, ipss, false, start_ips, target_ips);

    // Check timeout after first movement
    if (timeout_timer.time(vex::msec) > 500) {
        float current_vel = std::abs(drive_l.velocity(vex::velocityUnits::pct)) * 6.0 / 100.0;  // Convert to approx ips
        if (current_vel < 2.0) {
            printf("Wall follow timeout: robot not moving (%.1f ips)\n", current_vel);
            drive_l.stop(vex::brakeType::brake);
            drive_r.stop(vex::brakeType::brake);
            return;
        }
    }

    // Measure distance to wall after initial straight
    float current_dist = distance_left.objectDistance(vex::distanceUnits::in);

    // PHASE 2: Calculate triangle for middle 70% correction
    float leg1 = current_dist - target_wall_dist;  // Perpendicular
    float leg2 = total * 0.7;                      // Forward distance for correction

    float angle_rad = std::atan2(leg1, leg2);
    float angle_deg = angle_rad * 180.0 / 3.14159265359;
    float hypotenuse = std::sqrt(leg1 * leg1 + leg2 * leg2);

    printf("Phase 2: Triangle CurrentDist=%.1f Target=%.1f Leg1=%.1f Leg2=%.1f\n",
           current_dist, target_wall_dist, leg1, leg2);
    printf("Angle=%.1f° Hypotenuse=%.1f\n", angle_deg, hypotenuse);

    // Turn to angle
    turn_pid(-angle_deg, -1, 1, 500);
    wait(100, vex::msec);

    // Drive hypotenuse
    drive_straight(dir * hypotenuse, target_ips, ipss, false, target_ips, target_ips);
    printf("After correction: %.1f from wall\n", distance_left.objectDistance(vex::distanceUnits::in));

    // Turn back to 0°
    float heading_error = imu_rotation() - target_heading;
    turn_pid(-heading_error, -1, 1, 500);
    wait(100, vex::msec);

    // PHASE 3: Drive remaining 20% straight
    float last_straight = total * 0.2;
    printf("Phase 3: Drive straight %.1f (20%%)\n", last_straight);
    drive_straight(dir * last_straight, target_ips, ipss, do_decel, target_ips, end_ips);

    printf("Final: %.1f from wall (target=%.1f)\n\n",
           distance_left.objectDistance(vex::distanceUnits::in), target_wall_dist);

    if (do_decel && end_ips == 0) {
        drive_l.stop(vex::brakeType::brake);
        drive_r.stop(vex::brakeType::brake);
    }
}

// Backs up to tube position while maintaining distance from left wall
void backup_to_tube_leftdist() {
    // Back up 15 inches at 200 rpm (approx 83 ips) while maintaining 21.5" from left wall
    // Negative distance = reverse
    drive_straight_wall_follow(-15, 83, 100, 21.5, true, 0, 0);
}


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

    while (std::abs(pos_drive_l() - pos_start_l) < std::abs(inches)) {
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
    static char cmd_buf[64];
    static int cmd_idx = 0;

    while (true) {
        aivis.takeSnapshot(yellow);

        vision_mutex.lock();
        latest_vision_data.centerX = aivis.largestObject.centerX;
        latest_vision_data.exists = aivis.largestObject.exists;
        latest_vision_data.width = aivis.largestObject.width;
        latest_vision_data.height = aivis.largestObject.height;
        latest_vision_data.objectCount = aivis.objectCount;

        if (latest_vision_data.exists) {
            printf("VIS: X:%d W:%d H:%d Cnt:%d\n",
                latest_vision_data.centerX, latest_vision_data.width, latest_vision_data.height, latest_vision_data.objectCount);
        }
        vision_mutex.unlock();

        // Check for incoming color signature from tuning tool
        int ch;
        while ((ch = vexSerialReadChar(1)) != -1) {
            if (ch == '\n') {
                cmd_buf[cmd_idx] = '\0';
                if (cmd_buf[0] == 'C') {
                    int id, r, g, b, ha100, hs100;
                    if (sscanf(cmd_buf, "C,%d,%d,%d,%d,%d,%d",
                               &id, &r, &g, &b, &ha100, &hs100) == 6) {
                        float hangle = ha100 / 100.0f;
                        float hdsat  = hs100 / 100.0f;
                        vex::aivision::colordesc tuned_yellow(id, r, g, b, hangle, hdsat);
                        aivis.set(tuned_yellow);
                        printf("SIG: id=%d RGB(%d,%d,%d) h=%.1f s=%.2f\n",
                               id, r, g, b, hangle, hdsat);
                    }
                }
                cmd_idx = 0;
            } else if (cmd_idx < 63) {
                cmd_buf[cmd_idx++] = (char)ch;
            }
        }

        wait(20, vex::msec);
    }
}

void drive_straight_toward_goal(int duration_msec, bool target_small_goal, bool correct) {
    const int TICKS_PER_SEC = 50;
    const int MSEC_PER_TICK = 1000 / TICKS_PER_SEC;

    if (!correct) {
        // Simple time-based drive straight with IMU heading correction
        // ~29 ips = 230 RPM, ~57 ips = 450 RPM (DRIVE_REV_TO_IN ≈ 7.66 in/rev)
        float target_ips = target_small_goal ? 29.0f : 57.0f;
        const float ipss = 75.0f;
        float ips = 0.0f;
        float dir_mod = -1.0f; // driving in reverse toward goals

        PID pid_dir = PID(DRIVE_STRAIGHT_DIR_KP, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);

        vex::timer t;
        t.clear();

        while (t.time(vex::msec) < duration_msec) {
            if (ips < target_ips)
                ips += ipss / TICKS_PER_SEC;
            else
                ips = target_ips;

            float vel_rpm = ips / DRIVE_REV_TO_IN * 60;
            float dir_adj = pid_dir.adjust(target_heading, imu_rotation());

            drive_l.spin(DIR_FWD, dir_mod * vel_rpm + dir_adj, VEL_RPM);
            drive_r.spin(DIR_FWD, dir_mod * vel_rpm - dir_adj, VEL_RPM);

            wait(MSEC_PER_TICK, vex::msec);
        }
    } else {
        // Vision-based correction toward goal
        int TARGET_CENTER = 160;

        PID dir_pid = target_small_goal ?
            PID(drive_smallgoal_kp, drive_smallgoal_ki, drive_smallgoal_kd) :
            PID(drive_biggoal_kp, drive_biggoal_ki, drive_biggoal_kd);
        PID imu_pid = PID(5.0, DRIVE_STRAIGHT_DIR_KI, DRIVE_STRAIGHT_DIR_KD);
        PID rd = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);
        PID ld = PID(DRIVE_STRAIGHT_DL_KP, DRIVE_STRAIGHT_DL_KI, DRIVE_STRAIGHT_DL_KD);

        float current_vel = 0;
        float target_vel = 0;
        float slew_rate = 3.5;
        double dir_adj = 0;

        // Seed from current vision data so correction starts immediately
        vision_mutex.lock();
        bool has_initial = latest_vision_data.exists;
        int last_known_x = has_initial ? latest_vision_data.centerX : 160;
        int goal_x = last_known_x;
        int lost_frames = has_initial ? 0 : 100;
        vision_mutex.unlock();

        // Pre-rotate toward goal before driving
        if (has_initial && std::abs(TARGET_CENTER - goal_x) > 5) {
            if (target_small_goal) {
                // Small goal: rotate until within 30% of initial error with slight reverse
                int initial_err = std::abs(TARGET_CENTER - goal_x);
                int threshold = initial_err * 0.3;
                if (threshold < 5) threshold = 5;
                float pre_adj = dir_pid.adjust(TARGET_CENTER, goal_x);
                float creep = -80.0;
                vex::timer pre_t;
                pre_t.clear();
                while (pre_t.time(vex::msec) < 500) {
                    vision_mutex.lock();
                    if (latest_vision_data.exists) {
                        goal_x = latest_vision_data.centerX;
                        pre_adj = dir_pid.adjust(TARGET_CENTER, goal_x);
                    }
                    vision_mutex.unlock();
                    drive_r.spin(DIR_FWD, creep + pre_adj, VEL_RPM);
                    drive_l.spin(DIR_FWD, creep - pre_adj, VEL_RPM);
                    if (std::abs(TARGET_CENTER - goal_x) <= threshold)
                        break;
                    wait(20, vex::msec);
                }
            } else {
                // Big goal: quick 200ms pre-rotate
                float pre_adj = dir_pid.adjust(TARGET_CENTER, goal_x) * 0.4;
                vex::timer pre_t;
                pre_t.clear();
                while (pre_t.time(vex::msec) < 200) {
                    vision_mutex.lock();
                    if (latest_vision_data.exists) {
                        goal_x = latest_vision_data.centerX;
                        pre_adj = dir_pid.adjust(TARGET_CENTER, goal_x) * 0.4;
                    }
                    vision_mutex.unlock();
                    drive_r.spin(DIR_FWD, pre_adj, VEL_RPM);
                    drive_l.spin(DIR_FWD, -pre_adj, VEL_RPM);
                    if (std::abs(TARGET_CENTER - goal_x) <= 5)
                        break;
                    wait(20, vex::msec);
                }
            }
            dir_pid = target_small_goal ?
                PID(drive_smallgoal_kp, drive_smallgoal_ki, drive_smallgoal_kd) :
                PID(drive_biggoal_kp, drive_biggoal_ki, drive_biggoal_kd);
        }

        vex::timer t;
        t.clear();

        while (t.time(vex::msec) < duration_msec) {
            if (imu.roll() <= -6 && target_small_goal == true)
                break;

            vision_mutex.lock();
            bool has_target = latest_vision_data.exists;
            int current_center_x = latest_vision_data.centerX;
            vision_mutex.unlock();

            if (has_target) {
                goal_x = current_center_x;
                last_known_x = goal_x;
                lost_frames = 0;
            } else if (lost_frames < 10) {
                goal_x = last_known_x;
                lost_frames++;
            }

            if (has_target || lost_frames < 10) {
                current_vel = target_small_goal ? -230.0 : -400.0;
                if (std::abs(TARGET_CENTER - goal_x) > 5)
                    dir_adj = dir_pid.adjust(TARGET_CENTER, goal_x);
                else
                    dir_adj = 0;
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

            float r_out = current_vel + rd.adjust(current_vel, drive_r.velocity(VEL_RPM)) + dir_adj;
            float l_out = current_vel + ld.adjust(current_vel, drive_l.velocity(VEL_RPM)) - dir_adj;

            drive_r.spin(DIR_FWD, r_out, VEL_RPM);
            drive_l.spin(DIR_FWD, l_out, VEL_RPM);

            printf("seen=%d cX=%d goalX=%d err=%d dir=%.2f vel=%.1f L=%.1f R=%.1f lost=%d\n",
                has_target, current_center_x, goal_x, TARGET_CENTER - goal_x,
                dir_adj, current_vel, l_out, r_out, lost_frames);

            wait(20, vex::msec);
        }
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
    while (time_still < 40) {
        if (within_range(imu_rotation(), target_heading, 2.0))
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


// Update your header file (.h) and this .cpp file
void Arm::pid_step(double target_val, double max_speed) {
    pid.set_kP(arm_kp);
    pid.set_kI(arm_ki);
    pid.set_kD(arm_kd);

    double current_val = get_pot_value();
    double pid_val = pid.adjust(target_val, current_val);

    // --- NEW: Limit the speed ---
    if (pid_val > max_speed) pid_val = max_speed;
    if (pid_val < -max_speed) pid_val = -max_speed;

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
