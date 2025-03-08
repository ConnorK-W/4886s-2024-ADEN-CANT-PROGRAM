#include "../include/main.h"
#include "vex_global.h"
#include "vex_imu.h"

// Utility functions

// Returns whether a value is within range of a base value
bool within_range(double value, double base_value, double range) {
    return ((value <= base_value + range) && (value >= base_value - range));
}

// Derived from standard acceleration formula
float stop_dist(float current_vel, float acceleration, float target_vel) {
    return -(target_vel * target_vel - current_vel * current_vel) / (2 * acceleration);
}

// Returns quadrant of brain
int *side_pressed(void) {
    static int sides[2];
    if (!B_SCRN.PRESSED) {
        sides[0] = 0;
        sides[1] = 0;
        return sides;
    }

    if (B_SCRN.xPosition() < B_SCRN_X_MID)
        sides[X] = LEFT;
    else
        sides[X] = RIGHT;

    if (B_SCRN.yPosition() < B_SCRN_Y_MID)
        sides[Y] = UP;
    else
        sides[Y] = DOWN;

    return sides;
}

void tune_gyro(void) {
    imu.calibrate();
    while (imu.isCalibrating())
        wait(20, vex::msec);

    while (1) {
        B_SCRN.clearScreen();
        B_SCRN.setCursor(1, 1);
        B_SCRN.print("%.1f", ROTATION);
        wait(20, vex::msec);
    }
}

void handle_acceleration(float position, float distance, float velocity,
                         float max_velocity, float acceleration, int tick_rate, bool do_decel) {
    if (std::abs(position) + stop_dist(velocity, acceleration) >= std::abs(distance) && do_decel)
        velocity -= acceleration / tick_rate;
    else if (velocity < max_velocity)
        velocity += acceleration / tick_rate;
    else
        velocity = max_velocity;
}

void clear_column(int c) {
    for (int i = 0; i < B_SCRN_Y; i++) {
        draw_colored_pixel(i, c, vex::black);
    }
}

void draw_colored_pixel(int r, int c, vex::color color) {
    B_SCRN.setPenColor(color);
    B_SCRN.drawPixel(r, c);
}

void graph_pid() {
    int setpoint = target_heading;
    int correction = IMU_CORRECTION;
    vex::inertial sensor = imu;

    B_SCRN.clearScreen();

    int p_val = setpoint - sensor.rotation() * correction, i_val = 0, d_val;
    float scalar =  B_SCRN_Y_MID / setpoint;

    for (int i = 0; true; i++) {
        if (i > B_SCRN_X)
            i = 0;

        // Update pid vars
        d_val = p_val;
        p_val = setpoint - sensor.rotation() * correction;
        i_val += p_val;

        // Convert to graphable values
        int g_setpoint = setpoint * scalar;
        int g_sensor_val = sensor.rotation() * correction * scalar;
        int g_p_val = p_val * scalar;
        int g_i_val = i_val * scalar;
        int g_d_val = d_val * scalar;

        // Clear current column
        clear_column(i);

        // Graph lines
        draw_colored_pixel(i, g_setpoint, vex::white);
        draw_colored_pixel(i, g_sensor_val, vex::red);
        draw_colored_pixel(i, g_p_val, vex::blue);
        draw_colored_pixel(i, g_i_val, vex::yellow);
        draw_colored_pixel(i, g_d_val, vex::green);

        vex::wait(20, vex::msec);
    }
}

void lift_limit_torque() {
    if (lift.torque() >= 0) {
        lift.stop();
    }
}
    


// Shorthand functions

// Position in inches
double pos_drive_r() {
    return drive_r.position(vex::rotationUnits::rev) * DRIVE_REV_TO_IN;
}
double pos_drive_l() {
    return drive_l.position(vex::rotationUnits::rev) * DRIVE_REV_TO_IN;
}
// Velocity in inches per second
double vel_drive_r() {
    return drive_r.velocity(vex::rpm) * DRIVE_REV_TO_IN * 60;
}
double vel_drive_l() {
    return drive_l.velocity(vex::rpm) * DRIVE_REV_TO_IN * 60;
}

double raw_vel_dr() {
    return drive_r.velocity(vex::rpm);
}

double raw_vel_dl() {
    return drive_l.velocity(vex::rpm);
}

// Heading in degrees
double imu_rotation() {
    return imu.rotation() * IMU_CORRECTION;
}

void reset_imu_rotation() {
    imu.resetRotation();
    target_heading = 0;
}

// Controller joystick shorthand
double axis_rx() {
    return master.Axis1.position();
}
double axis_ry() {
    return master.Axis2.position();
}
double axis_lx() {
    return master.Axis3.position();
}
double axis_ly() {
    return master.Axis4.position();
}

// Controller button shorthand
double btn_r1() {
    return master.ButtonR1.pressing();
}
double btn_r2() {
    return master.ButtonR2.pressing();
}
double btn_l1() {
    return master.ButtonL1.pressing();
}
double btn_l2() {
    return master.ButtonL2.pressing();
}

double btn_a() {
    return master.ButtonA.pressing();
}
double btn_b() {
    return master.ButtonB.pressing();
}
double btn_x() {
    return master.ButtonX.pressing();
}
double btn_y() {
    return master.ButtonY.pressing();
}

double btn_right() {
    return master.ButtonRight.pressing();
}
double btn_left() {
    return master.ButtonLeft.pressing();
}
double btn_up() {
    return master.ButtonUp.pressing();
}
double btn_down() {
    return master.ButtonDown.pressing();
}