/**
 * @brief detects when a value is within range of a base value
 *
 * @param value value to check against
 * @param base_value center of valid range
 * @param range distance from base that returns true
 * @return whether it is within range of base
 */
bool within_range(double value, double base_value, double range);

/**
 * @brief finds distance to reach a target velocity given a certain acceleraion
 *
 * @param current_vel the current velocity
 * @param accel the expected acceleration
 * @param target_vel the target velocity to reach
 * @return the distance to reach target velocity
 */
float stop_dist(float current_vel, float accel, float target_vel = 0);

/**
 * @brief detects which side of the brain is pressed
 *
 * @return an array [x, y]
 */
int *side_pressed(void);

/**
 * @brief find gyro discrepencies
 */
void tune_gyro(void);

/**
 * @brief changes velocity at the acceleration rate
 *
 * @param dist disance travelled so far
 * @param target_dist desired distance to travel
 * @param vel current velocity
 * @param max_vel maximum velocity allowed
 * @param accel rate of acceleration
 * @param tps number of times this function is called per time unit of accel
 * @param do_decel whether to decelerate
 */
void handle_acceleration(float position, float distance, float velocity,
        float max_velocity, float acceleration, int tick_rate, bool do_decel);

/**
 * @brief Clears brain screen column c
 */
void clear_column(int c);

/**
 * @brief like vex::brain::screen::drawPixel(), but inncludes a hue
 */
void draw_colored_pixel(int r, int c, vex::color color);

/**
 * @brief Creates graph for pid turning
 *
 * @param setpoint Target value
 * @param sensor The inertial sensor object
 * @param correction The correction for the inertial sensor
 */
void graph_pid();

/**
 * @brief limmits the torque of lift and tells it to stop
*/
void lift_limit_torque(void);


// Shorthand functions

/**
 * @brief Gets position of right drive side in inches
 */
double pos_drive_r(void);

/**
 * @brief Gets position of left drive side in inches
 */
double pos_drive_l(void);

/**
 * @brief Gets velocity of right drive side in inches per second
 */
double vel_drive_r(void);

/**
 * @brief Gets velocity of left drive side in inches per second
 */
double vel_drive_l(void);

/**
 * @brief Gets velocity of right drive side in motor rpm
 */
double raw_vel_dr(void);

/**
 * @brief Gets velocity of left drive side in motor rpm
 */
double raw_vel_dl(void);

/**
 * @brief Gets heading of drive in degrees
 */
double imu_rotation(void);

/**
 * @brief Resets the imu heading
 */
void reset_imu_rotation(void);

/**
 * @brief Get right stick horizontal axis value
 *
 * @return Return value between -100 and 100
 */
double axis_rx(void);

/**
 * @brief Get right stick vertical axis value
 *
 * @return Return value between -100 and 100
 */
double axis_ry(void);

/**
 * @brief Get left stick horizontal axis value
 *
 * @return Return value between -100 and 100
 */
double axis_lx(void);

/**
 * @brief Get left stick vertical axis value
 *
 * @return Return value between -100 and 100
 */
double axis_ly(void);

/**
 * @brief Get state of button R1
 */
double btn_r1(void);

/**
 * @brief Get state of button R2
 */
double btn_r2(void);

/**
 * @brief Get state of button L1
 */
double btn_l1(void);

/**
 * @brief Get state of button L2
 */
double btn_l2(void);

/**
 * @brief Get state of button A
 */
double btn_a(void);

/**
 * @brief Get state of button B
 */
double btn_b(void);

/**
 * @brief Get state of button X
 */
double btn_x(void);

/**
 * @brief Get state of button Y
 */
double btn_y(void);

/**
 * @brief Get state of button Right
 */
double btn_right(void);

/**
 * @brief Get state of button Left
 */
double btn_left(void);

/**
 * @brief Get state of button Up
 */
double btn_up(void);

/**
 * @brief Get state of button Down
 */
double btn_down(void);

