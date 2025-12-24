/**
 * @param inches distance to travel - postive or negative
 * @param target_ips goal for velocity - positive
 * @param ipss acceleration - positive
 */
void drive_straight(float inches, float target_ips, float ipss, bool do_decel = true);

/**
 * Drives straight towards a goal using vision tracking for a set duration.
 * @param duration_msec The duration to drive in milliseconds.
 * @param target_small_goal If true: targets Small Goal (Lift UP, Slower). If false: targets Big Goal (Lift DOWN, Faster).
 */
void drive_straight_toward_goal(int duration_msec, bool target_small_goal);

/**
 * @param degrees changes target heading
 * @param turn_radius outer radius of the turn
 * @param target_ips goal for velocity - positive
 * @param ipss acceleration
 * @param reversed are we moving backwards?
 */
void drive_turn(float degrees, float turn_radius, float target_ips, float ipss, bool reversed = false);

/**
 * @param inches distance to travel linearly
 * @param target_ips velocity
 * @param ipss acceleration
 * @param do_decel whether to decelerate
 */
void drive_linear(float inches, float target_ips, float ipss, float do_decel = true);

/**
 * @param degrees changes target heading
 * @param target_ips goal for velocity - positive
 * @param ipss acceleration
 * @param turn_radius outer radius of the arc
 * @param reversed are we moving backwards?
 */
void drive_arc(float degrees, float outer_radius, float max_ips, float ipss, bool do_decel, bool reversed);

void turn_pid(float degrees, float ratio, int direction, int waitTime = 1000);

void straight_pid(float dist);

/**
 * @brief Fast PID arcing turn
 *
 * @param x horizontal displacement
 * @param y vertical displacement
 */
void swing_turn(float x, float y);
