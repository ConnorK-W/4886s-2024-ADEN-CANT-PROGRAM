// PID constants

// drive_straight() consts acceleration
#define DRIVE_STRAIGHT_DL_KP 0.155
#define DRIVE_STRAIGHT_DL_KI 0.0
#define DRIVE_STRAIGHT_DL_KD 0.650

// drive_straight_toward_biggoal() consts acceleration = 12.0 | final_max_rpm = -300
#define DRIVE_STRAIGHT_TOWARD_BIGGOAL_KP (0.8) 
#define DRIVE_STRAIGHT_TOWARD_BIGGOAL_KI 0.0
#define DRIVE_STRAIGHT_TOWARD_BIGGOAL_KD (0.0)

// drive_straight_toward_smallgoal() consts accel_base = 9.0, | final_max_rpm = -100
#define DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KP (0.95) 
#define DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KI 0.0
#define DRIVE_STRAIGHT_TOWARD_SMALLGOAL_KD (0.0)

// Direction control
#define DRIVE_STRAIGHT_DIR_KP 3.0
#define DRIVE_STRAIGHT_DIR_KI 0.00
#define DRIVE_STRAIGHT_DIR_KD 8.275


// drive_turn() consts
#define DRIVE_TURN_KP 0.0
#define DRIVE_TURN_KI 0.0
#define DRIVE_TURN_KD 0.0

#define DRIVE_TURN_DIR_KP 2.925
#define DRIVE_TURN_DIR_KI 0.00
#define DRIVE_TURN_DIR_KD 0.0


// turn_pid() consts
#define TURN_PID_KP 0.325
#define TURN_PID_KI 0.00
#define TURN_PID_KD 1.225
