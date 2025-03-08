// Drive gear ratio
#define DRIVE_GEARING (GEAR_36 / GEAR_48)
// Connversion factor from motor revolutions to inches travelled
#define DRIVE_REV_TO_IN (DRIVE_GEARING * PI * MED_OMNI_DIAM)
#define WHEEL_TO_WHEEL_DIST (11.5)

#define FULL_AWP 0
#define RED_RUSH 1
#define RED_QUAD 2
#define BLUE_QUAD 3
#define BLUE_RUSH 4
#define SKILLS 5
#define ELIMS_BLUE 6
#define ELIMS_RED 7

// Operator control modes
#define TNK 0
#define OSA 1
#define TSA 2

// Brain sides and coordinates
#define X 0
#define Y 1
#define LEFT 1
#define UP 1
#define RIGHT 2
#define DOWN 2

// Correct inertial sensor error
#define IMU_CORRECTION (0.9964)
// Height in pixels of default characters on brain
#define CHAR_HI 20
#define CHAR_WI 12

// Multiplier for turn sensitivty
#define SENSITIVITY 0.60

// Global variable declarations
extern int auton_mode;
extern float target_heading;

// Selection
extern const char *control_types[6];
extern const char *auton_types[4];

extern float turn_kp_l;
extern float turn_ki_l;
extern float turn_kd_l;

extern float turn_kp_r;
extern float turn_ki_r;
extern float turn_kd_r;

extern float move_kp, move_ki, move_kd;

extern float drive_accel_kp, drive_accel_ki, drive_accel_kd;
extern float drive_dir_kp, drive_dir_ki, drive_dir_kd;

extern vex::color color;