#ifndef _STDDEFS_H_
#define _STDDEFS_H_

// Important Numbers
#define PI 3.1415926535897932384626433832795

// Conversion
// multiple to convert from first to second
// divide to convert from second to first
#define STICK_TO_TRQ 1.0
#define STICK_TO_SPD 2 * 1.0
#define STICK_TO_TRB 3 * 1.0
#define STICK_TO_PCT 1.0
#define BTN_TO_TRQ 100.0
#define BTN_TO_SPD 200.0
#define BTN_TO_TRB 600.0
#define BTN_TO_PCT 100.0
#define BTN_TO_VLT 12.0

#define GEAR_12 12.0
#define GEAR_36 36.0
#define GEAR_48 48.0
#define GEAR_60 60.0
#define GEAR_72 72.0
#define GEAR_84 84.0

#define FULL_ROT_TO_DEG 360.0
#define HALF_ROT_TO_DEG 180.0
#define QRTR_ROT_TO_DEG 90.0

#define IN_TO_CM 2.54
#define RAD_TO_DEG 57.29578
#define SEC_TO_MSEC 1000.0

// Wheels
#define LRG_OMNI_DIAM 4.0
#define MED_OMNI_DIAM 3.25
#define SML_OMNI_DIAM 2.75
#define LRG_WHEEL_DIAM 5.0
#define MED_WHEEL_DIAM 3.25
#define SML_WHEEL_DIAM 2.75
#define TRACT_WHEEL_DIAM 3.25

#define LRG_OMNI_CIRC 13.1
#define MED_OMNI_CIRC 10.2
#define SML_OMNI_CIRC 8.6
#define LRG_WHEEL_CIRC 15.6
#define MED_WHEEL_CIRC 13.1
#define SML_WHEEL_CIRC 8.6
#define TRACT_WHEEL_CIRC 10.2

// Controller Parts
// for use with controller named 'master'
#define LEFT_STICK_Y master.Axis3.value()
#define LEFT_STICK_X master.Axis4.value()
#define RIGHT_STICK_Y master.Axis2.value()
#define RIGHT_STICK_X master.Axis1.value()

#define BTN_L1 master.ButtonL1
#define BTN_L2 master.ButtonL2
#define BTN_R1 master.ButtonR1
#define BTN_R2 master.ButtonR2

#define BTN_UP master.ButtonUp
#define BTN_DOWN master.ButtonDown
#define BTN_LEFT master.ButtonLeft
#define BTN_RIGHT master.ButtonRight

#define BTN_X master.ButtonX
#define BTN_B master.ButtonB
#define BTN_Y master.ButtonY
#define BTN_A master.ButtonA

#define C_SCREEN master.Screen

// Brain Parts
// for use with brain named 'Brain'
#define BATTERY Brain.Battery
#define B_SCRN Brain.Screen
#define B_SCRN_X 480
#define B_SCRN_Y 272
#define B_SCRN_X_MID (B_SCRN_X / 2.0)
#define B_SCRN_Y_MID (B_SCRN_Y / 2.0)

// Sensor Values
// for use with inertial sensor named 'imu' and vision sensor names 'vis'
#define HEADING imu.heading()
#define ROTATION imu.rotation()
#define YAW imu.yaw()
#define ROLL imu.roll()
#define PITCH imu.pitch()

// Ports
#define PORTA Brain.ThreeWirePort.A
#define PORTB Brain.ThreeWirePort.B
#define PORTC Brain.ThreeWirePort.C
#define PORTD Brain.ThreeWirePort.D
#define PORTE Brain.ThreeWirePort.E
#define PORTF Brain.ThreeWirePort.F
#define PORTG Brain.ThreeWirePort.G
#define PORTH Brain.ThreeWirePort.H

#define PORT1 vex::PORT1
#define PORT2 vex::PORT2
#define PORT3 vex::PORT3
#define PORT4 vex::PORT4
#define PORT5 vex::PORT5
#define PORT6 vex::PORT6
#define PORT7 vex::PORT7
#define PORT8 vex::PORT8
#define PORT9 vex::PORT9
#define PORT10 vex::PORT10
#define PORT11 vex::PORT11
#define PORT12 vex::PORT12
#define PORT13 vex::PORT13
#define PORT14 vex::PORT14
#define PORT15 vex::PORT15
#define PORT16 vex::PORT16
#define PORT17 vex::PORT17
#define PORT18 vex::PORT18
#define PORT19 vex::PORT19
#define PORT20 vex::PORT20

// Shorthand parameters
#define DIR_FWD vex::directionType::fwd
#define DIR_REV vex::directionType::rev
#define PCT_PCT vex::percentUnits::pct
#define ROT_DEG vex::rotationUnits::deg
#define ROT_REV vex::rotationUnits::rev
#define ROT_RAW vex::rotationUnits::raw
#define TRQ_NM vex::torqueUnits::Nm
#define TRQ_INLB vex::torqueUnits::InLb
#define TIME_SEC vex::timeUnits::sec
#define TIME_MSEC vex::timeUnits::msec
#define TMP_C vex::temperatureUnits::celsius
#define TMP_F vex::temperatureUnits::fahrenheit
#define VEL_PCT vex::velocityUnits::pct
#define VEL_RPM vex::velocityUnits::rpm
#define VEL_DPS vex::velocityUnits::dps
#define VLT_VLT vex::voltageUnits::volt
#define VLT_MVLT vex::voltageUnits::mV

#endif // _STDDEFS_H_
