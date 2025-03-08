#include "../include/main.h"
#include "stddefs.h"
#include "v5_apitypes.h"
#include "vex_global.h"
#include "vex_triport.h"

#define DRIVE_INSERT vex::gearSetting::ratio6_1
#define INTAKE_INSERT vex::gearSetting::ratio6_1

vex::brain Brain;
vex::controller master;

vex::motor drive_r1 = vex::motor(PORT14, DRIVE_INSERT, false);
vex::motor drive_r2 = vex::motor(PORT15, DRIVE_INSERT, false);
vex::motor drive_r3 = vex::motor(PORT16, DRIVE_INSERT, true);
// vex::motor drive_r4 = vex::motor(PORT4, DRIVE_INSERT, true);

vex::motor drive_l1 = vex::motor(PORT11, DRIVE_INSERT, false);
vex::motor drive_l2 = vex::motor(PORT12, DRIVE_INSERT, true);
vex::motor drive_l3 = vex::motor(PORT13, DRIVE_INSERT, true);
// vex::motor drive_l4 = vex::motor(PORT8, DRIVE_INSERT, false);

// Subsystem 3
vex::motor intakeLow = vex::motor(PORT1, INTAKE_INSERT, true);
vex::motor intakeHigh = vex::motor(PORT19, INTAKE_INSERT, true);

vex::motor lift = vex::motor(PORT3, vex::gearSetting::ratio18_1, false);

// 3 Wire Ports
vex::digital_out mogo_clamp = vex::digital_out(PORTA);
vex::digital_out Smith_MechR = vex::digital_out(PORTB);
vex::digital_out Smith_MechL = vex::digital_out(PORTC);
vex::digital_out intake_lift = vex::digital_out(PORTD);
vex::digital_out PTO = vex::digital_out(PORTC);


// Sensors
vex::inertial imu = vex::inertial(PORT20);
vex::optical colorSort = vex::optical(PORT2);
vex::timer totalTime = vex::timer();

vex::vision::signature red_ring = vex::vision::signature (3, 7579, 10923, 9251, -891, -433, -662, 4, 0);
vex::vision::signature blue_ring = vex::vision::signature (2, -3329, -2595, -2962, 5431, 6685, 6058, 5.8, 0);
vex::vision::signature neutral_post = vex::vision::signature(1, -841, 1, -420, -4645, -3667, -4156, 4.4, 0);

vex::vision vision1 = vex::vision(PORT20, 17, red_ring, blue_ring, neutral_post);

vex::motor_group drive_r = vex::motor_group(drive_r1, drive_r2, drive_r3);
vex::motor_group drive_l = vex::motor_group(drive_l1, drive_l2, drive_l3);
vex::motor_group drive_full = vex::motor_group(drive_r1, drive_r2, drive_r3, drive_l1, drive_l2, drive_l3);
vex::motor_group intake = vex::motor_group(intakeLow, intakeHigh);

vex::rotation liftDegree = vex::rotation(PORT10);