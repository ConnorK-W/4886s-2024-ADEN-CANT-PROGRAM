#include "../include/main.h"
#include "stddefs.h"
#include "v5_apitypes.h"
#include "vex_global.h"
#include "vex_triport.h"

#define DRIVE_INSERT vex::gearSetting::ratio6_1
#define INTAKE_INSERT vex::gearSetting::ratio6_1
#define ARM_INSERT vex::gearSetting::ratio_66_3200

vex::brain Brain;
vex::controller master;

vex::motor drive_l1 = vex::motor(PORT11, DRIVE_INSERT, true);
vex::motor drive_l2 = vex::motor(PORT12, DRIVE_INSERT, true);
vex::motor drive_l3 = vex::motor(PORT13, DRIVE_INSERT, false);
// vex::motor drive_r4 = vex::motor(PORT4, DRIVE_INSERT, true);

vex::motor drive_r1 = vex::motor(PORT18, DRIVE_INSERT, true);
vex::motor drive_r2 = vex::motor(PORT19, DRIVE_INSERT, false);
vex::motor drive_r3 = vex::motor(PORT20, DRIVE_INSERT, false);
// vex::motor drive_l4 = vex::motor(PORT8, DRIVE_INSERT, false);

// Subsystem 3
vex::motor intake1 = vex::motor(PORT16, INTAKE_INSERT, false);
vex::motor intake2 = vex::motor(PORT1, INTAKE_INSERT, false);
vex::motor intakeHigh = vex::motor(PORT1, INTAKE_INSERT, false);
Arm arm = Arm(PORT17, vex::gearSetting::ratio36_1, true);

// 3 Wire Ports
vex::digital_out intakeLift = vex::digital_out(PORTC);
vex::digital_out tounge = vex::digital_out(PORTB);
vex::digital_out lift = vex::digital_out(PORTD);
vex::digital_out hood = vex::digital_out(PORTA);


// Sensors
vex::inertial imu = vex::inertial(PORT10);
vex::optical colorSort = vex::optical(PORT1);
vex::timer totalTime = vex::timer();

vex::aivision::colordesc yellow = vex::aivision::colordesc(1, 255, 175, 83, 20.00, 0.30);
vex::aivision aivis = vex::aivision(PORT8, yellow);

vex::motor_group drive_r = vex::motor_group(drive_r1, drive_r2, drive_r3);
vex::motor_group drive_l = vex::motor_group(drive_l1, drive_l2, drive_l3);
vex::motor_group drive_full = vex::motor_group(drive_r1, drive_r2, drive_r3, drive_l1, drive_l2, drive_l3);
vex::motor_group intakeFull = vex::motor_group(intake1, intake2, intakeHigh);
vex::motor_group intakeLow = vex::motor_group(intake1, intake2);

// Potentiometer for lever
vex::potV2 lever_pot = vex::potV2(Brain.ThreeWirePort.E);
// Distance sensors
vex::distance distance_left = vex::distance(PORT6);
vex::distance distance_front = vex::distance(PORT7);