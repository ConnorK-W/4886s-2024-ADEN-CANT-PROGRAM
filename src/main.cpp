/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Aden                                             */
/*    Created:      Mon May 16 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "../include/main.h"

vex::competition Competition;

//#define TEST_FUNCS
#ifdef TEST_FUNCS
const bool run_main = false;
#else
const bool run_main = true;
#endif

int main() {
    if (run_main) {
        Competition.autonomous(autonomous);
        Competition.drivercontrol(opcontrol);
        pre_auton();
    }
    else {
        imu.calibrate();
        master.ButtonLeft.pressed(tune_dir_pid);
        master.ButtonRight.pressed(autonomous);
    }


    while (true) {
        wait(20, TIME_MSEC);
    }
}
