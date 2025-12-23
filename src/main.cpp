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

#define TEST_FUNCS
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
        vis.setBrightness(75);
        // vis.setWifiMode(vex::vision::wifiMode::on);
        while (1) {
            vis.takeSnapshot(yellow);
            B_SCRN.clearScreen();
            
            if (vis.largestObject.exists) {
                B_SCRN.printAt(20, 20, "X: %d   W: %d", vis.largestObject.centerX, vis.largestObject.width);
                B_SCRN.printAt(20, 40, "Y: %d   H: %d", vis.largestObject.centerY, vis.largestObject.height);
                B_SCRN.printAt(20, 60, "Count: %d", (int)vis.objectCount);
            } else {
                B_SCRN.printAt(20, 20, "No Yellow Found");
            }
            
            wait(50, vex::msec);
        }

        // imu.calibrate();
        // master.ButtonLeft.pressed(tune_fast_pid);
        // master.ButtonRight.pressed(autonomous);
    }


    while (true) {
        wait(20, TIME_MSEC);
    }
}
