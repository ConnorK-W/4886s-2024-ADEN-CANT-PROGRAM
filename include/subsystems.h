#pragma once
#include "pid.h"
#include "globals.h"

class Arm : public vex::motor {
    private:
        PID pid;
    public:
        Arm(int32_t index, vex::gearSetting gears, bool reverse) 
            : vex::motor(index, gears, reverse), 
              pid(PID(0,0,0)) // Initialize with dummy, will update from globals
              {}
        
        void rotate_pid(double target_val);
        void pid_step(double target_val);
};
