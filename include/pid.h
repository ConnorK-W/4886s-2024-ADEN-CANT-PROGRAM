#pragma once
#include "globals.h"

/**
 * @brief pid object for general pid cojtroo
 */
class PID {
    private:
        double kP;
        double kI;
        double kD;

        double error;
        double sum;
        double prev_error;
        double deriv;

        bool do_print_val;
    public:
        PID();
        PID(double init_kP, double init_kI, double init_kD);
        double adjust(double setpoint, double sensor_value);
        void tune_kP(float value, float mod);
        void tune_kI(float value, float mod);
        void tune_kD(float value, float mod);

        void set_kP(double val) { kP = val; }
        void set_kI(double val) { kI = val; }
        void set_kD(double val) { kD = val; }

        float get_const(char constant);
};

