#include "../include/main.h"
#include "stddefs.h"

PID::PID() {}

PID::PID(double init_kP, double init_kI, double init_kD) {
    kP = init_kP;
    kI = init_kI;
    kD = init_kD;

    error = 0;
    sum = 0;
    prev_error = 0;
    deriv = 0;
}

void PID::tune_kP(float value, float mod) {
    kP += value * mod;
    printf("kP: %lf\n", kP);
    B_SCRN.setCursor(3, 4);
    B_SCRN.print("kP: %lf", kP);
}

void PID::tune_kI(float value, float mod) {
    kI += value * mod;
    printf("kI: %lf\n", kI);
    B_SCRN.setCursor(5, 4);
    B_SCRN.print("kI: %lf", kI);
}

void PID::tune_kD(float value, float mod) {
    kD += value * mod;
    printf("kD: %lf\n", kD);
    B_SCRN.setCursor(7, 4);
    B_SCRN.print("kD: %lf", kD);
}

float PID::get_const(char constant) {
    if (constant == 'p')
        return this->kP;
    if (constant == 'i')
        return this->kI;
    if (constant == 'd')
        return this->kD;
    return 0;
}

double PID::adjust(double setpoint, double sensor_value) {
    error = setpoint - sensor_value;
    // Only increment sum if I loop is used
    if (kI != 0)
        sum += error;
    deriv = error - prev_error;
    prev_error = error;

    return (error * kP) + (sum * kI) + (deriv * kD);
}

