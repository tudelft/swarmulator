#include "pid.h"

PID::PID(float kp, float kd, float ki){
    _kp = kp;
    _kd = kd;
    _ki = ki;
}

float PID::step(float error, float dt){
    float de = error - _last_e;
    _last_e = error;

    _int_e += error*dt;
    if (_int_e > MAX_INTEGRAL_VALUE){
        _int_e = MAX_INTEGRAL_VALUE;
    }

    return (_kp*error + _kd*de + _ki*_int_e);
}