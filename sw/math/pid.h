#ifndef PID_H
#define PID_H

#define MAX_INTEGRAL_VALUE 100

class PID
{
private:
    float _kp;
    float _kd;
    float _ki;

    float _int_e;
    float _last_e;
public:
    PID(float kp, float kd, float ki);
    
    ~PID() {};

    float step(float error, float dt);
};

#endif // PID_H