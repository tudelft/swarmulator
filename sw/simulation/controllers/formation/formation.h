#ifndef FORMATION_H
#define FORMATION_H

#include "controller.h"
#include "eigen3/Eigen/Dense"
#include "template_calculator.h"
#include "types.h"
#include "multiranger.h"
#include "ranger.h"
#include <string>

class formation: public Controller{
    public:
        formation();
        ~formation(){};
        Template_Calculator t;
        float sensor_range;
        float min_sep;
        float formation_gain;

    public:
        // virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
        virtual void animation(const uint16_t ID);
        virtual Eigen::Vector3f get_velocity_cmd(const uint16_t ID);
};

#endif