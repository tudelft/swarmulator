#ifndef FORMATION_H
#define FORMATION_H

#include "controller.h"
#include "eigen3/Eigen/Dense"
#include "template_calculator.h"
#include "types.h"
#include <string>

class formation: public Controller{
    public:
        formation();
        ~formation(){};
        Template_Calculator t;
        float sensor_range;
        float min_sep;

    public:
        virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
        virtual void animation(const uint16_t ID);
        
        template<typename T>
        Vector<T> calc_consensus_vel(Vector<T> pos_t, Vector<T> pos_c, double gain, float d_t, float d_c, std::string type ="");
        Vector<float> get_velocity_cmd(const uint16_t ID);
};

#endif