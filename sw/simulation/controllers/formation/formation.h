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
        MultiRanger multi_ranger;
        Ranger ranger;
        float sensor_range;
        float min_sep;

    public:
        virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
        virtual void animation(const uint16_t ID);
        
        
        Eigen::Vector3f calc_consensus_vel(Eigen::Vector3f pos_t, Eigen::Vector3f pos_c, double gain, float d_t, float d_c, std::string type ="");
        Eigen::Vector3f get_velocity_cmd(const uint16_t ID);
};

#endif