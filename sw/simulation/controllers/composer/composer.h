#ifndef COMPOSER_H
#define COMPOSER_H

#include "controller.h"
#include "eigen3/Eigen/Dense"
#include "template_calculator.h"
#include "types.h"
#include "multiranger.h"
#include "ranger.h"
#include <string>

class composer: public Controller{
    public:
        composer();
        ~composer(){};
        
        // Vector of all controllers that contribute
        std::vector<Controller*> controllers;

        // obstacle avoidancee
        MultiRanger multi_ranger;
        float sensor_range;
        
        // flocking
        float min_sep;

    public:
        virtual void animation(const uint16_t ID);
        virtual Eigen::Vector3f get_velocity_cmd(const uint16_t ID);
};

#endif