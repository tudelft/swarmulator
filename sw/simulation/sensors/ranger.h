#ifndef RANGER_H
#define RANGER_H

#include "types.h"
#include "eigen3/Eigen/Dense"
class draw;

class Ranger{
    private:
        Pose self_pose;
        Pose parent_pose;
        float _range = 10; // cm
        float _fov = 25*M_PI/180; // degrees
        Eigen::Vector3f intersection;
        
        Eigen::Matrix<float, 5, 3> _lasers = Eigen::Matrix<float, 5, 3>(); // generic laser template
        Eigen::Matrix<float, 5, 3> _lasers_b; // laser coords wrt body frame
        Eigen::Matrix<float, 5, 3> _lasers_w; // laser coords wrt world frame

    public:
        Ranger(){};
        Ranger(Pose pose);
        float getMeasurement(Pose parent_pose);
        void animate(draw d);
        
};

#endif