#ifndef RANGER_H
#define RANGER_H

#include "types.h"
#include "eigen3/Eigen/Dense"
class draw;

class Ranger{
    private:
        Pose _pose;
        float _range = 10; // cm
        float _fov = 25; // degrees
        // Vector<Vector<float>> sensor_draw ={Vector<float>({}), Vector<float>({}), Vector<float>({})};
        Eigen::MatrixXf _lasers = Eigen::MatrixXf(5,3); // laser coords with respect to body frame
        Eigen::MatrixXf _lasers_t = Eigen::MatrixXf(5,3); // laser coords wrt global pose of sensor

    public:
        Ranger(){};
        Ranger(Pose pose):_pose(pose){
            _lasers.row(0) << -_range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
            _lasers.row(1) << _range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
            _lasers.row(2) << _range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
            _lasers.row(3) << -_range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
            _lasers.row(4) << 0,0,0;

            _lasers_t =  pose.transform(_lasers);
        };
        

        // float getMeasurement();
        
        /*
        pose is defined with respect to map origin
        */
        
        float getMeasurement(Pose parent_pose);

        void animate(draw d);
        
};

#endif