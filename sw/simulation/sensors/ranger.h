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
        std::vector<Eigen::Vector3f> intersections;
        Eigen::Vector3f intersection;
        
        // Vector<Vector<float>> sensor_draw ={Vector<float>({}), Vector<float>({}), Vector<float>({})};
        Eigen::Matrix<float, 5, 3> _lasers = Eigen::Matrix<float, 5, 3>(); // generic laser template
        Eigen::Matrix<float, 5, 3> _lasers_b; // laser coords wrt body frame
        Eigen::Matrix<float, 5, 3> _lasers_w; // laser coords wrt world frame
        int _num_lasers =_lasers.rows()-1;

    public:
        Ranger(){};
        Ranger(Pose pose):self_pose(pose){
            // assert(_num_lasers+1==_lasers.rows());
            // The raw template for this laser sensor
            _lasers.row(0) << 0,0,0; // apex of pyramid
            _lasers.row(1) << -_range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
            _lasers.row(2) << _range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
            _lasers.row(3) << _range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
            _lasers.row(4) << -_range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
            

            // pose of this particular ranger wrt the agent
            _lasers_b =  pose.transform(_lasers);
            print(_lasers_b);
        };
        

        // float getMeasurement();
        
        /*
        pose is defined with respect to map origin
        */
        
        float getMeasurement(Pose parent_pose);

        void animate(draw d);
        
};

#endif