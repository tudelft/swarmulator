#ifndef RANGER_H
#define RANGER_H

#include "types.h"

class Ranger{
    private:
        Pose _pose;
        float _range;
    
    public:

        Ranger(Pose pose):_pose(pose){};
        

        float getMeasurement();
        
        /*
        pose is defined with respect to map origin
        */
        
        float getMeasurement(Pose parent_pose);
        
};

#endif