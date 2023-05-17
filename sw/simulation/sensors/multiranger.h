#ifndef MULTIRANGER_H
#define MULTIRANGER_H

#include "types.h"
#include "ranger.h"

class MultiRanger{
    public:
        MultiRanger();
        std::vector<float> getMeasurements(Pose pose);
        Eigen::MatrixXf getAvoidDirections();
        void animate(draw d);
        Vector<Ranger> _rangers = Vector<Ranger>(5);

    private:
        // float _num_rangers = 4;
        float _ranger_dist = 2.0; // cm
        enum {FRONT, LEFT, RIGHT, BACK, BOTTOM};
        
        std::vector<float> _measurements = std::vector<float>(5, 10000);

};

#endif