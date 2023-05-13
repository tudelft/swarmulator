#ifndef MULTIRANGER_H
#define MULTIRANGER_H

#include "types.h"
#include "ranger.h"

// class draw; // make circular dependency otherwise

class MultiRanger{
    public:
        MultiRanger();
        std::vector<float> getMeasurements(Pose pose);
        void animate(draw d);

    private:
        // float _num_rangers = 4;
        float _ranger_dist = 2.0; // cm
        enum {FRONT, LEFT, RIGHT, BACK, BOTTOM};
        Vector<Ranger> _rangers = Vector<Ranger>(5);
        std::vector<float> _measurements;


};

#endif