#ifndef MULTIRANGER_H
#define MULTIRANGER_H

#include "types.h"
#include "ranger.h"

class MultiRanger{
    public:
        MultiRanger();
        Vector<float> getMeasurements(Pose pose);

    private:
        // float _num_rangers = 4;
        float _ranger_dist;
        enum {FRONT, LEFT, RIGHT, BACK, BOTTOM};
        Vector<Ranger> _rangers;
        Vector<float> _measurements;

};


#endif