#include "multiranger.h"
#include "ranger.h"
#include "math.h"
#include "draw.h"

MultiRanger::MultiRanger(){
    float theta = M_PI/2; 
    // these poses should be defined relative to the ranger default template. check the constructor to see how it's defined
    _rangers[FRONT] = Ranger(Pose({_ranger_dist, 0, 0}, {std::cos(theta/2), 0, std::sin(theta/2), 0}));
    _rangers[LEFT] = Ranger(Pose({0, _ranger_dist, 0}, {std::cos(-theta/2), std::sin(-theta/2), 0, 0}));
    _rangers[RIGHT] = Ranger(Pose({0, -_ranger_dist, 0}, {std::cos(theta/2), std::sin(theta/2), 0, 0}));
    _rangers[BACK] = Ranger(Pose({-_ranger_dist, 0, 0}, {std::cos(-theta/2),0, std::sin(-theta/2), 0 }));
    _rangers[BOTTOM] = Ranger(Pose({0, 0, 0}, {std::cos(theta), std::sin(theta), 0 , 0}));

}

std::vector<float> MultiRanger::getMeasurements(Pose pose){
    _measurements.clear();
    for (int i=0; i<_rangers.get_len(); i++){
        _measurements.push_back( _rangers[i].getMeasurement(pose));
    }
    // return Vector<float>({1});
    return _measurements;
}

void MultiRanger::animate(draw d){
    for (int i=0; i<_rangers.get_len(); i++){
        _rangers[i].animate(d);
    }
}