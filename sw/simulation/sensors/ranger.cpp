#include "ranger.h"
#include "draw.h"

void Ranger::animate(draw d){
    Eigen::Vector3f p1 = _lasers_t.row(0);
    Eigen::Vector3f p2 = _lasers_t.row(1);
    Eigen::Vector3f p3 = _lasers_t.row(2); 
    Eigen::Vector3f p4 = _lasers_t.row(3);
    Eigen::Vector3f p5 = _lasers_t.row(4);
    
    d.line(p5, p1);
    d.line(p5, p2);
    d.line(p5, p3);
    d.line(p5, p4);
    d.rect(p1, p2, p3, p4);

}

float Ranger::getMeasurement(Pose pose){
    return 1.0;
}