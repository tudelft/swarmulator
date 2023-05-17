#include "composer.h"
#include "formation.h"
#include "wall_avoidance.h"

composer::composer(){
    controllers.push_back(new formation());
    controllers.push_back(new wall_avoidance());
}

Eigen::Vector3f composer::get_velocity_cmd(const uint16_t ID){
    Eigen::Vector3f v_des(0,0,0);
    for (Controller* controller: controllers)
        v_des += controller->get_velocity_cmd(ID);

    return v_des.normalized()*3;
}

void composer::animation(const uint16_t ID){
    for (Controller* controller: controllers){
        controller->animation(ID);
    }
}