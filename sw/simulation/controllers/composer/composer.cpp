#include "composer.h"
#include "formation.h"
#include "wall_avoidance.h"
#include "target_follow.h"

composer::composer(){
    // controllers.push_back(new formation());
    controllers.push_back(new wall_avoidance());
    // target_follow* tf_controller = ;
    // tf_controller->set_waypoints();
    // controllers.push_back(new target_follow({Eigen::Vector3f(-15,-15,5), Eigen::Vector3f(-15,15,5), Eigen::Vector3f(15,-15,5)}));

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