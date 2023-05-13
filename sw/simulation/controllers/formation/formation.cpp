#include "formation.h"
#include "main.h"
#include <vector>
#include "draw.h"
// #include "multiranger.h"
#include "types.h"
#include "math.h"

#define SENSORS 8
#define SENSOR_MAX_RANGE 1.8

formation::formation(): t(SENSORS, SENSOR_MAX_RANGE){
    if (!strcmp(param->policy().c_str(), "")) {
        terminalinfo::error_msg("Please specify a valid policy file");
    } else {
        t.set_adjacency_matrix(param->policy());
    }

    sensor_range = SENSOR_MAX_RANGE;
    min_sep = 20;
    // print("\n",t.adjacency_mat_mag(1,0));
    float theta = M_PI/2; 
    // ranger = Ranger(Pose({2, 0, 0},  {std::cos(theta/2), 0, std::sin(theta/2), 0}));
    multi_ranger = MultiRanger();
}

void formation::animation(const uint16_t ID)
{
    draw d;
    d.circle_loop(sensor_range);
    // d.sphere(s[ID]->state.pose.pos, s[ID]->state.pose.toAxisAngle(), sensor_range);

    // sensor animations
    // ranger.animate(d);
    
    multi_ranger.animate(d);
//   multi_ranger.getMeasurements(s[ID]->state.pose);
}

void formation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y){}

/*
Generate a velocity towards/awayfrom target position such that current distance tends to target distance

pos_t: Target position
pos: Current positon
gain: multiplying factor 
d_t: Target/desired distance
d_c: Current distance

*/ 

Eigen::Vector3f formation::calc_consensus_vel(Eigen::Vector3f pos_t, Eigen::Vector3f pos_c, double gain, float d_t, float d_c, std::string type){
    double w = gain * (d_c/d_t - 1); // weight depends on gain and relative distance
    Eigen::Vector3f ret = pos_t - pos_c; 
    if (type == "unidir") w = abs(w);
    return ret*w; 
}

Eigen::Vector3f formation::get_velocity_cmd(const uint16_t ID){
    // Get vector of all neighbors from closest to furthest
    std::vector<uint> closest = o.request_closest(ID);
    Eigen::Vector3f v_des({0.,0.,0.});

    for (uint j: closest){
        // if (j==ID) continue;
        // COHESION
        float other_p_x = s[j]->get_position(0, true);
        float other_p_y = s[j]->get_position(1, true);
        Eigen::Vector3f other_p({other_p_x, other_p_y, 0.});

        float this_p_x = s[ID]->get_position(0, true);
        float this_p_y = s[ID]->get_position(1, true);
        Eigen::Vector3f this_p({this_p_x, this_p_y, 0.});
        Eigen::Vector3f pij_c = other_p - this_p;
        float dij_c = pij_c.norm();

        // FORMATION
        Eigen::Vector3f pij_t({t.adjacency_mat(j, ID, 0),t.adjacency_mat(j, ID, 1), 0.});
        float dij_t = min_sep * t.adjacency_mat_mag(j, ID);
        Eigen::Vector3f v_form = calc_consensus_vel(pij_t, pij_c, -0.1, dij_t, dij_c);
        v_des += v_form;
    }
 
    // formation
    // ranger.getMeasurement(s[ID]->state.pose);
    multi_ranger.getMeasurements(s[ID]->state.pose);

    return v_des.normalized()*3;
}