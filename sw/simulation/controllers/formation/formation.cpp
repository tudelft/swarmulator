#include "formation.h"
#include "main.h"
#include <vector>
#include "draw.h"
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
    formation_gain = 10;
}

void formation::animation(const uint16_t ID)
{
    draw d;
    // d.circle_loop(sensor_range);
}

Eigen::Vector3f formation::get_velocity_cmd(const uint16_t ID){
    // Get vector of all neighbors from closest to furthest
    std::vector<uint> closest = o.request_closest(ID);
    Eigen::Vector3f v_form({0.,0.,0.});

    for (uint j: closest){
        // if (j==ID) continue;
        // COHESION
        // float other_p_x = s[j]->get_position(0, true);
        // float other_p_y = s[j]->get_position(1, true);
        Eigen::Vector3f other_p = s[j]->get_position(true);


        // float this_p_x = s[ID]->get_position(0, true);
        // float this_p_y = s[ID]->get_position(1, true);
        Eigen::Vector3f this_p = s[ID]->get_position(true);

        Eigen::Vector3f pij_c = other_p - this_p;
        float dij_c = pij_c.norm();

        // FORMATION
        Eigen::Vector3f pij_t({t.adjacency_mat(j, ID, 0),t.adjacency_mat(j, ID, 1), 0.});
        float dij_t = min_sep * t.adjacency_mat_mag(j, ID);

        // start the nonlinear curve in a 10m region around the target
        // v_form += nonlin_idx_max(other_p, this_p, 6, dij_t+10, dij_t, dij_c,0.312);
        v_form += vel_transfer(other_p, this_p, 6, dij_t+10, dij_t, 0.3,0.3);
        // v_des += v_form;
    }
 
   return v_form;
    // print(v_coll);
    // v_des += v_coll;

    // return v_des.normalized()*3;
}