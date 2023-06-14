#include "target_follow.h"
#include "draw.h"

target_follow::target_follow(std::vector<Eigen::Vector3f> waypoints){
    target_follow_gain = 5;
    wp_radius = 0.5;
    wp_counter = 0;
    wp_list = waypoints;
    // current_wp = wp_list[wp_counter];
    start = true;

}   

Eigen::Vector3f target_follow::get_velocity_cmd(const uint16_t ID){
    
    centroid = o.get_centroid(); // update centroid

 // check if waypoint has been reached
    if ((dist_to_wp < wp_radius) || start){
        
        current_wp = wp_list[wp_counter % wp_list.size()];
        dist_to_wp_max = (current_wp - centroid).norm();
        wp_counter += 1;
        start = false;
    }
        
    dist_to_wp = (current_wp - centroid).norm();  // update distance to waypoint
    // print(dist_to_wp);
    
    
    // Eigen::Vector3f v_follow = nonlin_idx_max(current_wp, centroid, target_follow_gain, dist_to_wp_max, wp_radius, dist_to_wp,0.3);
    // Eigen::Vector3f v_follow = prop(current_wp, centroid, target_follow_gain, wp_radius, dist_to_wp);
    Eigen::Vector3f v_follow = vel_transfer(current_wp, centroid, 6, dist_to_wp_max, 0, wp_radius,0.3);

    // print("follow speed = ", v_follow.norm());
    return v_follow;
}

bool target_follow::current_wp_reached(){
    return (dist_to_wp<wp_radius)? true: false;
}

void target_follow::animation(const uint16_t ID)
{
    draw d;
    d.points(wp_list);
        
}