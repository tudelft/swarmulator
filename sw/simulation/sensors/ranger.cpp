#include "ranger.h"
#include "draw.h"

Ranger::Ranger(Pose pose):self_pose(pose){
    // The raw template for this laser sensor
    _lasers.row(0) << 0,0,0; // apex of pyramid
    _lasers.row(1) << -_range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
    _lasers.row(2) << _range*std::tan(_fov/2), _range*std::tan(_fov/2),_range;
    _lasers.row(3) << _range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
    _lasers.row(4) << -_range*std::tan(_fov/2), -_range*std::tan(_fov/2),_range;
    
    // pose of this particular ranger wrt the agent
    _lasers_b =  pose.transform(_lasers);
}

void Ranger::animate(draw d){
    Eigen::Vector3f p0 = _lasers_b.row(0);
    Eigen::Vector3f p1 = _lasers_b.row(1);
    Eigen::Vector3f p2 = _lasers_b.row(2); 
    Eigen::Vector3f p3 = _lasers_b.row(3);
    Eigen::Vector3f p4 = _lasers_b.row(4);
    
    d.line(p0, p1);
    d.line(p0, p2);
    d.line(p0, p3);
    d.line(p0, p4);
    d.rect(p1, p2, p3, p4);
    if (abs(intersection(0))>=1e-10){ // just a way to check if the vector is not garbage
        d.points(parent_pose.inv_transform(intersection.transpose()));
    }   
}


/*
Check lasers for this ranger with all obstacles and return the single min distance 
*/
float Ranger::getMeasurement(Pose pose){

    // dynamic pose update
    parent_pose = pose; // animation uses this so update it here
    _lasers_w = parent_pose.transform(_lasers_b);

    float min_dist = 10000;
    Eigen::Vector3f min_pt(0,0,0);
    
    // for each laser
    for (int i=1; i<=_lasers_w.rows()-1; i++){
        float min_dist_obs = 10000;
        Eigen::Vector3f min_pt_obs(0,0,0);
        Eigen::Index min_id_obs;
        // for each obstacle in environment
        for (ObstacleBase* obstacle: environment.obstacles){
            Eigen::MatrixXf intersection_obs = obstacle->check_collision(_lasers_w.row(0), _lasers_w.row(i));
            if (intersection_obs.rows()!=0){
                // find the closest point among each obsacle for this laser        
                float dist = (intersection_obs.rowwise() - pose.pos.transpose()).rowwise().squaredNorm().minCoeff(&min_id_obs);
                if (dist<min_dist_obs){
                    min_pt_obs = intersection_obs.row(min_id_obs);
                    min_dist_obs = dist;
                }
            }
        }
        // find the closest point among each laser
        if (min_dist_obs < min_dist){
            min_dist = min_dist_obs;
            min_pt = min_pt_obs;
        }
    }
    intersection = min_pt;
    
    return min_dist;
}