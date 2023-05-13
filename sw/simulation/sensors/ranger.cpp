#include "ranger.h"
#include "draw.h"

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
        // for (Eigen::MatrixXf intersection_set: intersections){            
        //     d.points(parent_pose.inv_transform(intersection_set.transpose()));
        // }
    }
    
}


/*
Check lasers for this ranger with all obstacles and return the single min distance 
*/
float Ranger::getMeasurement(Pose pose){

    // dynamic pose update
    parent_pose = pose; // animation uses this so update it here
    _lasers_w = parent_pose.transform(_lasers_b);
    
    // intersections.clear(); // reset the vector

    float min_dist = 10000;
    Eigen::Vector3f min_pt(0,0,0);
    Eigen::Index min_id;
    
    // for each laser
    for (int i=1; i<=_num_lasers;i++){
        float min_dist_obs = 10000;
        Eigen::Vector3f min_pt_obs(0,0,0);
        Eigen::Index min_id_obs;
        // for each obstacle in environment
        for (ObstacleBase* obstacle: environment.obstacles){
            
            Eigen::MatrixXf intersection_obs = obstacle->check_collision(_lasers_w.row(0), _lasers_w.row(i));
            // Eigen::MatrixXf intersection_obs = obstacle->check_collision(_lasers_w);

            if (intersection_obs.rows()!=0){
                
                float dist = (intersection_obs.rowwise() - pose.pos.transpose()).rowwise().squaredNorm().minCoeff(&min_id_obs);
                if (dist<min_dist_obs){
                    
                    min_pt_obs = intersection_obs.row(min_id_obs);
                    min_dist_obs = dist;
                    };
            }
        }
        // print(min_dist_obs);
        // if (min_pt.sum()!=0)
        // intersections.push_back(min_pt); 
        if (min_dist_obs < min_dist){
            min_dist = min_dist_obs;
            min_pt = min_pt_obs;
        }
    }
    intersection = min_pt;
    
    return min_dist;
}