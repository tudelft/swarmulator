#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "types.h"
class draw;

class ObstacleBase{
    public:
        Pose _pose; 
        Vector<float> size;
        Eigen::MatrixXf _points;
        Eigen::MatrixXf _points_t;
    public:
        ObstacleBase(){};
        ObstacleBase(Eigen::MatrixXf points, Pose pose):_pose(pose), _points(points){ _points_t = _pose.transform(points);};
        virtual void animate(draw d)=0;
        
        // for obstacles with single intersection
        virtual Eigen::Vector3f check_collision(Eigen::Vector3f p0, Eigen::Vector3f p1, int& check){return Eigen::Vector3f();};
        
        // for obstacles with multiple points of intersection
        virtual Eigen::MatrixXf check_collision(Eigen::Vector3f p0, Eigen::Vector3f p1){return Eigen::MatrixXf();};
        virtual Eigen::MatrixXf check_collision(Eigen::Matrix<float, Eigen::Dynamic, 3> points){return Eigen::MatrixXf();};
        
        // virtual std::vector<Eigen::MatrixXf> check_collisions(Eigen::Vector3f p0, Eigen::Vector3f p1, int* checks[]);
};

class Quad: public ObstacleBase{
    public:
        Quad(){};
        Quad(Eigen::Matrix<float, 4, 3> points, Pose pose):ObstacleBase(points, pose){};
        // Quad(Eigen::Vector3f p1,Eigen::Vector3f p2, Eigen::Vector3f p3, Eigen::Vector3f p4, Pose pose):ObstacleBase(points, pose){};
        void animate(draw d);

        Eigen::Vector3f check_collision(Eigen::Vector3f p0, Eigen::Vector3f p1, int& check);
        
};


class Cuboid: public ObstacleBase{
    private:
        std::vector<Quad*> _planes = std::vector<Quad*>(6);
    public:
        Vector<float> size;
        Cuboid(){};
        Cuboid(Vector<float> _size, Pose pose);
        
        // returns the point at which the line segment p0 p1 intersects with the cube
        // Eigen::VectorXf intersect(Eigen::VectorXf p0, Eigen::VectorXf p1);
        void animate(draw d);
        // Vector<Quad*> get_primitives(){return _planes;}
        Eigen::MatrixXf check_collision(Eigen::Vector3f p0, Eigen::Vector3f p1);
        Eigen::MatrixXf check_collision(Eigen::Matrix<float, Eigen::Dynamic, 3> points);
};

#endif