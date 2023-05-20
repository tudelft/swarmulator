#include "obstacles.h"
#include "draw.h"


void Quad::animate(draw d){
    d.polyline(_points_t);
}

Eigen::Vector3f Quad::check_collision(Eigen::Vector3f p0,Eigen::Vector3f p1, int& check){
    /*
    from: https://www.mathworks.com/matlabcentral/fileexchange/17751-straight-line-and-plane-intersection?w.mathworks.com
    */
    std::vector<Eigen::Vector3f> collisions;

    Eigen::Vector3f A = _points_t.row(0);
    Eigen::Vector3f B = _points_t.row(1);
    Eigen::Vector3f C = _points_t.row(3);

    Eigen::Vector3f n =  Eigen::Vector3f(A-B).cross(Eigen::Vector3f(A-C));
    Eigen::Vector3f n_cap = n/n.norm();
    Eigen::Vector3f u = p1-p0;
    Eigen::Vector3f w = p0 -  Eigen::Vector3f(_points_t.row(0));
    float D = n_cap.dot(u);
    float N = -n_cap.dot(w);
    if (abs(D)<10e-7){
        if (N==0) check = 0; // segment is in plane
        else check = 1; // no intersection
    }
    
    float si = N/D;
    Eigen::Vector3f intersection = p0 + si*u;
    
//     Eigen::Matrix4f A;
//     Eigen::Matrix4f B;

//     Eigen::Vector3f x1 = _points_t.row(0);
//     Eigen::Vector3f x2 = ;
//     Eigen::Vector3f x3({1,1,1});
//     Eigen::Vector3f x4({0,0.6,0});
//     Eigen::Vector3f x5({-0.4,0,0.4});
    
//     A << 1,1,1,1,
//         x1[0],x2[0],x3[0],x4[0],
//         x1[1],x2[1],x3[1],x4[1],
//         x1[2],x3[2],x3[2],x4[2];
        
//     B << 1,1,1,0,
//         x1[0],x2[0],x3[0],x5[0]-x4[0],
//         x1[1],x2[1],x3[1],x5[1]-x4[1],
//         x1[2],x2[2],x3[2],x5[2]-x4[2];

//   float t = -A.determinant()/B.determinant();
//   Eigen::Vector3f res = x4 + (x5-x4)*t;

    if (si<0 or si>1) check = 1; // intersection point lies outside the line segment (no intersection)
    else check = 2; // unique intersection point;

    // check if intersection lies within the quad face. Assumes that the quadrilateral is a parallelogram for now
    Eigen::Vector3f a = intersection - A; 
    float b = a.dot((B-A).normalized()); // projection on AB
    float c = a.dot((C-A).normalized()); // projection on AC
    
    // if these projections are not within range, set the check to 1
    if (((0 < b) && b < (B-A).norm() && 
         (0 < c) && c < (C-A).norm()) == false){
        check = 1;
    }
    return intersection;
}


/*
Checks all faces of the cuboid for intersections made by the line p0 -- p1

Returns: matrix with rows as intersection points
*/
Eigen::MatrixXf Cuboid::check_collision(Eigen::Vector3f p0, Eigen::Vector3f p1){
    Eigen::Matrix<float, 6,3> collisions; // tbh there might only be 2 intersections at max (for a cuboid)
    int i = 0, j = 0;
    for (Quad* plane: _planes){
        int check;
        Eigen::Vector3f intersection = plane->check_collision(p0, p1, check);
        if (check == 2){
            collisions.row(j) = intersection;
            j++;
        }
        i++;
    }
    return collisions(Eigen::seq(0,j-1), Eigen::all);;
}

/*
Checks all faces of the cuboid for intersections with all the lines starting from the 
first row in points(arg) to all other rows as points(arg)

Returns a matrix with all intersections

*/
Eigen::MatrixXf Cuboid::check_collision(Eigen::Matrix<float, Eigen::Dynamic, 3> points){
    std::vector<Eigen::MatrixXf> collisions_vec;    

    for (int i=1; i<points.rows(); i++){
        Eigen::MatrixXf collisions_line = check_collision(points.row(0), points.row(i));
        if (collisions_line.rows()!=0) {
            collisions_vec.push_back(collisions_line);
        };
    }

    Eigen::MatrixXf collisions;
    for (Eigen::MatrixXf collisions_line: collisions_vec){
        int m = collisions.rows();
        int n = collisions_line.rows();
        collisions.conservativeResize(m + n, 3);
        collisions.block(m, 0, n, 3) = collisions_line;
    }

    return collisions;
}


Cuboid::Cuboid(Vector<float> size, Pose pose):_bbox(size){
    float l = size[0];
    float b = size[1];
    float h = size[2];

    std::initializer_list<float> p1 = {-l/2, -b/2, -h/2};
    std::initializer_list<float> p2 = {-l/2, -b/2, h/2};
    std::initializer_list<float> p3 = {-l/2, b/2, h/2};
    std::initializer_list<float> p4 = {-l/2, b/2, -h/2};
    std::initializer_list<float> p5 = {l/2, -b/2, -h/2};
    std::initializer_list<float> p6 = {l/2, -b/2, h/2};
    std::initializer_list<float> p7 = {l/2, b/2, h/2};
    std::initializer_list<float> p8 = {l/2, b/2, -h/2};

    // vertices should be ordered 
    _planes[0] = new Quad(Eigen::Matrix<float, 4, 3>({p1, p2, p3, p4}) , pose); // back
    _planes[1] = new Quad(Eigen::Matrix<float, 4, 3>({p1, p2, p6, p5}) , pose); // left
    _planes[2] = new Quad(Eigen::Matrix<float, 4, 3>({p3, p4, p8, p7}) , pose); // right
    _planes[3] = new Quad(Eigen::Matrix<float, 4, 3>({p1, p4, p8, p5}) , pose); // bottom
    _planes[4] = new Quad(Eigen::Matrix<float, 4, 3>({p2, p3, p7, p6}) , pose); // top
    _planes[5] = new Quad(Eigen::Matrix<float, 4, 3>({p5, p6, p7, p8}) , pose); // front

}

void Cuboid::animate(draw d){
    for (uint i = 0;i< _planes.size(); i++){
        _planes[i]->animate(d);
    }
}