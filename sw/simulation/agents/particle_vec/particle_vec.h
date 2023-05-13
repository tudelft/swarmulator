#ifndef PARTICLE_VEC_H
#define PARTICLE_VEC_H

#include "agent.h"
#include <vector>
#include "types.h"
#include "main.h"
/*
A particle that uses vectorised calculations. Allows extension to 3D
*/
// std::unique_ptr param;

class particle_vec: public Agent{
  public:
    /**
     * Constructor
     */
    particle_vec(int i, State state, float tstep);
    
    
    /**
     * State update implementation
     */
    State state_update(State state);

    /**
     * Animation openGL implementation for visualization
     */
    void animation();
    Eigen::MatrixXf tri_pts = param->scale()*Eigen::MatrixXf({{2,0,0}, {-1,1,0},{-1,-1,0}}); 
};

#endif
