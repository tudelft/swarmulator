#ifndef PARTICLE_VEC_H
#define PARTICLE_VEC_H

#include "agent.h"
#include <vector>
#include "types.h"

/*
A particle that uses vectorised calculations. Allows extension to 3D
*/

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
};

#endif
