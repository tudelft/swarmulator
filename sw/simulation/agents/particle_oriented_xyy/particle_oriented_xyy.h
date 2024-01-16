#ifndef PARTICLE_ORIENTED_XYY_H
#define PARTICLE_ORIENTED_XYY_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

/**
 * This child class of agent implements the dynamics of simple accelerated oriented vehicles using a kinematic model
 */
class particle_oriented_xyy: public Agent
{
public:
  /**
   * Constructor
   */
  particle_oriented_xyy(int i, std::vector<float> state, float tstep);

  /**
   * State update implementation
   */
  std::vector<float> state_update(std::vector<float> s);

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*PARTICLE_ORIENTED_XYY_H*/