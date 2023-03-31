#ifndef AGENT_THREAD_H
#define AGENT_THREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include <chrono>

#include "settings.h"
#include "randomgenerator.h"
#include "environment.h"
#include "terminalinfo.h"
#include "auxiliary.h"

// Include relevant h file for the agent.
// This path is defined at build time depending on the chosen agent.
// No need to change this.
#include AGENT_INCLUDE

/**
 * Update the agent simulation
 * @param ID The ID of the agent/robot
 */
void run_agent_simulation_step(const int &ID)
{
  while (program_running) {

    bool ready = (agents.size() == nagents || simtime_seconds > 0.);
    if (ready) {
      main_mutex.lock_shared();
      std::vector<float> s_0 = agents.at(ID)->state;
      std::vector<float> s_n = agents.at(ID)->state_update(s_0); // State update
      main_mutex.unlock_shared();

      /****** Wall physics engine ********/
      // The dynamics are updated unless the robot crashes into a wall, at which point the velocity and acceleration are set to 0.
      // The crash into a wall is detected by checking whether the robot is near a wall and the velocity vector intersects with the wall.

      // Check if hitting a wall
      std::vector<float> test = s_n; // Store a copy of the state update
      float r_temp, ang_temp, vx_temp, vy_temp;

      // Get direction of velocity vector
      cart2polar(s_n[STATE_VX], s_n[STATE_VY], r_temp, ang_temp);

      // Detect a potential crash by projecting the position forward based on the velocity, based on a simple test of whether the collision could happen within one second, provided the current velocity is kept.
      // Here we just project the velocity.
      polar2cart(r_temp, ang_temp, vx_temp, vy_temp);
      test[0] += vx_temp;
      test[1] += vy_temp;

      // Check if hitting the wall.
      // Update or kill the dynamics in case of collision.
      // The environment.check_for_collision function checks whether the vector s0->test intersects with any walls.
      if (!environment.check_for_collision(ID, s_0, test, ang_temp)) {
        // No wall collision --> Update the dynamics
        main_mutex.lock(); // Sync
        agents.at(ID)->state = s_n; // Update
        main_mutex.unlock();
      }

      else { // Wall! --> Kill the dynamics!
        main_mutex.lock(); // Sync
        agents.at(ID)->state[STATE_VX] = 0.0; // v_x = 0
        agents.at(ID)->state[STATE_VY] = 0.0; // v_y = 0
        agents.at(ID)->state[STATE_AX] = 0.0; // a_x = 0
        agents.at(ID)->state[STATE_AY] = 0.0; // a_y = 0
        agents.at(ID)->controller->moving = false; // Not moving
        main_mutex.unlock();
      }

      // Update the global clock.
      // The clock of the 0th robot is used as the reference for the global clock.
      if (ID == 0) {
        simtime_seconds += 1. / param->simulation_updatefreq();
        environment.loop();
      }


      // Run the local clock
      // If param->simulation_realtimefactor()=0, which can be set in conf/parameters.xml, the thread will not sleep and just run at full speed.
      if (param->simulation_realtimefactor() > 0) {
        int t_wait = (int)1e6 / (param->simulation_updatefreq() * param->simulation_realtimefactor());
        std::this_thread::sleep_for(std::chrono::microseconds(t_wait));
      }
    }
  }
}

/**
 * Generates new agent + simulation thread at given position x0 y0
 *
 * @param ID The ID of the agent/robot
 * @param x Initial position of the agent in x
 * @param y Initial position of the agent in y
 */
void create_new_agent(const int &ID, const std::vector<float> &states)
{
  // Initiate a new agent. The define "AGENT" is defined at build time.
  // No need to change this part of the code if you want to use a different agent, just compile swarmulator with:
  // >> make AGENT=myawesomeagent
  // By default, AGENT=particle
  // See wiki for a detailed explanation of how to create your own agent.
  main_mutex.lock();
  agents.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  main_mutex.unlock();

  // Info message
  std::stringstream ss;
  ss << "Robot " << ID << " initiated";
  terminalinfo::info_msg(ss.str());

  // Initiate the thread that controls the agent
  std::thread agent(run_agent_simulation_step, ID);

  // Detach thread so that it runs independently
  agent.detach();
}
#endif /*AGENT_THREAD_H*/
