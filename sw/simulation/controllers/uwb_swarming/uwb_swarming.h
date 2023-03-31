#ifndef UWB_SWARMING_H
#define UWB_SWARMING_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "randomgenerator.h"
#include "uwb_channel.h"

#define COMMAND_LOCAL 1

#define LINK_NONE 0
#define LINK_PING 1
#define LINK_RANGE 2

struct ekf_input {
  float rhoX;
  float rhoY;
  float dPsi;
};

/**
 * Basic exploration behavior which randomly moves in an environment while avoiding neighbors.
 *
 */
class uwb_swarming: public Controller
{
private:
  ekf_input own_input;
  std::vector<uint8_t> connectivity_vector;
  void communicate();
  void estimate_rel_pos();
public:
  /**
   * @brief Construct a new uwb_swarming object
   *
   */
  uwb_swarming();

  void init(const uint16_t ID);
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);

};

#endif /*EXPLORATION_H*/
