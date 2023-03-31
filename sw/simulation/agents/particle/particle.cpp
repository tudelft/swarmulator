#include "particle.h"
#include "trigonometry.h"
#include "draw.h"

particle::particle(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
  controller->set_saturation(1.0);
}

std::vector<float> particle::state_update(std::vector<float> s)
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float v_x = 0.0;
  float v_y = 0.0;
  controller->get_velocity_command(ID, v_x, v_y);
  controller->saturate(v_x);
  controller->saturate(v_y);
  moving = controller->moving;

  float vxr, vyr;
  rotate_l2g_xy(v_x, v_y, orientation, vxr, vyr);

  // Acceleration
  s.at(STATE_AX) = 2 * (vxr - s[STATE_VX]); // Acceleration x
  s.at(STATE_AY) = 2 * (vyr - s[STATE_VY]); // Acceleration y

  // Velocity
  s.at(STATE_VX) += s[STATE_AX] * dt; // Velocity x
  s.at(STATE_VY) += s[STATE_AY] * dt; // Velocity y

  // Position
  s.at(STATE_X) += s[STATE_VX] * dt + 0.5 * s[STATE_AX] * pow(dt, 2); // Position x
  s.at(STATE_Y) += s[STATE_VY] * dt + 0.5 * s[STATE_AY] * pow(dt, 2); // Position y

  return s;
};

void particle::animation()
{
  draw d;
  d.circle(param->scale());
}
