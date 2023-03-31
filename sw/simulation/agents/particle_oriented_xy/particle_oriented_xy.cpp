#include "particle_oriented_xy.h"
#include "trigonometry.h"
#include "randomgenerator.h"
#include "draw.h"
using namespace std;

particle_oriented_xy::particle_oriented_xy(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[STATE_YAW];
  controller->set_saturation(0.5);
  controller->init(ID);
  manual = false;
}

vector<float> particle_oriented_xy::state_update(vector<float> s)
{
  // NED frame
  // x+ towards North
  // y+ towards East
  float vx_des, vy_des = 0.;
  float vx_global, vy_global, dpsirate = 0.;
  if (!manual) {
    controller->get_velocity_command(ID, vx_des, vy_des); // Command comes out in the local frame
  } else {
    vx_des = manualx;
    vy_des = manualy;
    dpsirate = manualpsi_delta;
  }
  controller->saturate(vx_des);
  controller->saturate(vy_des);
#if COMMAND_LOCAL
  rotate_l2g_xy(vx_des, vy_des, s[STATE_YAW], vx_global, vy_global);
#else
  vx_global = vx_des;
  vy_global = vy_des;
#endif
  s.at(STATE_YAWRATE) = dpsirate;
  s.at(STATE_YAW) += s[STATE_YAWRATE] * dt;
  orientation = wrapToPi_f(s[STATE_YAW]);

  // Acceleration control
  float ka = 2;
  s.at(STATE_AX) = ka * (vx_global - s[STATE_VX]); // Acceleration global frame
  s.at(STATE_AY) = ka * (vy_global - s[STATE_VY]); // Acceleration global frame
  moving = controller->moving;

  // Velocity
  s.at(STATE_VX) += s[STATE_AX] * dt; // Velocity x global frame
  s.at(STATE_VY) += s[STATE_AY] * dt; // Velocity y global frame

  // Position
  s.at(STATE_X) += s[STATE_VX] * dt + 0.5 * s[STATE_AX] * pow(dt, 2); // Position x global frame
  s.at(STATE_Y) += s[STATE_VY] * dt + 0.5 * s[STATE_AY] * pow(dt, 2); // Position y global frame

  return s;
};

void particle_oriented_xy::animation()
{
  draw d;

  d.triangle(param->scale());
}
