#include "wheeled.h"
#include "draw.h"
#include "trigonometry.h"

using namespace std;

wheeled::wheeled(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[STATE_YAW];
}

vector<float> wheeled::state_update(vector<float> state)
{
  float leftwheelspeed, rightwheelspeed;

  controller->get_velocity_command(ID, leftwheelspeed, rightwheelspeed);
  controller->saturate(leftwheelspeed);
  controller->saturate(rightwheelspeed);
  moving = controller->moving;

  // Model
  float v_x = r / 2 * leftwheelspeed + r / 2 * rightwheelspeed;
  float v_y = 0.0;
  float psi_rate = - r / L * leftwheelspeed + r / L * rightwheelspeed;

  // Orientation
  state.at(STATE_YAWRATE) = psi_rate; // Orientation rate
  state.at(STATE_YAW) += state.at(STATE_YAWRATE); // Orientation
  orientation = state.at(6);

  // Velocity
  float vxr, vyr;
  rotate_l2g_xy(v_x, v_y, orientation, vxr, vyr); // Local frame to global frame
  state.at(STATE_VX) = vxr; // Velocity x
  state.at(STATE_VY) = vyr; // Velocity y

  // Position
  state.at(STATE_X) += state[STATE_VX] * dt; // Position x
  state.at(STATE_Y) += state[STATE_VY] * dt; // Position y

  return state;
}


void wheeled::animation()
{
  draw d;
  d.triangle(param->scale());
}
