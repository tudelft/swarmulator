#include "leader_follower.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"

#define STATE_ESTIMATOR 0

#define NDI_MOST_RECENT ndihandle.data_entries - 1

// Delay of trajectory with respect to the leader
#ifndef NDI_DELAY
#define NDI_DELAY 4
#endif

/** Select method
 Method 0: First order approximation, no acceleration or yaw rate used
 Method 1: First order approximation, acceleration and yaw rate used, but yaw rate not taken into account in integral
*/
#define NDI_METHOD 1

// Initilizer
leader_follower::leader_follower() : Controller()
{
  ndihandle.delay = NDI_DELAY;
  ndihandle.tau_x = 3.0;
  ndihandle.tau_y = 3.0;
  ndihandle.wn_x = 0.9;
  ndihandle.wn_y = 0.9;
  ndihandle.eps_x = 0.28;
  ndihandle.eps_y = 0.28;
  ndihandle.Kp = -1.5;
  ndihandle.Ki = 0.0;
  ndihandle.Kd = -3.0;
};
void leader_follower::bindNorm(float max_command)
{
  float normcom = sqrt(ndihandle.commands[1] * ndihandle.commands[1] + ndihandle.commands[0] * ndihandle.commands[0]);
  if (normcom > max_command) {
    ndihandle.commands_lim[0] = ndihandle.commands[0] * max_command / normcom;
    ndihandle.commands_lim[1] = ndihandle.commands[1] * max_command / normcom;
  } else {
    ndihandle.commands_lim[0] = ndihandle.commands[0];
    ndihandle.commands_lim[1] = ndihandle.commands[1];
  }
}
float leader_follower::accessCircularFloatArrElement(float arr[], int index)
{
  float value;
  int realindex = (ndihandle.data_start + index) % NDI_PAST_VALS;
  value = arr[realindex];
  return value;
}

// TODO: Trapezoidal integration, and not simply discarding values before tcur-delay, but linearly interpolating to tcur-delay
float leader_follower::computeNdiFloatIntegral(float ndiarr[], float curtime)
{
  float integral = 0;
  float dt;
  for (int i = 0; i < ndihandle.data_entries - 1; i++) {
    dt = accessCircularFloatArrElement(ndihandle.tarr, i + 1) - accessCircularFloatArrElement(ndihandle.tarr, i);
    integral += dt * accessCircularFloatArrElement(ndiarr, i);
  }
  dt = curtime - accessCircularFloatArrElement(ndihandle.tarr, NDI_MOST_RECENT);
  integral += dt * accessCircularFloatArrElement(ndiarr, NDI_MOST_RECENT);
  return integral;
}

void leader_follower::cleanNdiValues(float tcur)
{
  int curentries = ndihandle.data_entries;
  for (int i = 0; i < curentries; i++) {
    if ((tcur - ndihandle.tarr[ndihandle.data_start]) > ndihandle.delay) {
      ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
      ndihandle.data_entries--;
    }
  }
  return;
}

/**
  * Update the NDI controller periodically
  */
void leader_follower::uwb_follower_control_periodic(void)
{
  // Re-initialize commands
  ndihandle.commands[0] = 0;
  ndihandle.commands[1] = 0;

  // Get current values
  float curtime = simtime_seconds;
  cleanNdiValues(curtime);
  if (ndihandle.data_entries > 0) {
    float oldx = accessCircularFloatArrElement(ndihandle.xarr, 0);
    float oldy = accessCircularFloatArrElement(ndihandle.yarr, 0);
    float newu1 = accessCircularFloatArrElement(ndihandle.u1arr, NDI_MOST_RECENT);
    float newv1 = accessCircularFloatArrElement(ndihandle.v1arr, NDI_MOST_RECENT);
    float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr, 0);
    float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr, 0);
    oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr, curtime);
    oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr, curtime);

    float Minv[2][2];
    MAKE_MATRIX_PTR(_MINV, Minv, 2);
    float_mat_zero(_MINV, 2, 2); //fmat_make_zeros(Minv, 2, 2);
    Minv[0][0] = -ndihandle.tau_x;
    Minv[1][1] = -ndihandle.tau_y;
    float l[2], oldxed, oldyed;

#if (NDI_METHOD == 0)
    l[0] = newu1 / ndihandle.tau_x;
    l[1] = newv1 / ndihandle.tau_y;
    oldxed = oldu2 - newu1;
    oldyed = oldv2 - newv1;
#elif (NDI_METHOD == 1)
    float newr1 = accessCircularFloatArrElement(ndihandle.r1arr, NDI_MOST_RECENT);
    float oldax2 = accessCircularFloatArrElement(ndihandle.ax2arr, 0);
    float olday2 = accessCircularFloatArrElement(ndihandle.ay2arr, 0);
    l[0] = (newu1 - newr1 * newr1 * oldx - newr1 * ndihandle.tau_x * newv1 + oldax2 * ndihandle.tau_x + 2 * newr1 *
            ndihandle.tau_x * oldv2) / ndihandle.tau_x;
    l[1] = (newv1 - newr1 * newr1 * oldy + newr1 * ndihandle.tau_y * newu1 + olday2 * ndihandle.tau_y - 2 * newr1 *
            ndihandle.tau_y * oldu2) / ndihandle.tau_y;
    oldxed = oldu2 - newu1 + newr1 * oldy;
    oldyed = oldv2 - newv1 - newr1 * oldx;
#endif

    float v[2];
    v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
    v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

    float sig[2];
    sig[0] = v[0] - l[0];
    sig[1] = v[1] - l[1];

    float_mat_vect_mul(ndihandle.commands, _MINV, sig, 2, 2);
  }
}

void leader_follower::get_velocity_command(const uint16_t ID, float &vx_des, float &vy_des)
{
  // Store data from leader's position estimate
  if (ndihandle.data_entries == NDI_PAST_VALS) {
    ndihandle.data_entries--;
    ndihandle.data_start = (ndihandle.data_start + 1) % NDI_PAST_VALS;
  }

  float px, py, vx, vy, vx0, vy0, ax0, ay0;
  // float px_true, py_true, vx_true, vy_true, vx0_true, vy0_true, ax0_true, ay0_true;
  if (ID > 0 && simtime_seconds > 10) {
    uint8_t ID_tracked = 0; //ID - 1;
#if COMMAND_LOCAL
#if STATE_ESTIMATOR
    filter.run(ID, ID_tracked);
    px = filter.ekf_rl.X[0];
    py = filter.ekf_rl.X[1];
    vx = filter.ekf_rl.X[4];
    vy = filter.ekf_rl.X[5];
    rotate_l2g_xy(filter.ekf_rl.X[6], filter.ekf_rl.X[7], filter.ekf_rl.X[8], vx0, vy0);
#else
    polar2cart(o.request_distance(ID, ID_tracked), o.request_bearing(ID, ID_tracked), px, py);
    rotate_g2l_xy(agents[ID]->get_state(STATE_VX), agents[ID]->get_state(STATE_VY), agents[ID]->get_state(STATE_YAW), vx, vy);
    rotate_g2l_xy(agents[ID_tracked]->get_state(STATE_VX), agents[ID_tracked]->get_state(STATE_VY), agents[ID]->get_state(STATE_YAW), vx0, vy0);
    rotate_g2l_xy(agents[ID_tracked]->get_state(STATE_AX), agents[ID_tracked]->get_state(STATE_AY), agents[ID]->get_state(STATE_YAW), ax0, ay0);
#endif
    rotate_g2l_xy(agents[ID_tracked]->get_state(STATE_AX), agents[ID_tracked]->get_state(STATE_AY), agents[ID]->get_state(STATE_YAW), ax0, ay0);
    ndihandle.xarr[ndihandle.data_end] = px;
    ndihandle.yarr[ndihandle.data_end] = py;
    ndihandle.u1arr[ndihandle.data_end] = vx;
    ndihandle.v1arr[ndihandle.data_end] = vy;
    ndihandle.u2arr[ndihandle.data_end] = vx0;
    ndihandle.v2arr[ndihandle.data_end] = vy0;
    ndihandle.r1arr[ndihandle.data_end] = agents[ID]->get_state(7);
    ndihandle.ax2arr[ndihandle.data_end] = ax0;
    ndihandle.ay2arr[ndihandle.data_end] = ay0;
#else
    ndihandle.xarr[ndihandle.data_end] = o.request_distance_dim(ID, ID_tracked, 0);
    ndihandle.yarr[ndihandle.data_end] = o.request_distance_dim(ID, ID_tracked, 1);
    ndihandle.u1arr[ndihandle.data_end] = agents[ID]->get_state(2);
    ndihandle.v1arr[ndihandle.data_end] = agents[ID]->get_state(3);
    ndihandle.u2arr[ndihandle.data_end] = agents[0]->get_state(2);
    ndihandle.v2arr[ndihandle.data_end] = agents[0]->get_state(3);
    ndihandle.r1arr[ndihandle.data_end] = agents[ID]->get_state(7);
    ndihandle.ax2arr[ndihandle.data_end] = agents[0]->get_state(4);
    ndihandle.ay2arr[ndihandle.data_end] = agents[0]->get_state(5);
#endif
    ndihandle.tarr[ndihandle.data_end] = simtime_seconds;
    ndihandle.data_end = (ndihandle.data_end + 1) % NDI_PAST_VALS;
    ndihandle.data_entries++;
    uwb_follower_control_periodic();
    bindNorm(0.1);
    vx_des = ndihandle.commands_lim[0];
    vy_des = ndihandle.commands_lim[1];
  }
}

void leader_follower::animation(const uint16_t ID)
{

}