#include "ekf_state_estimator.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"

// Initilizer
ekf_state_estimator::ekf_state_estimator()
{
  initialized = false;
};

void ekf_state_estimator::init_ekf_filter()
{
  float pxf, pyf;
  polar2cart(o.request_distance(ID, ID_tracked), o.request_bearing(ID, ID_tracked), pxf, pyf);
  discrete_ekf_no_north_new(&ekf_rl);
  ekf_rl.X[0] = pxf;
  ekf_rl.X[1] = pyf;
  ekf_rl.X[8] = wrapToPi_f(agents[ID_tracked]->get_state(6) - agents[ID]->get_state(6));
  initialized = true;
  simtime_seconds_store = simtime_seconds;
}

void ekf_state_estimator::run_ekf_filter()
{
  // All in local frame of follower!!!! Values for position, velocity, acceleration
  float vxf, vyf, vx0f, vy0f, axf, ayf, ax0, ay0;
  rotate_g2l_xy(agents[ID]->get_state(STATE_VX), agents[ID]->get_state(STATE_VY), agents[ID]->get_state(STATE_YAW), vxf,  vyf);
  rotate_g2l_xy(agents[ID]->get_state(STATE_AX), agents[ID]->get_state(STATE_AY), agents[ID]->get_state(STATE_YAW), axf, ayf);
  rotate_g2l_xy(agents[ID_tracked]->get_state(STATE_VX), agents[ID_tracked]->get_state(STATE_VY), agents[ID_tracked]->get_state(STATE_YAW), vx0f, vy0f);
  rotate_g2l_xy(agents[ID_tracked]->get_state(STATE_AX), agents[ID_tracked]->get_state(STATE_AY), agents[ID_tracked]->get_state(STATE_YAW), ax0, ay0);
  ekf_rl.dt = simtime_seconds - simtime_seconds_store;
  simtime_seconds_store = simtime_seconds;
  float U[EKF_L] = {axf, ayf, ax0, ay0, agents[ID]->get_state(7), agents[ID_tracked]->get_state(7)};
  float Z[EKF_M] = {o.request_distance(ID, ID_tracked), 0.0, 0.0, vxf, vyf, vx0f, vy0f};
  discrete_ekf_no_north_predict(&ekf_rl, U);
  discrete_ekf_no_north_update(&ekf_rl, Z);
  ekf_rl.X[8] = wrapToPi_f(ekf_rl.X[8]);
}

void ekf_state_estimator::run(uint16_t ID_in, uint16_t ID_tracked_in)
{
  if (!initialized) {
    ID = ID_in;
    ID_tracked = ID_tracked_in;
    init_ekf_filter();
    printf("Launched EKF instance for %d to %d\n", ID, ID_tracked);
  } else {
    run_ekf_filter();
  }
}
