#include "uwb_swarming.h"

#include <GL/freeglut.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "main.h"
#include "draw.h"
#include "auxiliary.h"
#include "uwb_channel.h"
#include "full_ekf.h"
#include "single_ekf.h"
// #include "swarm_storage.h"
#include "swarm_ranging.h"
#include "pid.h"
#include "rel_loc_estimator.h"

#define SENSOR_MAX_RANGE 1.8

#define RANGE_TO_NONE 0
#define RANGE_TO_CLOSEST 1
#define RANGE_TO_RANDOM 2
#define RANGE_TO_ALL 3

#define N_RANGING_TARGETS 2

#define RUN_EKF_ON_ALL_DRONES 1

#define DESIRED_REL_X 1.5f
#define DESIRED_REL_Y 0.8f
#define DESIRED_REL_PSI 0.0f

uwb_swarming::uwb_swarming() : Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
  _last_velocity_update = 0;
  _last_ping_tx_seconds = 0;
  _last_ekf_seconds = 0;
  _current_vx = 0;
  _current_vy = 0;
  _vel_cmd[0] = 0.0f;
  _vel_cmd[1] = 0.0f;
  _psirate_cmd = 0.0f;
  _avoiding_collision = false;

}

void uwb_swarming::init(const uint16_t ID)
{
  this->ID = ID;
  // _swarm.init(ID);
  _ranging.init(ID);

  if (RUN_EKF_ON_ALL_DRONES || ID == 0)
  {
    _p_ekf[ESTIMATOR_NONE] = NULL;
    _p_ekf[ESTIMATOR_EKF_REF] = new FullEKF(nagents - 1, this->ID, "Ref EKF");
    _p_ekf[ESTIMATOR_EKF_FULL] = new FullEKF(nagents - 1, this->ID, "Full EKF");
    _p_ekf[ESTIMATOR_EKF_SINGLE] = new SingleEKF(nagents - 1, this->ID, "Bank EKF");
    _p_ekf[ESTIMATOR_EKF_DYNAMIC] = new FullEKF(3, this->ID, "Dynamic EKF");    

    _ranging_mode = RANGE_TO_CLOSEST;
  }
  else
  {
    _ranging_mode = RANGE_TO_CLOSEST;
  }

  if (ID != 0)
  {
    _p_rel_pos_pid_x = new PID(0.5, 0.1, 0.00001);
    _p_rel_pos_pid_y = new PID(0.5, 0.1, 0.00001);
    _p_rel_pos_pid_psi = new PID(0.5, 0.1, 0.00001);
  }

  #ifdef LOG
    std::stringstream name;
    name << "log_drone_" << this->ID;
    _pFlogger = new FileLogger(name.str());
    std::stringstream header;
    header << "time"
           << ", ref_c1_mean, ref_c3_mean, ref_c3_max, ref_c5_mean, ref_c5_max, ref_icr_mean, ref_icr_max, ref_t_us" 
           << ", ful_c1_mean, ful_c3_mean, ful_c3_max, ful_c5_mean, ful_c5_max, ful_icr_mean, ful_icr_max, ful_t_us" 
           << ", bnk_c1_mean, bnk_c3_mean, bnk_c3_max, bnk_c5_mean, bnk_c5_max, bnk_icr_mean, bnk_icr_max, bnk_t_us" 
           << ", dyn_c1_mean, dyn_c3_mean, dyn_c3_max, dyn_c5_mean, dyn_c5_max, dyn_icr_mean, dyn_icr_max, dyn_t_us" 
           << std::endl;
    _pFlogger->write_data(header);
  #endif

}

void uwb_swarming::get_velocity_command(const uint16_t ID, float &v_x, float &psirate)
{
  state_estimation();

  if (simtime_seconds >= _last_cmd_seconds + 10 * CMD_INTERVAL)
  {
    _vel_cmd[0] = 0.5;
    _psirate_cmd = rg.uniform_float(-0.3 * M_PI, 0.3 * M_PI);
    _last_cmd_seconds = simtime_seconds;
  }

  v_x = _vel_cmd[0];
  psirate = _psirate_cmd;
  wall_avoidance_turn(ID, v_x, psirate, SENSOR_MAX_RANGE);
}
/*
void uwb_swarming::get_velocity_command(const uint16_t ID, float &v_x, float &v_y, float &dpsi)
{
  state_estimation();

  if (simtime_seconds < 60)
  {
    // initialization
    if (_avoiding_collision || simtime_seconds >= _last_cmd_seconds + 10 * CMD_INTERVAL)
    {
      _vel_cmd[0] = rg.uniform_float(-1, 1);
      _vel_cmd[1] = rg.uniform_float(-1, 1);
      _psirate_cmd = 0.0f;
      _last_cmd_seconds = simtime_seconds;
    }
  }
  else
  {
    // leader
    if (ID == 0 && (_avoiding_collision || simtime_seconds >= _last_cmd_seconds + 20 * CMD_INTERVAL))
    {
      _vel_cmd[0] = 0.5;
      _vel_cmd[1] = 0; // rg.uniform_float(-1, 1);
      _psirate_cmd = rg.uniform_float(-0.2 * M_PI, 0.2 * M_PI);
      _last_cmd_seconds = simtime_seconds;
    }

    // follower
    if (ID != 0 && (_avoiding_collision || simtime_seconds >= _last_cmd_seconds + CMD_INTERVAL))
    {
      // errors in leader frame
      float el_x, el_y;
      uint16_t st_id = _swarm.get_storage_index(0);
      rotate_l2g_xy(_swarm._agents[st_id].state.relX,
                    _swarm._agents[st_id].state.relY,
                    _swarm._agents[st_id].state.relPsi,
                    el_x, el_y);
      // desired position in follower frame
      float des_x, des_y;
      rotate_g2l_xy(DESIRED_REL_X, DESIRED_REL_Y,
                    _swarm._agents[st_id].state.relPsi, des_x, des_y);

      // errors in follower frame
      float e_psi = DESIRED_REL_PSI - _swarm._agents[st_id].state.relPsi;
      float e_x = des_x - _swarm._agents[st_id].state.relX;
      float e_y = des_y - _swarm._agents[st_id].state.relY;

      // commands (Proportional control for now)
      _vel_cmd[0] = _p_rel_pos_pid_x->step(e_x, CMD_INTERVAL);
      _vel_cmd[1] = _p_rel_pos_pid_y->step(e_y, CMD_INTERVAL);
      _psirate_cmd = _p_rel_pos_pid_psi->step(e_psi, CMD_INTERVAL);
      _last_cmd_seconds = simtime_seconds;
    }
  }

  v_x = _vel_cmd[0];
  v_y = _vel_cmd[1];
  dpsi = _psirate_cmd;
  // wall_avoidance_bounce(ID, v_x, v_y, SENSOR_MAX_RANGE);
  _avoiding_collision = wall_avoidance_xyy(ID, v_x, v_y, dpsi, SENSOR_MAX_RANGE);
}
*/
void uwb_swarming::get_own_input(ekf_input_t &own_input)
{
  float vx = agents[this->ID]->state[STATE_VX];
  float vy = agents[this->ID]->state[STATE_VY];
  float yaw = agents[this->ID]->state[STATE_YAW];
  rotate_g2l_xy(vx, vy, yaw, own_input.rhoX, own_input.rhoY);

  own_input.id = this->ID;
  own_input.rhoX += this->rg.gaussian_float(0, MEAS_NOISE_RHOX);
  own_input.rhoY += this->rg.gaussian_float(0, MEAS_NOISE_RHOY);
  own_input.dPsi = agents[this->ID]->state[STATE_YAWRATE] + this->rg.gaussian_float(0, MEAS_NOISE_DPSI);
}

void uwb_swarming::state_estimation()
{
  ekf_input_t own_input;
  get_own_input(own_input);

  if (simtime_seconds >= _last_ping_tx_seconds + BASE_RANGING_PING_INTERVAL)
  {
    _ranging.send_ranging_ping(own_input.rhoX, own_input.rhoY, own_input.dPsi);
    _last_ping_tx_seconds = simtime_seconds + rg.uniform_float(-BASE_RANGING_PING_INTERVAL / 10, BASE_RANGING_PING_INTERVAL / 10);
  }

  std::vector<ekf_range_measurement_t> measurements;
  std::vector<ekf_input_t> inputs;
  _ranging.receive_new_ranging(measurements, inputs);

  std::vector<ekf_range_measurement_t> all_measurements;
  std::vector<ekf_input_t> all_inputs;
  _ranging.receive_all_ranging(all_measurements, all_inputs);

  if (RUN_EKF_ON_ALL_DRONES || ID == 0)
  {
    for (uint8_t iEst = 0; iEst < ESTIMATOR_MAX; iEst++)
    {
      if (_p_ekf[iEst] == NULL)
      {
        continue;
      } else if (iEst == ESTIMATOR_EKF_REF){
        _p_ekf[ESTIMATOR_EKF_REF]->enqueue_inputs(all_inputs);
        _p_ekf[ESTIMATOR_EKF_REF]->enqueue_ranges(all_measurements);
        _p_ekf[ESTIMATOR_EKF_REF]->step(simtime_seconds, own_input);
      }
      else
      {
        _p_ekf[iEst]->enqueue_inputs(inputs);
        _p_ekf[iEst]->enqueue_ranges(measurements);
        _p_ekf[iEst]->step(simtime_seconds, own_input);
      }
    }
  }

  // Calculate performance indicators
  _ranging._evaluator.calculate_load();

  std::vector<uint16_t> ids_in_comm_range_ordered;
  std::vector<float> relX;
  std::vector<float> relY;
  std::vector<float> range;
  std::vector<float>::iterator it;
  uint16_t idx;
  float dx_g, dy_g, dx_l, dy_l, range_tmp;

  for (uint16_t iAgent = 0; iAgent<nagents; iAgent++){
    if (iAgent == this->ID){
      continue;
    } else { 
      dx_g = agents[iAgent]->state[STATE_X] - agents[this->ID]->state[STATE_X];
      dy_g = agents[iAgent]->state[STATE_Y] - agents[this->ID]->state[STATE_Y];
      
      range_tmp = sqrt(pow(dx_g, 2) + pow(dy_g,2));
      if (range_tmp <= COMMUNICATION_RANGE){
        it = std::upper_bound(range.begin(), range.end(), range_tmp);
        idx = it - range.begin(); // calculate this first, insert() can mess with begin()

        range.insert(it, range_tmp);
        ids_in_comm_range_ordered.insert(ids_in_comm_range_ordered.begin()+idx, iAgent);
        
        rotate_g2l_xy(dx_g, dy_g, agents[this->ID]->state[STATE_YAW], dx_l, dy_l);
        relX.insert(relX.begin()+idx, dx_l);
        relY.insert(relY.begin()+idx, dy_l);      
      }
    }
  }
  for (uint8_t iEst = 0; iEst < ESTIMATOR_MAX; iEst++){
    if (_p_ekf[iEst] == NULL)
      {
        continue;
      }
      else
      {
        _p_ekf[iEst]->update_performance(ids_in_comm_range_ordered, relX, relY);
      }
  }
  #ifdef LOG
    std::stringstream data;
    // data << simtime_seconds << std::endl; // << ", " << _other_id << ", "
    //      << dx_l << ", " << dy_l << ", " << dyaw << ", "
    //      << _state[EKF_ST_X] << ", " << _state[EKF_ST_Y] << ", " << _state[EKF_ST_PSI] << ", "
    //      << ex << ", " << ey << ", " << epsi << std::endl;
    data << simtime_seconds
          << std::fixed << std::setprecision(2) 
          << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.c1.mean << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.c3.mean << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.c3.max << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.c5.mean << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.c5.max << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.icr.mean << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.icr.max << "," << _p_ekf[ESTIMATOR_EKF_REF]->_performance.comp_time_us 
          << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.c1.mean << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.c3.mean << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.c3.max << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.c5.mean << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.c5.max << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.icr.mean << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.icr.max << "," << _p_ekf[ESTIMATOR_EKF_FULL]->_performance.comp_time_us 
          << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.c1.mean << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.c3.mean << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.c3.max << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.c5.mean << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.c5.max << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.icr.mean << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.icr.max << "," << _p_ekf[ESTIMATOR_EKF_SINGLE]->_performance.comp_time_us 
          << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.c1.mean << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.c3.mean << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.c3.max << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.c5.mean << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.c5.max << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.icr.mean << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.icr.max << "," << _p_ekf[ESTIMATOR_EKF_DYNAMIC]->_performance.comp_time_us 
          << std::endl;
    _pFlogger->write_data(data);
  #endif
}

void uwb_swarming::animation(const uint16_t ID)
{
  draw d;

  d.circle_loop(COMMUNICATION_RANGE);

  if (this->ID == 0)
  {
    // _p_ekf_ref->animate();
    // _p_ekf_single->animate();
  }
  // float dxg, dyg, dxl, dyl;

  // for (uint i=0; i<this->connectivity_vector.size(); i++){
  //   if (i<this->ID && this->connectivity_vector[i]==LINK_PING){
  //     dxg = agents[i]->state[STATE_X] - agents[ID]->state[STATE_X];
  //     dyg = agents[i]->state[STATE_Y] - agents[ID]->state[STATE_Y];
  //     rotate_g2l_xy(dxg, dyg, agents[ID]->state[STATE_YAW], dxl, dyl);
  //     // d.line(dxl, dyl, 1.5, orange); // orange
  //   }
  // }
}

void uwb_swarming::rel_loc_animation(const uint16_t agent_ID, const uint16_t estimator_ID)
{
  draw d;

  // draw self
  d.agent_raw(agent_ID, 0.0f, 0.0f, 0.0f);
  d.circle_loop(COMMUNICATION_RANGE);

  _ranging.rel_loc_animation(agent_ID);

  // // other agents in range, groundtruth
  float dx_g, dy_g, dx_l, dy_l;
  float d_yaw;
  float own_yaw = agents[agent_ID]->state[STATE_YAW];

  for (uint16_t i_agent = 0; i_agent < agents.size(); i_agent++)
  {
    if (i_agent == agent_ID)
    {
      continue;
    }

    // Draw groundtruth
    dx_g = agents[i_agent]->state[STATE_X] - agents[agent_ID]->state[STATE_X];
    dy_g = agents[i_agent]->state[STATE_Y] - agents[agent_ID]->state[STATE_Y];
    rotate_g2l_xy(dx_g, dy_g, own_yaw, dx_l, dy_l);
    d_yaw = agents[i_agent]->state[STATE_YAW] - agents[agent_ID]->state[STATE_YAW];

    d.agent_raw(i_agent, dx_l, dy_l, d_yaw);

    // Draw estimate
    // uint16_t storage_id = _swarm.get_storage_index(i_agent);
    // d.estimate(i_agent, _swarm._agents[storage_id].state.relX,
    //            _swarm._agents[storage_id].state.relY,
    //            _swarm._agents[storage_id].state.relPsi);
    // if(_p_ekf[estimator_ID] != NULL && _p_ekf[estimator_ID]->get_index(i_agent, storage_id)){
    //   d.estimate(i_agent, _p_ekf[estimator_ID]->_state[storage_id][EKF_ST_X],
    //                       _p_ekf[estimator_ID]->_state[storage_id][EKF_ST_Y],
    //                       _p_ekf[estimator_ID]->_state[storage_id][EKF_ST_PSI]);
    // }
    if(_p_ekf[estimator_ID] != NULL){
      _p_ekf[estimator_ID]->animate(i_agent);
    }
  }

  std::stringstream stream;
  stream << "Agent " << std::to_string(agent_ID);
  if (_p_ekf[estimator_ID] != NULL)
  {
    stream << "\nEstimator " << _p_ekf[estimator_ID]->_name;
    stream << " - Mean Error:";
    stream << " C1=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c1.mean;
    stream << " C3=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c3.mean;
    stream << " C5=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c5.mean;
    stream << " ICR=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.icr.mean;
    stream << " time (us): " << _p_ekf[estimator_ID]->_performance.comp_time_us;
    // d.info_text("Agent " + std::to_string(agent_ID) +
    //             "\nEstimator " + _p_ekf[estimator_ID]->_name +
    //             " - Mean Error: N/A" +
    //             "\nLocal UWB Datarate: " + std::to_string((int)(100 * _ranging._evaluator._current_load)) + "% (" + std::to_string((int)(100 * _ranging._evaluator._avg_load)) + "%)");
  }
  else
  {
    stream << "\nEstimator None";
    stream << " - Mean Error: N/A";
    // d.info_text("Agent " + std::to_string(agent_ID) +
    //             "\nEstimator None" + 
    //             " - Mean Error: N/A" +
    //             "\nLocal UWB Datarate: " + std::to_string((int)(100 * _ranging._evaluator._current_load)) + "% (" + std::to_string((int)(100 * _ranging._evaluator._avg_load)) + "%)");
  }
  stream << "\nLocal UWB Datarate: "; 
  stream << std::to_string((int)(100 * _ranging._evaluator._current_load)) << "%";
  stream << "(" << std::to_string((int)(100 * _ranging._evaluator._avg_load)) << "%)";
  d.info_text(stream.str());
}