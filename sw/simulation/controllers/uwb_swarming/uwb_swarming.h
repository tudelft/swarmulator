#ifndef UWB_SWARMING_H
#define UWB_SWARMING_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include "controller.h"
#include "randomgenerator.h"
#include "uwb_channel.h"
// #include "swarm_storage.h"
#include "full_ekf.h"
#include "single_ekf.h"
#include "pid.h"

#include "terminalinfo.h"


#define COMMAND_LOCAL 1
#define RANGING_TIMEOUT_TICKS 100
#define BASE_RANGING_PING_INTERVAL 0.01f
#define CMD_INTERVAL 0.1f

#define ANIMATION_TIMEOUT 0.5f
#define INFO_TEXT_UPDATE 1


/**
 * Basic exploration behavior which randomly moves in an environment while avoiding neighbors.
 *
 */
class uwb_swarming: public Controller
{
private:
  terminalinfo console_print;
  uint8_t _ranging_mode;
  float _last_velocity_update;
  float _current_vx;
  float _current_vy;
  bool _avoiding_collision;
  float _last_ping_tx_seconds;
  float _last_ekf_seconds;
  // SwarmStorage _swarm;
  SwarmRanging _ranging;

  RelLocEstimator *_p_ekf[ESTIMATOR_MAX];
  // FullEKF *_p_ekf_ref;
  // SingleEKF *_p_ekf_single;

  PID * _p_rel_pos_pid_x;
  PID * _p_rel_pos_pid_y;
  PID * _p_rel_pos_pid_psi;

  float _vel_cmd[2];
  float _psirate_cmd;
  float _last_cmd_seconds;

  // evaluation & animation
  // uint64_t _uwb_air_time_ns_now;
  // uint64_t _uwb_air_time_ns_avg;
  // float _uwb_air_utilization_now;
  // float _uwb_air_utilization_avg;
  // float _last_air_utilization_calculation;
  // float _time_air_utilization_avg;

  #ifdef LOG
    FileLogger *_pFlogger;
  #endif

  /**
   * @brief Construct the ekf_input of this agent
   */
  void get_own_input(ekf_input_t &own_input);
  
  /**
   * @brief Run the relative position estimator(s)
   */
  void state_estimation();
public:
  /**
   * @brief Construct a new uwb_swarming object
   *
   */
  uwb_swarming();

  /**
   * @brief Initialize the controller
   */
  void init(const uint16_t ID);

  /**
   * @brief Calculate how much data is transmitted through the UWB channel at
   * this agents location (average bps since last call)
   */
  float evaluate_uwb_load(const float time_seconds);

  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  // virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y, float &dpsi);
  virtual void animation(const uint16_t ID);
  virtual void rel_loc_animation(const uint16_t agent_ID, const uint16_t estimator_ID);

};

#endif /*EXPLORATION_H*/
