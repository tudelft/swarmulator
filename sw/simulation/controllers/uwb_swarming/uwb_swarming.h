#ifndef UWB_SWARMING_H
#define UWB_SWARMING_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include "controller.h"
#include "randomgenerator.h"
#include "uwb_channel.h"
#include "rel_loc_estimator.h"
// #include "swarm_storage.h"
#include "pid.h"
#include "log.h"

#include "terminalinfo.h"

// Evaluation modes (only ever activate one)

#define EKF_VARIANTS

// #define EKF_DYNAMIC_NMAX

// #define EKF_LIMIT_TEST

// #define EKF_ABLATION_FUL

// #define EKF_ABLATION_DYN

// end eval modes

#ifdef EKF_ABLATION_FUL
  #define EKF_ABLATION
  #define ABLATION_TYPE ESTIMATOR_EKF_FULL
#endif
#ifdef EKF_ABLATION_DYN
  #define EKF_ABLATION
  #define ABLATION_TYPE ESTIMATOR_EKF_DYNAMIC
#endif

// #define COMMAND_LOCAL 1
#define RANGING_TIMEOUT_TICKS 100
#define CMD_INTERVAL 0.1f

#define ANIMATION_TIMEOUT 0.5f
#define INFO_TEXT_UPDATE 1

#ifdef EKF_ABLATION
  #define N_EKF 8
#else
  #define N_EKF 4
#endif

/**
 * Basic exploration behavior which randomly moves in an environment while avoiding neighbors.
 *
 */
class uwb_swarming: public Controller
{
private:
  terminalinfo console_print;
  float _last_velocity_update;
  float _current_target_x;
  float _current_target_y;
  float _total_distance_to_target;
  float _current_ideal_speed;
  bool _avoiding_collision;
  float _next_ping_tx_seconds;
  float _last_estimation_time;
  float _ref_time;
  uint16_t _agents_in_range;

  // SwarmStorage _swarm;
  // SwarmRanging _ranging;

  RelLocEstimator *_p_ekf[N_EKF];

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

  FileLogger *_pFlogger;
  FileLogger *_pFlogger_relpos;

  /**
   * @brief Construct the ekf_input of this agent
   */
  void get_own_input(ekf_input_t &own_input);
  
  /**
   * @brief Run the relative position estimator(s)
   */
  void state_estimation();

  /**
   * @brief Calculate error statistics of the ekf
   */
  void calculate_ekf_errors();

  void log_write_header();
  void log_write_data();

  void log_relpos_write_header();
  void log_relpos_write_data();

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
