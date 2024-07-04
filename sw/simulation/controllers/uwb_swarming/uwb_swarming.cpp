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
// #include "swarm_storage.h"
#include "swarm_ranging.h"
#include "pid.h"
#include "rel_loc_estimator.h"
#include "log.h"
#include "settings.h"

#define SENSOR_MAX_RANGE 1.8  // Sensor for wall avoidance

#define RANGE_TO_NONE 0
#define RANGE_TO_CLOSEST 1
#define RANGE_TO_RANDOM 2
#define RANGE_TO_ALL 3

#define V_MAX 1.0f // m/s
#define V_RAMPUP_TIME 1.0f // s

// Initialization: smaller movement with frequent direction changes for better initialization
#define INITIALIZATION_TIME 30 // [s]
#define INITIALIZATION_SIZE 2.0f // movement size [m]


// #define DESIRED_REL_X 1.5f
// #define DESIRED_REL_Y 0.8f
// #define DESIRED_REL_PSI 0.0f

uwb_swarming::uwb_swarming() : Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
  _last_velocity_update = 0;
  _next_ping_tx_seconds = COMMUNICATION_BASE_INTERVAL + rg.uniform_float(-COMMUNICATION_BASE_INTERVAL / 10, COMMUNICATION_BASE_INTERVAL / 10);
  _last_estimation_time = -1;
  _current_target_x = 0;
  _current_target_y = 0;
  _total_distance_to_target = 0;
  _current_ideal_speed = 0;
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

  // Standard Comparison
  #ifdef EKF_VARIANTS
    _p_ekf[0] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_REF);
    _p_ekf[1] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_FULL);
    _p_ekf[2] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC);    
    _p_ekf[3] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DECOUPLED);
  #endif

  // Dyn EKF evaluation
  #ifdef EKF_DYNAMIC_NMAX
    _p_ekf[0] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC, 5, "05dyn");
    _p_ekf[1] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC, 8, "08dyn");
    _p_ekf[2] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC, 10, "10dyn");    
    _p_ekf[3] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC, 15, "15dyn");
  #endif

  // EKF ablation study
  #ifdef EKF_ABLATION
    uint8_t ekf_ablation_type = ABLATION_TYPE;
    _p_ekf[0] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "noo improvements");
    _p_ekf[0]->disable_all_ambiguity_improvements();

    _p_ekf[1] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "a00 improved init");
    _p_ekf[1]->disable_all_ambiguity_improvements();
    _p_ekf[1]->_enable_improved_initialization = true;

    _p_ekf[2] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "b00 cov inflation");
    _p_ekf[2]->disable_all_ambiguity_improvements();
    _p_ekf[2]->_enable_covariance_inflation = true;

    _p_ekf[3] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "ab0 init and cov");
    _p_ekf[3]->disable_all_ambiguity_improvements();
    _p_ekf[3]->_enable_improved_initialization = true;
    _p_ekf[3]->_enable_covariance_inflation = true;

    _p_ekf[4] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "abc select secondary");
    _p_ekf[4]->disable_all_ambiguity_improvements();
    _p_ekf[4]->_enable_improved_initialization = true;
    _p_ekf[4]->_enable_covariance_inflation = true;
    _p_ekf[4]->_enable_selective_secondary_range = true;

    _p_ekf[5] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "abd withhold secondary");
    _p_ekf[5]->disable_all_ambiguity_improvements();
    _p_ekf[5]->_enable_improved_initialization = true;
    _p_ekf[5]->_enable_covariance_inflation = true;
    _p_ekf[5]->_enable_withhold_measurements = true;

    _p_ekf[6] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "abe reset agents");
    _p_ekf[6]->disable_all_ambiguity_improvements();
    _p_ekf[6]->_enable_improved_initialization = true;
    _p_ekf[6]->_enable_covariance_inflation = true;
    _p_ekf[6]->_enable_nis_agent_reset = true;

    _p_ekf[7] = new RelLocEstimator(this->ID, ekf_ablation_type, 10, "all improvements");
  #endif
  #ifdef EKF_LIMIT_TEST
    _p_ekf[0] = new RelLocEstimator(this->ID, ESTIMATOR_EKF_DYNAMIC);
    _p_ekf[1] = NULL;
    _p_ekf[2] = NULL;    
    _p_ekf[3] = NULL;
  #endif

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
    log_write_header();

    if (ID==0){
      name << "_relpos";
      _pFlogger_relpos = new FileLogger(name.str());
      log_relpos_write_header();
    }
  #endif

}

// velocity command go to random points with fixed yaw
void uwb_swarming::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  // since the simulation time runs on a different thread, to make sure all estimators
  // use the same reference time we avoid using simtime_seconds inside the controller
  _ref_time = simtime_seconds;
  if (_ref_time >= _last_estimation_time + EKF_ESTIMATION_INTERVAL){
    state_estimation();
    _last_estimation_time = _ref_time;
  }
  
  float delta_x = _current_target_x - agents[ID]->state[STATE_X];
  float delta_y = _current_target_y - agents[ID]->state[STATE_Y];
  float distance_to_target = sqrtf(powf(delta_x, 2.0f)+powf(delta_y, 2.0f));

  // check if target is reached
  if (distance_to_target < 0.1 || _total_distance_to_target < 0.1){
    float env_lim = environment.limits();
    if (_ref_time<INITIALIZATION_TIME){
      // during initialization, change movement more often
      float x_now = agents[ID]->state[STATE_X];
      float y_now = agents[ID]->state[STATE_Y];
      
      _current_target_x = rg.uniform_float(x_now-INITIALIZATION_SIZE, x_now+INITIALIZATION_SIZE);
      _current_target_x = std::max(-env_lim, std::min(_current_target_x, env_lim));
      
      _current_target_y = rg.uniform_float(y_now-INITIALIZATION_SIZE, y_now+INITIALIZATION_SIZE);
      _current_target_y = std::max(-env_lim, std::min(_current_target_y, env_lim));
    } else{
      _current_target_x = rg.uniform_float(-env_lim, env_lim);
      _current_target_y = rg.uniform_float(-env_lim, env_lim);
    }

    // std::cout << "Ag" << ID << ": New target (" << _current_target_x << "," << _current_target_y << ")" << std::endl;
    delta_x = _current_target_x - agents[ID]->state[STATE_X];
    delta_y = _current_target_y - agents[ID]->state[STATE_Y];
    distance_to_target = sqrtf(powf(delta_x, 2.0f)+powf(delta_y, 2.0f));
    _total_distance_to_target = distance_to_target;
    _current_ideal_speed = 0;
  }

  float dt = _ref_time-_last_cmd_seconds;  
  _last_cmd_seconds = _ref_time;
  if (distance_to_target < _total_distance_to_target/2){
    _current_ideal_speed += (V_MAX/V_RAMPUP_TIME)*dt;
  } else {
    _current_ideal_speed -= (V_MAX/V_RAMPUP_TIME)*dt;
  }

  _vel_cmd[0] = std::max(_current_ideal_speed, V_MAX) * delta_x/distance_to_target;
  _vel_cmd[1] = std::max(_current_ideal_speed, V_MAX) * delta_y/distance_to_target;

  v_x = _vel_cmd[0];
  v_y = _vel_cmd[1];
  // could add some wall avoidance here, but not needed if square room with no obstacles
  // psirate = 0;
  // wall_avoidance_turn(ID, v_x, psirate, SENSOR_MAX_RANGE);
}


void uwb_swarming::get_own_input(ekf_input_t &own_input)
{
  float vx = agents[this->ID]->state[STATE_VX];
  float vy = agents[this->ID]->state[STATE_VY];
  
  own_input.id = this->ID;
  own_input.vx = vx + this->rg.gaussian_float(0, MEAS_NOISE_VX);
  own_input.vy = vy + this->rg.gaussian_float(0, MEAS_NOISE_VY);
  own_input.timestamp = _ref_time;
}

void uwb_swarming::state_estimation()
{
  ekf_input_t own_input;
  get_own_input(own_input);

  if (_ref_time >= _next_ping_tx_seconds)
  {
    _ranging.send_ranging_ping(own_input.vx, own_input.vy);
    _next_ping_tx_seconds = _ref_time + COMMUNICATION_BASE_INTERVAL + rg.uniform_float(-COMMUNICATION_BASE_INTERVAL / 10, COMMUNICATION_BASE_INTERVAL / 10);
  }
  _ranging.process_incoming_data(_ref_time);

  std::vector<ekf_range_measurement_t> measurements;
  std::vector<ekf_input_t> inputs;
  _ranging.get_new_ranging(measurements, inputs);

  std::vector<ekf_range_measurement_t> all_measurements;
  std::vector<ekf_input_t> all_inputs;
  _ranging.get_all_ranging(all_measurements, all_inputs, _ref_time);

  for (uint8_t iEst = 0; iEst < N_EKF; iEst++)
  {
    if (_p_ekf[iEst] == NULL)
    {
      continue;
    } else if (iEst == ESTIMATOR_EKF_REF){
      _p_ekf[ESTIMATOR_EKF_REF]->enqueue_inputs(all_inputs);
      _p_ekf[ESTIMATOR_EKF_REF]->enqueue_ranges(all_measurements);
      _p_ekf[ESTIMATOR_EKF_REF]->step(_ref_time, own_input);
    }
    else
    {
      _p_ekf[iEst]->enqueue_inputs(inputs);
      _p_ekf[iEst]->enqueue_ranges(measurements);
      _p_ekf[iEst]->step(_ref_time, own_input);
    }
  }

  // Calculate performance indicators
  _ranging._evaluator.calculate_load(_ref_time);
  calculate_ekf_errors();
  
  #ifdef LOG
    log_write_data();
    if (ID==0){
      log_relpos_write_data();
    }
  #endif
}

void uwb_swarming::calculate_ekf_errors(){
  std::vector<uint16_t> ids_in_comm_range_ordered;
  std::vector<float> relX;
  std::vector<float> relY;
  std::vector<float> range;
  std::vector<float>::iterator it;
  uint16_t idx;
  float dx_g, dy_g, range_tmp;

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
        
        relX.insert(relX.begin()+idx, dx_g);
        relY.insert(relY.begin()+idx, dy_g);      
      }
    }
  }
  _agents_in_range = ids_in_comm_range_ordered.size();
  for (uint8_t iEst = 0; iEst < N_EKF; iEst++){
    if (_p_ekf[iEst] == NULL)
      {
        continue;
      }
      else
      {
        _p_ekf[iEst]->update_performance(ids_in_comm_range_ordered, relX, relY);
      }
  }
}

void uwb_swarming::animation(const uint16_t ID)
{
  draw d;

  d.circle_loop(COMMUNICATION_RANGE);
}

void uwb_swarming::rel_loc_animation(const uint16_t agent_ID, const uint16_t estimator_ID)
{
  draw d;

  // draw self
  d.agent_raw(agent_ID, 0.0f, 0.0f, 0.0f);
  d.circle_loop(COMMUNICATION_RANGE);

  _ranging.rel_loc_animation(agent_ID);

  // // other agents in range, groundtruth
  float dx_g, dy_g; //, dx_l, dy_l;
  float d_yaw;
  //float own_yaw = agents[agent_ID]->state[STATE_YAW];

  for (uint16_t i_agent = 0; i_agent < agents.size(); i_agent++)
  {
    if (i_agent == agent_ID)
    {
      continue;
    }

    // Draw groundtruth
    dx_g = agents[i_agent]->state[STATE_X] - agents[agent_ID]->state[STATE_X];
    dy_g = agents[i_agent]->state[STATE_Y] - agents[agent_ID]->state[STATE_Y];
    // rotate_g2l_xy(dx_g, dy_g, own_yaw, dx_l, dy_l);
    d_yaw = agents[i_agent]->state[STATE_YAW] - agents[agent_ID]->state[STATE_YAW];

    // d.agent_raw(i_agent, dx_l, dy_l, d_yaw);
    d.agent_raw(i_agent, dx_g, dy_g, d_yaw);
  
    if(_p_ekf[estimator_ID] != NULL){
      _p_ekf[estimator_ID]->animate(i_agent);
    }
  }

  std::stringstream infostream;
  std::stringstream statestream;

  infostream << "Agent " << std::to_string(agent_ID);
  statestream << "States\n";

  if (_p_ekf[estimator_ID] != NULL)
  {
    infostream << "\nEstimator " << _p_ekf[estimator_ID]->_name;
    infostream << " - Abs Mean Error:";
    infostream << " C1=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c1.abs_mean;
    infostream << " C3=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c3.abs_mean;
    infostream << " C5=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.c5.abs_mean;
    infostream << " ICR=" << std::fixed << std::setprecision(2) << _p_ekf[estimator_ID]->_performance.icr.abs_mean;
    infostream << " n=" << _agents_in_range;
    infostream << " time (us): " << _p_ekf[estimator_ID]->_performance.comp_time_us;
    // d.info_text("Agent " + std::to_string(agent_ID) +
    //             "\nEstimator " + _p_ekf[estimator_ID]->_name +
    //             " - Mean Error: N/A" +
    //             "\nLocal UWB Datarate: " + std::to_string((int)(100 * _ranging._evaluator._current_load)) + "% (" + std::to_string((int)(100 * _ranging._evaluator._avg_load)) + "%)");
    for (uint16_t iAgent=0; iAgent<_p_ekf[estimator_ID]->_n_agents; iAgent++){
      if (_p_ekf[estimator_ID]->_ids[iAgent]==ID){
        statestream << "N/A\n"; 
      } else{
        statestream << _p_ekf[estimator_ID]->_ids[iAgent] << ": (";
        statestream << std::fixed << std::setprecision(1);
        statestream << _p_ekf[estimator_ID]->_state[iAgent][EKF_ST_X] << ",";
        statestream << _p_ekf[estimator_ID]->_state[iAgent][EKF_ST_Y] << ")\n";
      }
    }
  }
  else
  {
    infostream << "\nEstimator None";
    infostream << " - Mean Error: N/A";
    // d.info_text("Agent " + std::to_string(agent_ID) +
    //             "\nEstimator None" + 
    //             " - Mean Error: N/A" +
    //             "\nLocal UWB Datarate: " + std::to_string((int)(100 * _ranging._evaluator._current_load)) + "% (" + std::to_string((int)(100 * _ranging._evaluator._avg_load)) + "%)");
  }
  infostream << "\nLocal UWB Datarate: "; 
  infostream << std::to_string((int)(100 * _ranging._evaluator._current_load)) << "%";
  infostream << "(" << std::to_string((int)(100 * _ranging._evaluator._avg_load)) << "%)";

  d.info_text(infostream.str());
  d.state_text(statestream.str());
}

void uwb_swarming::log_write_header(){
  std::stringstream header;
  header << "time";           // col G.0
  header << "," << "n_icr";   // col G.1
  header << "," << "uwb_load";// col G.2
  for (uint8_t iEst = 0; iEst < N_EKF; iEst++){
    if (_p_ekf[iEst] != NULL){
      std::string short_name = _p_ekf[iEst]->_name.substr(0,3);
      header << "," << short_name << "_c1_abs";   // col E.0
      header << "," << short_name << "_c1_rel";   // col E.1
      header << "," << short_name << "_c3_abs";   // col E.2
      header << "," << short_name << "_c3_rel";   // col E.3
      header << "," << short_name << "_c5_abs";   // col E.4
      header << "," << short_name << "_c5_rel";   // col E.5
      header << "," << short_name << "_icr_abs";  // col E.6
      header << "," << short_name << "_icr_rel";  // col E.7
      header << "," << short_name << "_t_us";     // col E.8
      header << "," << short_name << "_nis_sum";  // col E.9
      header << "," << short_name << "_nis_dof";  // col E.10
    }
  }
  header << std::endl;
  _pFlogger->write_data(header);
}

void uwb_swarming::log_write_data(){
  std::stringstream data;
  data << _ref_time;                                // col G.0
  data << "," << _agents_in_range;                  // col G.1
  data << std::fixed << std::setprecision(3); 
  data << "," << _ranging._evaluator._current_load; // col G.2
  for (uint8_t iEst = 0; iEst < N_EKF; iEst++){
    if (_p_ekf[iEst] != NULL){
      data << "," << _p_ekf[iEst]->_performance.c1.abs_mean;  // col E.0
      data << "," << _p_ekf[iEst]->_performance.c1.rel_mean;  // col E.1
      data << "," << _p_ekf[iEst]->_performance.c3.abs_mean;  // col E.2
      data << "," << _p_ekf[iEst]->_performance.c3.rel_mean;  // col E.3
      data << "," << _p_ekf[iEst]->_performance.c5.abs_mean;  // col E.4
      data << "," << _p_ekf[iEst]->_performance.c5.rel_mean;  // col E.5
      data << "," << _p_ekf[iEst]->_performance.icr.abs_mean; // col E.6
      data << "," << _p_ekf[iEst]->_performance.icr.rel_mean; // col E.7
      data << "," << _p_ekf[iEst]->_performance.comp_time_us; // col E.8
      data << "," << _p_ekf[iEst]->_performance.nis_sum;      // col E.9
      data << "," << _p_ekf[iEst]->_performance.nis_dof;      // col E.10
    }
  }
  data << std::endl;
  _pFlogger->write_data(data);
}


void uwb_swarming::log_relpos_write_header(){
  std::stringstream header;
  header << "time";           // col G.0
  for (uint16_t iAgent=1; iAgent < nagents; iAgent++){
    header << "," << "x" << iAgent; // col G.A.1
    header << "," << "y" << iAgent; // col G.A.2
  }
  
  for (uint8_t iEst = 0; iEst < N_EKF; iEst++){
    if (_p_ekf[iEst] != NULL){
      std::string short_name = _p_ekf[iEst]->_name.substr(0,3);
      for (uint16_t iAgent=1; iAgent<nagents; iAgent++){
        header << "," << short_name << "_x" << iAgent;   // col E.A.0
        header << "," << short_name << "_y" << iAgent;   // col E.A.1
      }
    }
  }
  header << std::endl;
  _pFlogger_relpos->write_data(header);
}

void uwb_swarming::log_relpos_write_data(){
  std::stringstream data;
  data << _ref_time;                                // col G.0
  for (uint16_t iAgent=1; iAgent < nagents; iAgent++){
    // Draw groundtruth
    float dx_g = agents[iAgent]->state[STATE_X] - agents[ID]->state[STATE_X];
    float dy_g = agents[iAgent]->state[STATE_Y] - agents[ID]->state[STATE_Y];
    data << std::fixed << std::setprecision(3); 
    data << "," << dx_g; // col G.A.1
    data << "," << dy_g; // col G.A.2
  }

  for (uint8_t iEst = 0; iEst < N_EKF; iEst++){
    if (_p_ekf[iEst] != NULL){
      for (uint16_t iAgent=1; iAgent < nagents; iAgent++){
        float dx_e, dy_e;
        if (_p_ekf[iEst]->get_state(iAgent, &dx_e, &dy_e)){
          data << "," << dx_e; // col E.A.0
          data << "," << dy_e; // col E.A.1
        } else {
          data << "," << "nan"; // col E.A.0
          data << "," << "nan"; // col E.A.1
        }
      }
    }
  }
  data << std::endl;
  _pFlogger_relpos->write_data(data);
}