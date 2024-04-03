#ifndef _REL_LOC_ESTIMATOR_H_
#define _REL_LOC_ESTIMATOR_H_

#include <stdint.h>
#include <vector>
#include <string>

#include "ekf_types.h"
#include "ekf_math.h"
#include "agent_initializer.h"
// #include "swarm_storage.h"

#define ESTIMATOR_NONE 0
#define ESTIMATOR_EKF_REF 1
#define ESTIMATOR_EKF_FULL 2
#define ESTIMATOR_EKF_DYNAMIC 3
#define ESTIMATOR_EKF_DECOUPLED 4
#define ESTIMATOR_MAX 5

#define EKF_INTERVAL 0.01f
#define COVARIANCE_MAX_VALUE 100000 // max error on initialization can be 2*COMMUNICATION_RANGE


struct error_stats_t {
    float max;
    float mean;
    uint8_t N;
};

struct performance_metrics_t {
    error_stats_t c1;
    error_stats_t c3;
    error_stats_t c5;
    error_stats_t icr;
    float comp_time_us;
};

class RelLocEstimator
{
protected:
    // START mirror stuff (possibly remove later)
    // small variations during initialization can cause the 
    // estimate to be mirrored, we can fix this during initialization
    // by comparing the change of P in the regular and mirrored estimate
    std::vector<std::vector<MatrixFloat>> _mirror_P;
    std::vector<std::vector<float>> _mirror_state;

    float _direct_error;
    float _mirror_direct_error;
    float _next_mirror_check;

    bool mirror_is_init;
    void init_mirror(); // initialize the mirrored state
    void select_mirror(); // check which estimate is better and move it to the main state
    // END mirror stuff

    uint16_t _self_id;
    uint16_t _n_agents;
    bool _always_decouple;

    float _last_prediction_time;
    float _last_reset_time;

    std::vector<ekf_range_measurement_t> _range_queue;
    std::vector<ekf_input_t> _input_queue;

    AgentInitializer* _ag_init;

    std::vector<uint16_t> _ids;                 // agent ids corresponding to blocks in the state space. Unused and timed out slots show _self_id
    std::vector<std::vector<float>> _input;     // input vector in blocks (by agent)
    std::vector<std::vector<float>> _state;     // state vector in blocks (by agent)
    std::vector<std::vector<MatrixFloat>> _P;   // Covariance Matrix in blocks (by agent)
    std::vector<float> _last_seen;
    std::vector<float> _rssi;
    std::vector<std::vector<float>> _cov; //tmp for animation

    float _self_input[EKF_IN_DIM];

    /**
     * @brief get the index at which an agent is stored in the vectors
     * @param[in] id: ID of the agent to be localized
     * @param[out] index: index at which the agent with given id is stored
     * @return true if index can be provided, false if agent is unknown
     */
    bool get_index(const uint16_t id, uint16_t *index); // get storage index of an id
    
    /**
     * @brief Add a new agent to the state space an appropriate position in the vectors
     * Position can't be specified
     * @param[out] idx: returns the position at which the new agent was added
     * @return true if agent was added, false if not
     */
    bool add_agent(const uint16_t agent_id, const float agent_rssi, const float x0, const float y0, const float var_x, const float var_y, uint16_t *idx);
    
    /**
     * @brief Remove an agent from the state space
     * 
     * @param[in] agent_id: ID of agent to remove
     */
    void remove_agent(const uint16_t agent_id);
    
    void predict(float time, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, bool decouple_agents);
    bool update_with_direct_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, float* error_accum, bool decouple_agents);
    bool update_with_indirect_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P);


    bool assert_state_valid();
    bool assert_covariance_valid(std::vector<std::vector<MatrixFloat>> &P);

public:
    std::string _name;
    performance_metrics_t _performance;

    RelLocEstimator(const uint16_t self_id, const uint16_t nagents, bool decouple_agents, const std::string name);
    ~RelLocEstimator(){};

    /**
     * @brief resets the estimator to its original state at startup
     */
    void reset();

    void enqueue_inputs(std::vector<ekf_input_t> &inputs);
    void enqueue_ranges(std::vector<ekf_range_measurement_t> &ranges);

    void step(const float time, ekf_input_t &self_input);
    

    void update_performance(std::vector<uint16_t> &ids_in_comm_range_ordered, std::vector<float> &relX, std::vector<float> &relY);
    void animate(const uint16_t target_ID);
};

#endif // _REL_LOC_ESTIMATOR_H_