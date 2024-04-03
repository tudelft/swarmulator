#ifndef _REL_LOC_ESTIMATOR_H_
#define _REL_LOC_ESTIMATOR_H_

#include <stdint.h>
#include <vector>
#include <string>

#include "ekf_types.h"
#include "agent_initializer.h"
// #include "swarm_storage.h"

#define ESTIMATOR_NONE 0
#define ESTIMATOR_EKF_REF 1
#define ESTIMATOR_EKF_FULL 2
#define ESTIMATOR_EKF_DIRECT 3
#define ESTIMATOR_EKF_SINGLE 4
#define ESTIMATOR_EKF_DYNAMIC 5
#define ESTIMATOR_MAX 6

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
    uint16_t _self_id;
    uint16_t _n_agents;
    float _last_prediction_time;
    float _last_reset_time;

    std::vector<ekf_range_measurement_t> _range_queue;
    std::vector<ekf_input_t> _input_queue;

    AgentInitializer* _ag_init;

    std::vector<uint16_t> _ids;     // unused and timed out slots show _self_id
    std::vector<float> _last_seen;
    std::vector<float> _rssi;
    std::vector<std::vector<float>> _input;
    std::vector<std::vector<float>> _state;
    std::vector<std::vector<float>> _cov; //tmp for some tests

    float _self_input[EKF_IN_DIM];
    bool get_index(const uint16_t id, uint16_t *index); // get storage index of an id
    bool assert_state_valid();

public:
    std::string _name;
    performance_metrics_t _performance;

    RelLocEstimator(const uint16_t nagents, const uint16_t self_id, const std::string name);
    ~RelLocEstimator(){};

    void enqueue_inputs(std::vector<ekf_input_t> &inputs);
    void enqueue_ranges(std::vector<ekf_range_measurement_t> &ranges);

    virtual void step(const float time, ekf_input_t &self_input){};
    
    void update_performance(std::vector<uint16_t> &ids_in_comm_range_ordered, std::vector<float> &relX, std::vector<float> &relY);
    void animate(const uint16_t target_ID);
};

#endif // _REL_LOC_ESTIMATOR_H_