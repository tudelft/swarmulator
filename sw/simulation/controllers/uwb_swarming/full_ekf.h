#ifndef FULL_EKF_H
#define FULL_EKF_H

#include <stdint.h>

#include <string>
#include <vector>

#include "uwb_channel.h"
#include "ekf_math.h"
#include "ekf_types.h"
// #include "swarm_storage.h"
#include "rel_loc_estimator.h"

/** 
 * @brief EKF with a single state vector for all agents that can
 * therefore use indirect measurements. Possible behaviours:
 * - Reference EKF: Use n_agents = agents_in_sim and provide all inputs & measurements
 * - Full EKF: Use n_agents = agents_in_sim with local inputs & ranges
 * - Dynamic EKF: Use n_agents < agents_in_sim with local inputs & ranges
 * 
 */

class FullEKF : public RelLocEstimator
{
private:
    std::vector<std::vector<MatrixFloat>> _P;

    // small variations during initialization can cause the 
    // estimate to be mirrored, we can fix this during initialization
    // by comparing the change of P in the regular and mirrored estimate
    std::vector<std::vector<MatrixFloat>> _mirror_P;
    std::vector<std::vector<float>> _mirror_state;

    float _direct_error;
    float _mirror_direct_error;
    float _next_mirror_check;
    /**
     * @brief Get the index at which the state/input/covariance for the agent with given
     * is stored. Returns false if the id is not currently tracked by the filter.
     *
     * @param id 
     * @param index
     */

    bool update_with_direct_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, float* error_accum);
    bool update_with_indirect_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P);

    void remove_agent(const uint16_t agent_id);
    // add agent at an empty position in the vectors. Position CANNOT be
    // specified, but is returned in idx. Returns false if agent couldn't be added.
    bool add_agent(const uint16_t agent_id, const float agent_rssi, uint16_t *idx);

    void clip_covariance(std::vector<std::vector<MatrixFloat>> &P);
    bool assert_covariance_valid(std::vector<std::vector<MatrixFloat>> &P);

    bool mirror_is_init;
    void init_mirror(); // initialize the mirrored state
    void select_mirror(); // check which estimate is better and move it to the main state

public:
    /**
     * @brief Construct a new FullEKF object
     *
     * @param nagents Number of agents to range to
     * @param selfID id of the agent on which the EKF is running
     */
    FullEKF(const uint16_t nagents, const uint16_t selfID, const std::string name);

    void step(const float time, ekf_input_t &self_input);
    void predict(float time, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P);
};

#endif //FULL_EKF_H