#ifndef SINGLE_EKF_H
#define SINGLE_EKF_H

#include <stdint.h>

#include <string>
#include <vector>

#include "uwb_channel.h"
#include "ekf_math.h"
#include "ekf_types.h"
#include "rel_loc_estimator.h"
#include "log.h"

class SingleEKF : public RelLocEstimator
{
private:
    std::vector<MatrixFloat> _P;
    
    bool update_with_direct_range(const ekf_range_measurement_t &meas);
    
    void clip_covariance();
    void check_state_valid();
    
public:
    /**
     * @brief Construct a new SingleEKF object
     *
     */
    SingleEKF(const uint16_t nagents, const uint16_t selfID, const std::string name);

    void step(const float time, ekf_input_t &self_input);
    void predict(float time);
    void update_with_uwb(const ranging_message &uwb);
    void animate();

};

#endif //FULL_EKF_H