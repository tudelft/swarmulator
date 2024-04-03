#include "rel_loc_estimator.h"

#include "draw.h"
#include "trigonometry.h"





RelLocEstimator::RelLocEstimator(const uint16_t nagents, const uint16_t self_id, const std::string name)
{
    _self_id = self_id;
    _n_agents = nagents;
    _name = name;
    _last_prediction_time = 0;

    // initialize own input to zero
    _self_input[EKF_IN_VX] = 0.0;
    _self_input[EKF_IN_VY] = 0.0;

    for (uint16_t iAgent=0; iAgent<nagents; iAgent++){
        _state.push_back(std::vector<float>(EKF_ST_DIM));
        _input.push_back(std::vector<float>(EKF_IN_DIM));
        _cov.push_back(std::vector<float>(EKF_IN_DIM));
        _ids.push_back(self_id);
        _rssi.push_back(0.0f);
        _last_seen.push_back(0.0f);
    }
}

void RelLocEstimator::enqueue_inputs(std::vector<ekf_input_t> &inputs){
    for (uint16_t iInput=0; iInput<inputs.size(); iInput++){
        _input_queue.push_back(inputs[iInput]);
    }
}

void RelLocEstimator::enqueue_ranges(std::vector<ekf_range_measurement_t> &ranges){
    for (uint16_t iRange=0; iRange<ranges.size(); iRange++){
        _range_queue.push_back(ranges[iRange]);
    }
}

bool RelLocEstimator::get_index(const uint16_t id, uint16_t *index){
    bool success = false;
    if (id != _self_id){
        for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
            if (_ids[iAgent]==id){
                *index=iAgent;
                success = true;
            }
        }
    }
    return success;
}

void RelLocEstimator::update_performance(std::vector<uint16_t> &ids_in_comm_range_ordered, std::vector<float> &relX, std::vector<float> &relY){
    float ex, ey, etot;
    uint16_t ekf_idx;

    float accumulator = 0.0f;
    float e_max = 0;
    uint8_t count = 0;

    uint16_t N_icr = ids_in_comm_range_ordered.size();
    
    // calculate individual errors for agents in comm range
    for (uint16_t i=0; i<N_icr; i++){
        if(get_index(ids_in_comm_range_ordered[i], &ekf_idx)){
            ex = _state[ekf_idx][EKF_ST_X] - relX[i];
            ey = _state[ekf_idx][EKF_ST_Y] - relY[i];
            etot= sqrt(pow(ex, 2) + pow(ey, 2));

            accumulator += etot;
            count += 1;
            if (etot > e_max){
                e_max = etot;
            }
        }

        // error stats only for C1, C3, C5
        error_stats_t *pErrorStats;
        switch (i) {
            case 0:
                pErrorStats = &_performance.c1;
                break;
            case 2:
                pErrorStats = &_performance.c3;
                break;
            case 4:
                pErrorStats = &_performance.c5;
                break;
            default:
                pErrorStats = NULL;
                break;
        }

        if (pErrorStats != NULL){
            pErrorStats->N = count;
            pErrorStats->max = e_max;
            if (count > 0){
                pErrorStats->mean = accumulator/count;
            } else {
                pErrorStats->mean = 0;
            }
        }
    }
    // finally for ICR (all in comm range)
    _performance.icr.N = count;
    _performance.icr.max = e_max;
    if (count > 0){
        _performance.icr.mean = accumulator/count;
    } else {
        _performance.icr.mean = 0;
    }
}


void RelLocEstimator::animate(const uint16_t target_ID){
    draw d;
    uint16_t storage_id;
    if (get_index(target_ID, &storage_id)){
        // float x_l, y_l;
        // rotate_g2l_xy(_state[storage_id][EKF_ST_X], _state[storage_id][EKF_ST_Y],
        //             agents[_self_id]->state[STATE_YAW], x_l, y_l);        
        // d.estimate(target_ID, x_l,y_l, 0);
        // d.estimate(target_ID, 
        //             _state[storage_id][EKF_ST_X],
        //             _state[storage_id][EKF_ST_Y],
        //             0);
        d.estimate_with_cov(target_ID, 
                    _state[storage_id][EKF_ST_X],
                    _state[storage_id][EKF_ST_Y],
                    0, 
                    sqrtf(_cov[storage_id][EKF_ST_X]),
                    sqrtf(_cov[storage_id][EKF_ST_Y]));
    }
}

bool RelLocEstimator::assert_state_valid(){
    bool valid = true;
    for (uint8_t i=0; i<_state.size(); i++){
        if (_state[i][EKF_ST_X] > 1000 || _state[i][EKF_ST_X] < -1000 
            || _state[i][EKF_ST_Y] > 1000 || _state[i][EKF_ST_Y] < -1000){
            valid = false;
            break;
        }
        if (isnan(_state[i][EKF_ST_X]) || isinf(_state[i][EKF_ST_X]) || 
            isnan(_state[i][EKF_ST_Y]) || isinf(_state[i][EKF_ST_Y])){
            valid = false;
            break;
        }
        
    }
    return valid;
}