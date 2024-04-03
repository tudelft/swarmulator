#include "rel_loc_estimator.h"

#include "draw.h"
#include "trigonometry.h"

#define INITIALIZATION_PERIOD 30 // don't use indirect measurements during initialization
#define REL_LOC_TIMEOUT 2 // time after which an agent can be discarded [s]

#define MIRROR_END_TIME 30 // during initialization, a mirrored estimate is also calculated


static float proc_noise_velXY = MEAS_NOISE_VX;
static float meas_noise_uwb = MEAS_NOISE_UWB;

/***************
 * Constructor *
 ***************/

RelLocEstimator::RelLocEstimator(const uint16_t self_id, const uint16_t nagents, bool decouple_agents, const std::string name)
{
    _self_id = self_id;
    _n_agents = nagents;
    _always_decouple = decouple_agents;
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
        
        _P.push_back(std::vector<MatrixFloat>());
        for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
            _P[iAgent].push_back(MatrixFloat(EKF_ST_DIM, EKF_ST_DIM));
        }
    }

    _ag_init = new AgentInitializer(_self_id);
    reset();
}

/********************
 * Public Functions *
 ********************/

void RelLocEstimator::reset(){
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        _P[iAgent][iAgent].data[EKF_ST_X][EKF_ST_X] = STDEV_INITIAL_POS_XY;
        _P[iAgent][iAgent].data[EKF_ST_X][EKF_ST_Y] = 0;
        _P[iAgent][iAgent].data[EKF_ST_Y][EKF_ST_X] = 0;
        _P[iAgent][iAgent].data[EKF_ST_Y][EKF_ST_Y] = STDEV_INITIAL_POS_XY;

        _state[iAgent][STATE_X] = 0;
        _state[iAgent][STATE_Y] = 0;

        _ids[iAgent] = _self_id; // means no agent
    }

    mirror_is_init = false;
    _last_reset_time = simtime_seconds;
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

void RelLocEstimator::step(const float time, ekf_input_t &self_input){
    auto start = std::chrono::high_resolution_clock::now();
    bool decouple_agents = _always_decouple;
    if (time-_last_reset_time < INITIALIZATION_PERIOD){
        decouple_agents = true;
    }

    // if (_decouple_agents == false){
    //     if (!mirror_is_init && time < MIRROR_END_TIME && time > MIRROR_START_TIME){
    //         init_mirror();
    //     }
    //     if (mirror_is_init && time > _next_mirror_check){
    //         select_mirror();
    //     }
    // }

    // add own input
    _self_input[EKF_IN_VX] = self_input.vx;
    _self_input[EKF_IN_VY] = self_input.vy;

    // add other agent's input
    uint16_t idx;
    for (uint16_t iInput=0; iInput<_input_queue.size(); iInput++){
        if (get_index(_input_queue[iInput].id, &idx)){
            _input[idx][EKF_IN_VX] = _input_queue[iInput].vx;
            _input[idx][EKF_IN_VY] = _input_queue[iInput].vy;
            _rssi[idx] = _input_queue[iInput].rssi;
            _last_seen[idx] = _input_queue[iInput].timestamp;
        } else {
            // agent not yet in state space
            float x0, y0, var_x, var_y;
            if (_ag_init->get_initial_position(_input_queue[iInput].id, time, &x0, &y0, &var_x, &var_y)){
                if(add_agent(_input_queue[iInput].id, _input_queue[iInput].rssi, x0, y0, var_x, var_y, &idx)){
                    _input[idx][EKF_IN_VX] = _input_queue[iInput].vx;
                    _input[idx][EKF_IN_VY] = _input_queue[iInput].vy;
                    _rssi[idx] = _input_queue[iInput].rssi;
                    _last_seen[idx] = _input_queue[iInput].timestamp;
                }
            } else {
                _ag_init->add_velocities(_input_queue[iInput].id, _input_queue[iInput].vx, _input_queue[iInput].vy, _input_queue[iInput].timestamp);
            }
        }
    }
    _input_queue.clear();
    
    // Prediction step
    if (time > _last_prediction_time + EKF_INTERVAL){
        predict(time, _state, _P, decouple_agents);
        if (mirror_is_init){predict(time, _mirror_state, _mirror_P, decouple_agents);}
        _last_prediction_time = time;
    }

    // Measurement updates
    for (uint16_t iMeas=0; iMeas<_range_queue.size(); iMeas++){
        if (_range_queue[iMeas].id_A == _self_id || _range_queue[iMeas].id_B == _self_id){
            update_with_direct_range(_range_queue[iMeas], _state, _P, &_direct_error, decouple_agents);
            if(mirror_is_init){update_with_direct_range(_range_queue[iMeas], _mirror_state, _mirror_P, &_mirror_direct_error, decouple_agents);}
        
        } else if (decouple_agents == false) {
            update_with_indirect_range(_range_queue[iMeas], _state, _P);
            if(mirror_is_init){update_with_indirect_range(_range_queue[iMeas], _mirror_state, _mirror_P);}

        } else {
            continue;
        }
    }
    _range_queue.clear();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    _performance.comp_time_us = duration.count();
    

    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        _cov[iAgent][EKF_ST_X] = _P[iAgent][iAgent].data[EKF_ST_X][EKF_ST_X];
        _cov[iAgent][EKF_ST_Y] = _P[iAgent][iAgent].data[EKF_ST_Y][EKF_ST_Y];
    }

    if (!assert_state_valid()){
        std::cout << _name << " " << _self_id << ": State Invalid" << std::endl;
        reset();
    }
    if (!assert_covariance_valid(_P)){
        std::cout << _name << " " << _self_id << ": Cov Invalid" << std::endl;
        reset();
    }
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
        d.estimate_with_cov(target_ID, 
                    _state[storage_id][EKF_ST_X],
                    _state[storage_id][EKF_ST_Y],
                    0, 
                    sqrtf(_cov[storage_id][EKF_ST_X]),
                    sqrtf(_cov[storage_id][EKF_ST_Y]));
    }
}

/********************
 * Private Functions *
 ********************/

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

bool RelLocEstimator::add_agent(const uint16_t agent_id, const float agent_rssi, const float x0, const float y0, const float var_x, const float var_y, uint16_t *idx){
    // Check if agent is already in
    if (get_index(agent_id, idx)){
        return false;
    }
    // Find best index to add: empty > last_seen > rssi
    bool index_found = false;
    uint16_t best_idx = 0;
    float best_last_seen = 0;
    float best_rssi = agent_rssi; 
    for (uint16_t iIdx=0; iIdx < _ids.size(); iIdx++){
        if (_ids[iIdx] == _self_id){
            // empty location > best case
            best_idx = iIdx;
            index_found = true;
            break;
        }
        if (_last_seen[iIdx]-simtime_seconds>REL_LOC_TIMEOUT || _last_seen[iIdx]<best_last_seen){
            // an agent hasn't been seen in a while > also good
            best_idx = iIdx;
            best_last_seen = _last_seen[iIdx];
            index_found = true;
            continue;
        }
        if (best_last_seen == 0 && _rssi[iIdx]<best_rssi && _rssi[iIdx] < 1.1*agent_rssi){
            // require at least 5% lower rssi for a change (hysteresis)
            best_idx = iIdx;
            best_rssi = _rssi[iIdx];
            index_found = true;
        }
    }

    if (index_found){
        // add agent
        //std::cout << _name << " " << _self_id << ": Swapped agent at idx " << best_idx
        //          << ": " << _ids[best_idx] << ">" << agent_id << std::endl;
        *idx = best_idx;
        _ids[best_idx] = agent_id;
        
        _state[best_idx][EKF_ST_X] = x0;
        _state[best_idx][EKF_ST_Y] = y0;

        if(mirror_is_init){
            _mirror_state[best_idx][EKF_ST_X] = _state[best_idx][EKF_ST_X];
            _mirror_state[best_idx][EKF_ST_Y] = _state[best_idx][EKF_ST_Y];
        }
        
        _input[best_idx][EKF_IN_VX] = 0;
        _input[best_idx][EKF_IN_VY] = 0;
        
        for (uint16_t iAgent=0; iAgent<_P.size(); iAgent++){
            for (uint16_t iRow=0; iRow<EKF_ST_DIM; iRow++){
                for (uint16_t iCol=0; iCol<EKF_ST_DIM; iCol++){
                    _P[best_idx][iAgent].data[iRow][iCol] = 0;
                    _P[iAgent][best_idx].data[iRow][iCol] = 0;
                    if (mirror_is_init){
                        _mirror_P[best_idx][iAgent].data[iRow][iCol] = 0;
                        _mirror_P[iAgent][best_idx].data[iRow][iCol] = 0;
                    }
                }
            }

        }
        _P[best_idx][best_idx].data[EKF_ST_X][EKF_ST_X] = var_x;
        _P[best_idx][best_idx].data[EKF_ST_Y][EKF_ST_Y] = var_y;
        if (mirror_is_init){
            _mirror_P[best_idx][best_idx].data[EKF_ST_X][EKF_ST_X] = var_x;
            _mirror_P[best_idx][best_idx].data[EKF_ST_Y][EKF_ST_Y] = var_y;
        }
        return true;
    } else {
        return false;
    }
}

void RelLocEstimator::remove_agent(const uint16_t agent_id){
    uint16_t idx;
    if (get_index(agent_id, &idx)){
        _ids[idx] = _self_id; // means no agent
        // No need to clear other vectors as they are only accessed through the id
    } else {
        std::cout << _name << ": Could not remove agent " << agent_id << ": Agent not found!" << std::endl;
    }
}

void RelLocEstimator::predict(float time, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, bool decouple_agents){
    float dt = time-_last_prediction_time;

    // A = Identity
    // Off diagonal blocks of B are 0
    std::vector<MatrixFloat> diagA;
    // std::vector<MatrixFloat> diagB;

    // Agent specific updates
    float d_state[EKF_ST_DIM];
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        // Prediction Jacobian is A = Identity
        // Input Matrix: Not needed if Qs = Qi and noise in x & y is the same
        
        // state change
        d_state[EKF_ST_X] = _input[iAgent][EKF_IN_VX] - _self_input[EKF_IN_VX];
        d_state[EKF_ST_Y] = _input[iAgent][EKF_IN_VY] - _self_input[EKF_IN_VY];

        state[iAgent][EKF_ST_X] += dt * d_state[EKF_ST_X];
        state[iAgent][EKF_ST_Y] += dt * d_state[EKF_ST_Y];
    }

    // Propagate Covariance
    MatrixFloat tmpSS1 (EKF_ST_DIM, EKF_ST_DIM);
    MatrixFloat tmpSS2 (EKF_ST_DIM, EKF_ST_DIM);

    for (uint16_t i=0; i<_n_agents; i++){
        for (uint16_t j=0; j<_n_agents; j++){
            // Prediction Propagation
            // A=I >> Ai*Pij*Aj^T = Pij
            
            if((decouple_agents == false) || (i==j)){
                // Add "self" process noise
                P[i][j].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
                P[i][j].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
            }
        }
        // Add other agent process noise (only diagonal)
        P[i][i].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        P[i][i].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
    }
}

bool RelLocEstimator::update_with_direct_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, float* error_accum, bool decouple_agents){
    bool success = false;

    uint16_t id_B;
    if (meas.id_A == _self_id){
        id_B = meas.id_B;
    } else if (meas.id_B == _self_id){
        id_B = meas.id_A;
    } else{
        return false;
    }

    uint16_t idx;
    if (get_index(id_B, &idx)){
        float pred = std::sqrt(pow(state[idx][EKF_ST_X],2)+pow(state[idx][EKF_ST_Y],2));
        float error = meas.range-pred;
        *error_accum += abs(error);

        // Actually, H = [H0 .. Hi .. Hn], but only Hi != 0
        MatrixFloat HT(EKF_ST_DIM, 1);
        HT.data[EKF_ST_X][0] = state[idx][EKF_ST_X]/pred;
        HT.data[EKF_ST_Y][0] = state[idx][EKF_ST_Y]/pred;

        // when decoupled, only update state of the agent in question
        if(decouple_agents){
            MatrixFloat PHT(EKF_ST_DIM,1);
            fmat_mult(_P[idx][idx], HT, PHT);

            float HPHT_R = meas_noise_uwb;
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                HPHT_R += HT.data[iState][0]*PHT.data[iState][0];
            }

            // State update
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                _state[idx][iState] += PHT.data[iState][0]*error/HPHT_R;
            }

            // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
            MatrixFloat tmpNN (EKF_ST_DIM, EKF_ST_DIM);
            for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                    _P[idx][idx].data[iRow][iCol] -= (PHT.data[iRow][0]*PHT.data[iCol][0])/HPHT_R;
                }
            }


        } else{
            // PHT = [P0i HiT .. Pii HiT .. Pni HiT]T
            std::vector<MatrixFloat> PHT;
            for (uint16_t iRow=0; iRow<_n_agents; iRow++){
                PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
                fmat_mult(P[iRow][idx], HT, PHT[iRow]);
            }

            float HPHT_R = meas_noise_uwb;
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                HPHT_R += HT.data[iState][0]*PHT[idx].data[iState][0];
            }

            // state update (X = X + K(y-h), K=PH^T/(HPH^T+R)
            for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
                for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                    state[iAgent][iState] += PHT[iAgent].data[iState][0]*error/HPHT_R;
                }
            }

            // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
            for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
                for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
                    for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                        for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                            P[iAgent][jAgent].data[iRow][iCol] -= (PHT[iAgent].data[iRow][0]*PHT[jAgent].data[iCol][0])/HPHT_R;
                        }
                    }
                }
            }
        }
        success = true;
    } else {
        // agent not yet in state space
        _ag_init->add_range(id_B, meas.range, meas.timestamp);
    }
    return success;
}

bool RelLocEstimator::update_with_indirect_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P){
    bool success = false;
    if (meas.id_A == _self_id || meas.id_B == _self_id){
        return false;
    }
    uint16_t agent_i, agent_j;
    if (get_index(meas.id_A, &agent_i) && get_index(meas.id_B, &agent_j)){
        float pred = std::sqrt(pow(state[agent_j][EKF_ST_X]-state[agent_i][EKF_ST_X],2)
                                +pow(state[agent_j][EKF_ST_Y]-state[agent_i][EKF_ST_Y],2));
        
        float error = meas.range-pred;
        
        // H = [0 .. Hi .. 0 .. Hj .. 0]
        MatrixFloat HiT(EKF_ST_DIM, 1);
        HiT.data[EKF_ST_X][0] = -(state[agent_j][EKF_ST_X]-state[agent_i][EKF_ST_X])/pred;
        HiT.data[EKF_ST_Y][0] = -(state[agent_j][EKF_ST_Y]-state[agent_i][EKF_ST_Y])/pred;

        MatrixFloat HjT(EKF_ST_DIM, 1);
        HjT.data[EKF_ST_X][0] = (state[agent_j][EKF_ST_X]-state[agent_i][EKF_ST_X])/pred;
        HjT.data[EKF_ST_Y][0] = (state[agent_j][EKF_ST_Y]-state[agent_i][EKF_ST_Y])/pred;
        
        std::vector<MatrixFloat> PHT;
        MatrixFloat tmp1(EKF_ST_DIM, 1);
        MatrixFloat tmp2(EKF_ST_DIM, 1);
        for (uint16_t iRow=0; iRow<_n_agents; iRow++){
            PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
            fmat_mult(P[iRow][agent_i], HiT, tmp1);
            fmat_mult(P[iRow][agent_j], HjT, tmp2);
            fmat_add(tmp1, tmp2, PHT[iRow]);
        }

        float HPHT_R = meas_noise_uwb;
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            HPHT_R += HiT.data[iState][0]*PHT[agent_i].data[iState][0];
            HPHT_R += HjT.data[iState][0]*PHT[agent_j].data[iState][0];
        }

        // state update (X = X + K(y-h), K=PH^T/(HPH^T+R)
        for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                state[iAgent][iState] += PHT[iAgent].data[iState][0]*error/HPHT_R;
            }
        }
        // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
        for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
            for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
                for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                    for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                        P[iAgent][jAgent].data[iRow][iCol] -= (PHT[iAgent].data[iRow][0]*PHT[jAgent].data[iCol][0])/HPHT_R;
                    }
                }
            }
        }
        success = true;
    }
    return success;
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

bool RelLocEstimator::assert_covariance_valid(std::vector<std::vector<MatrixFloat>> &P){
    bool valid = true;
    float det;
    for (uint16_t i=0; i<P.size(); i++){
        det = fmat_det(P[i][i]);
        if(abs(det) < 0.000000001 || isnan(det) || isinf(det)){
            // std::cout << _name << " " <<_self_id<<": Cov" << i << i << "=" << det << std::endl;
            valid = false;
        }
    }
    return valid;
}

/********************
 * Mirror Functions *
 ********************/

void RelLocEstimator::init_mirror() {
    _direct_error = 0;
    _mirror_direct_error = 0;
    // // reset covariance
    // for (uint16_t iAgent = 0; iAgent < _P.size(); iAgent++){
    //     for (uint16_t jAgent = 0; jAgent < _P.size(); jAgent++){
    //         for (uint16_t iState = 0; iState < EKF_ST_DIM; iState++){
    //             for (uint16_t jState = 0; jState < EKF_ST_DIM; jState++){
    //                 _P[iAgent][jAgent].data[iState][jState] = 0;
    //             }
    //         }
    //         _P[iAgent][jAgent].data[EKF_ST_X][EKF_ST_X] = STDEV_INITIAL_POS_XY;
    //         _P[iAgent][jAgent].data[EKF_ST_Y][EKF_ST_Y] = STDEV_INITIAL_POS_XY;
    //     }
    // }
    _mirror_state = _state;
    _mirror_P = _P;
    for (uint16_t iAgent = 0; iAgent < _state.size(); iAgent++){
        // _mirror_state.push_back(_state[iAgent]);
        _mirror_state[iAgent][EKF_ST_X] = -_mirror_state[iAgent][EKF_ST_X];
        _mirror_state[iAgent][EKF_ST_Y] = -_mirror_state[iAgent][EKF_ST_Y];
        // _mirror_P.push_back(_P[iAgent]);
    }
    // float trace = 0;
    // float mirror_trace = 0;

    // for (uint16_t iAgent = 0; iAgent < _P.size(); iAgent++){
    //     trace += fmat_trace(_P[iAgent][iAgent]);
    //     mirror_trace += fmat_trace(_mirror_P[iAgent][iAgent]);
    // }
    std::cout << _name << " " << _self_id << ": Mirror init, "
                // << mirror_trace << " vs " << trace 
                << std::endl;
    
    mirror_is_init = true;
    _next_mirror_check = MIRROR_END_TIME;
}

void RelLocEstimator::select_mirror(){
    // float trace = 0;
    // float mirror_trace = 0;

    // for (uint16_t iAgent = 0; iAgent < _P.size(); iAgent++){
    //     trace += fmat_trace(_P[iAgent][iAgent]);
    //     mirror_trace += fmat_trace(_mirror_P[iAgent][iAgent]);
    // }

    // if (mirror_trace < trace){
    //     std::swap(_state, _mirror_state);
    //     std::swap(_P, _mirror_P);
    //     // for (uint16_t iAgent = 0; iAgent < _state.size(); iAgent++){
    //     // }
    //     std::cout << _name << " " << _self_id << ": Swapped to mirrored estimate, "
    //                 << mirror_trace << " vs " << trace << std::endl;
    // } else {
    //     std::cout << _name << " " << _self_id << ": No swap, "
    //                 << mirror_trace << " vs " << trace << std::endl;
    // }

    if (_mirror_direct_error < _direct_error){
        std::swap(_state, _mirror_state);
        std::swap(_P, _mirror_P);
        // for (uint16_t iAgent = 0; iAgent < _state.size(); iAgent++){
        // }
        std::cout << _name << " " << _self_id << ": Swapped to mirrored estimate, "
                    << _mirror_direct_error << " vs " << _direct_error << std::endl;
    } else {
        // std::cout << _name << " " << _self_id << ": No swap, "
        //             << _mirror_direct_error << " vs " << _direct_error << std::endl;
    }
    _direct_error = 0;
    _mirror_direct_error = 0;
    _next_mirror_check += MIRROR_END_TIME;
    // mirror_is_init = false;
}

