#include "rel_loc_estimator.h"

#include "draw.h"
#include "trigonometry.h"

#define EKF_INITIALIZATION_PERIOD 20 // don't use indirect measurements during initialization
#define AGENT_INIT_PERIOD 2 // Time during which an agent is not updated with indirect measurements
#define REL_LOC_TIMEOUT 5 // time after which an agent can be discarded [s]

#define NIS_WINDOW_SIZE 20 // Number of normalized innovation errors kept in memory
#define NIS_GRACE_PERIOD 10 // Time before agent can be removed based on bad NIS


static float proc_noise_velXY = MEAS_NOISE_VX;
static float meas_noise_uwb = MEAS_NOISE_UWB;

/***************
 * Constructor *
 ***************/

RelLocEstimator::RelLocEstimator(const uint16_t self_id, uint8_t est_type)
{
    _self_id = self_id;
    _est_type = est_type;

    switch (_est_type)
    {
    case ESTIMATOR_EKF_REF:
        _name = "Reference EKF";
        _n_agents = nagents - 1;
        break;
    case ESTIMATOR_EKF_FULL:
        _name = "Full EKF";
        _n_agents = nagents - 1;
        break;
    case ESTIMATOR_EKF_DYNAMIC:
        _name = "Dynamic EKF";
        _n_agents = 4;
        break;
    case ESTIMATOR_EKF_DECOUPLED:
        _name = "Decoupled EKF";
        _n_agents = nagents - 1;
        break;
    default:
        break;
    }


    _last_prediction_time = 0;

    // initialize own input to zero
    _self_input[EKF_IN_VX] = 0.0;
    _self_input[EKF_IN_VY] = 0.0;

    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        _state.push_back(std::vector<float>(EKF_ST_DIM));
        _input.push_back(std::vector<float>(EKF_IN_DIM));
        _cov.push_back(std::vector<float>(EKF_IN_DIM));
        _cov_det.push_back(0.0f);
        _NIS.push_back(std::vector<float>(NIS_WINDOW_SIZE));
        _mean_NIS.push_back(1.0f);
        _ids.push_back(self_id);
        _last_range.push_back(0.0f);
        _last_seen.push_back(0.0f);
        _agent_added_timestamp.push_back(0.0f);
        
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
    _current_time = time;

    // add own input
    _self_input[EKF_IN_VX] = self_input.vx;
    _self_input[EKF_IN_VY] = self_input.vy;

    // add other agent's input
    uint16_t idx;
    for (uint16_t iInput=0; iInput<_input_queue.size(); iInput++){
        process_input(_input_queue[iInput]);
    }
    _input_queue.clear();
    
    // Prediction step
    if (_current_time > _last_prediction_time + EKF_INTERVAL){
        predict();
        _last_prediction_time = _current_time;
    }

    // Measurement updates
    for (uint16_t iMeas=0; iMeas<_range_queue.size(); iMeas++){
        if (_range_queue[iMeas].id_A == _self_id || _range_queue[iMeas].id_B == _self_id){
            update_with_direct_range(_range_queue[iMeas]); 
        } else if (_est_type != ESTIMATOR_EKF_DECOUPLED) {
            update_with_indirect_range(_range_queue[iMeas]);
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
        _cov_det[iAgent] = fmat_det(_P[iAgent][iAgent]);
    }

    for (uint16_t idx=0; idx<_n_agents; idx++){
        validate_agent(idx);
    }
    // if (!assert_state_valid()){
    //     std::cout << _name << " " << _self_id << ": State Invalid" << std::endl;
    //     reset();
    // }
    // if (!assert_covariance_valid(_P)){
    //     std::cout << _name << " " << _self_id << ": Cov Invalid" << std::endl;
    //     reset();
    // }
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
                    sqrtf(_cov[storage_id][EKF_ST_Y]), _mean_NIS[storage_id]);
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

bool RelLocEstimator::add_agent(const agent_initialization_data_t &init_data, uint16_t *idx){
    // Check if agent is already in
    if (get_index(init_data.id, idx)){
        return false;
    }
    // Find best index to add: empty > last_seen > distance
    bool index_found = false;
    uint16_t best_idx = 0;

    // Check if there is an empty slot
    for (uint16_t iIdx=0; iIdx<_ids.size(); iIdx++){
        if(_ids[iIdx] == _self_id){
            best_idx = iIdx;
            index_found = true;
            break;
        }
    }

    // If no index found yet, check if an agent timed out
    if (index_found == false){
        float best_last_seen = _current_time;
        for (uint16_t iIdx=0; iIdx<_ids.size(); iIdx++){
            if(_current_time - _last_seen[iIdx] > REL_LOC_TIMEOUT && _last_seen[iIdx]<best_last_seen){
                best_idx = iIdx;
                best_last_seen = _last_seen[iIdx];
                index_found = true;
            }
        }
    }

    // if still no index found, find agent with largest range
    if (index_found == false){
        float largest_range = 0; 
        for (uint16_t iIdx=0; iIdx < _ids.size(); iIdx++){
            if (_last_range[iIdx]>largest_range){
                best_idx = iIdx;
                largest_range = _last_range[iIdx];
                index_found = true;
            }
        }
        if (index_found){
            // Hysteresis to avoid frequent switching
            float new_agent_range = sqrtf(powf(init_data.x0, 2)+powf(init_data.y0, 2));
            if (largest_range < 1.25*new_agent_range){
                index_found = false;
            }
        }
    }

    if (index_found){
        // add agent
        //std::cout << _name << " " << _self_id << ": Swapped agent at idx " << best_idx
        //          << ": " << _ids[best_idx] << ">" << agent_id << std::endl;
        *idx = best_idx;
        _ids[best_idx] = init_data.id;
        
        _state[best_idx][EKF_ST_X] = init_data.x0;
        _state[best_idx][EKF_ST_Y] = init_data.y0;
        
        _input[best_idx][EKF_IN_VX] = 0;
        _input[best_idx][EKF_IN_VY] = 0;
        
        for (uint16_t iAgent=0; iAgent<_P.size(); iAgent++){
            for (uint16_t iRow=0; iRow<EKF_ST_DIM; iRow++){
                for (uint16_t iCol=0; iCol<EKF_ST_DIM; iCol++){
                    _P[best_idx][iAgent].data[iRow][iCol] = 0;
                    _P[iAgent][best_idx].data[iRow][iCol] = 0;
                }
            }

        }
        _P[best_idx][best_idx].data[EKF_ST_X][EKF_ST_X] = powf(init_data.stdev_x, 2);
        _P[best_idx][best_idx].data[EKF_ST_Y][EKF_ST_Y] = powf(init_data.stdev_y, 2);

        for (uint16_t iNIS=0; iNIS < NIS_WINDOW_SIZE; iNIS++){
            _NIS[best_idx][iNIS] = 1.0f;
        }
        _mean_NIS[best_idx] = 1.0f;

        _agent_added_timestamp[best_idx] = init_data.timestamp;
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

void RelLocEstimator::process_input(const ekf_input_t &input){
    uint16_t idx;
    if (get_index(input.id, &idx) || initialize_agent(input.id, &idx)){
        _input[idx][EKF_IN_VX] = input.vx;
        _input[idx][EKF_IN_VY] = input.vy;
        _last_seen[idx] = input.timestamp;
    } else {
        // agent not in state space and can't be added
        // --> update data in agent-initializer
        _ag_init->add_velocities(input.id, input.vx, input.vy, input.timestamp);
        
    }
}

bool RelLocEstimator::initialize_agent(const uint16_t agent_id, uint16_t *idx){
    bool success = false;
    agent_initialization_data_t init_data;
    init_data.id = agent_id;
    init_data.timestamp = _current_time;

    switch (_est_type)
    {
    case ESTIMATOR_EKF_REF:
        // Perfect initialization
        init_data.x0 = agents[agent_id]->state[STATE_X] - agents[_self_id]->state[STATE_X];
        init_data.y0 = agents[agent_id]->state[STATE_Y] - agents[_self_id]->state[STATE_Y];
        init_data.stdev_x = 0.01;
        init_data.stdev_y = 0.01;
        success = add_agent(init_data, idx);
        break;
    
    case ESTIMATOR_EKF_DECOUPLED:
        // "Lazy" initialization
        init_data.x0 = 0.0f;
        init_data.y0 = 0.0f;
        init_data.stdev_x = 100;
        init_data.stdev_y = 100;
        success = add_agent(init_data, idx);
        break;

    default:
        // geometric initialization from multiple range measurements
        if (_ag_init->get_initial_position(agent_id, _current_time, &init_data)){
            success = add_agent(init_data, idx);
        }
        break;
    }

    return success;
}

void RelLocEstimator::predict(){
    float dt = _current_time-_last_prediction_time;

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

        _state[iAgent][EKF_ST_X] += dt * d_state[EKF_ST_X];
        _state[iAgent][EKF_ST_Y] += dt * d_state[EKF_ST_Y];
    }

    // Propagate Covariance
    MatrixFloat tmpSS1 (EKF_ST_DIM, EKF_ST_DIM);
    MatrixFloat tmpSS2 (EKF_ST_DIM, EKF_ST_DIM);

    for (uint16_t i=0; i<_n_agents; i++){
        for (uint16_t j=0; j<_n_agents; j++){
            // Prediction Propagation
            // A=I >> Ai*Pij*Aj^T = Pij
            
            if((_est_type != ESTIMATOR_EKF_DECOUPLED) || (i==j)){
                // Add "self" process noise
                _P[i][j].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
                _P[i][j].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
            }
        }
        // Add other agent process noise (only diagonal)
        _P[i][i].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        _P[i][i].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
    }
}

bool RelLocEstimator::update_with_direct_range(const ekf_range_measurement_t &meas){
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
        _last_range[idx] = meas.range;
        float pred = std::sqrt(pow(_state[idx][EKF_ST_X],2)+pow(_state[idx][EKF_ST_Y],2));
        float error = meas.range-pred;

        // Actually, H = [H0 .. Hi .. Hn], but only Hi != 0
        MatrixFloat HT(EKF_ST_DIM, 1);
        HT.data[EKF_ST_X][0] = _state[idx][EKF_ST_X]/pred;
        HT.data[EKF_ST_Y][0] = _state[idx][EKF_ST_Y]/pred;

        // when decoupled, only update state of the agent in question
        if(_est_type == ESTIMATOR_EKF_DECOUPLED){
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

            // normalized innovation squared update
            for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE-1; iNIS++){
                _NIS[idx][iNIS] = _NIS[idx][iNIS+1];
            }
            _NIS[idx][NIS_WINDOW_SIZE-1] = error*error/HPHT_R;

        } else{
            // PHT = [P0i HiT .. Pii HiT .. Pni HiT]T
            std::vector<MatrixFloat> PHT;
            for (uint16_t iRow=0; iRow<_n_agents; iRow++){
                PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
                fmat_mult(_P[iRow][idx], HT, PHT[iRow]);
            }

            float HPHT_R = meas_noise_uwb;
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                HPHT_R += HT.data[iState][0]*PHT[idx].data[iState][0];
            }

            // state update (X = X + K(y-h), K=PH^T/(HPH^T+R)
            for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
                for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                    _state[iAgent][iState] += PHT[iAgent].data[iState][0]*error/HPHT_R;
                }
            }

            // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
            for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
                for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
                    for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                        for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                            _P[iAgent][jAgent].data[iRow][iCol] -= (PHT[iAgent].data[iRow][0]*PHT[jAgent].data[iCol][0])/HPHT_R;
                        }
                    }
                }
            }

            // normalized innovation squared update
            for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE-1; iNIS++){
                _NIS[idx][iNIS] = _NIS[idx][iNIS+1];
            }
            _NIS[idx][NIS_WINDOW_SIZE-1] = error*error/HPHT_R;

        }
        
        float sum_NIS = 0;
        for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE; iNIS++){
            sum_NIS += _NIS[idx][iNIS];
        }
        _mean_NIS[idx] = sum_NIS/NIS_WINDOW_SIZE;
        // if ( _est_type != ESTIMATOR_EKF_DECOUPLED && (sum_NIS > 45.3) && 
        if ( _est_type != ESTIMATOR_EKF_DECOUPLED && (sum_NIS > 100) && 
                (meas.timestamp > _last_reset_time + EKF_INITIALIZATION_PERIOD) &&
                (meas.timestamp > _agent_added_timestamp[idx] + NIS_GRACE_PERIOD)){
            // outside 99.9% confidence for k=20, remove agent to reinitialize
            remove_agent(id_B);
        }
        success = true;
    } else {
        // agent not yet in state space
        _ag_init->add_direct_range(id_B, meas.range, meas.timestamp);
        
    }
    return success;
}

bool RelLocEstimator::update_with_indirect_range(const ekf_range_measurement_t &meas){
    bool success = false;
    if (meas.id_A == _self_id || meas.id_B == _self_id || _est_type == ESTIMATOR_EKF_DECOUPLED){
        return false;
    }
    uint16_t agent_i, agent_j;
    if (get_index(meas.id_A, &agent_i) && get_index(meas.id_B, &agent_j)){
        if ( (meas.timestamp > _agent_added_timestamp[agent_i]-AGENT_INIT_PERIOD)
            || (meas.timestamp > _agent_added_timestamp[agent_j]-AGENT_INIT_PERIOD)){
                return false;
        }
        float pred = std::sqrt(pow(_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X],2)
                                +pow(_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y],2));
        
        float error = meas.range-pred;
        
        // H = [0 .. Hi .. 0 .. Hj .. 0]
        MatrixFloat HiT(EKF_ST_DIM, 1);
        HiT.data[EKF_ST_X][0] = -(_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X])/pred;
        HiT.data[EKF_ST_Y][0] = -(_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y])/pred;

        MatrixFloat HjT(EKF_ST_DIM, 1);
        HjT.data[EKF_ST_X][0] = (_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X])/pred;
        HjT.data[EKF_ST_Y][0] = (_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y])/pred;
        
        std::vector<MatrixFloat> PHT;
        MatrixFloat tmp1(EKF_ST_DIM, 1);
        MatrixFloat tmp2(EKF_ST_DIM, 1);
        for (uint16_t iRow=0; iRow<_n_agents; iRow++){
            PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
            fmat_mult(_P[iRow][agent_i], HiT, tmp1);
            fmat_mult(_P[iRow][agent_j], HjT, tmp2);
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
                _state[iAgent][iState] += PHT[iAgent].data[iState][0]*error/HPHT_R;
            }
        }
        // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
        for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
            for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
                for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                    for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                        _P[iAgent][jAgent].data[iRow][iCol] -= (PHT[iAgent].data[iRow][0]*PHT[jAgent].data[iCol][0])/HPHT_R;
                    }
                }
            }
        }
        success = true;
    } else {
        // at least one agent not in state space
        multilat_point_t mlp;
        if (get_index(meas.id_A, &agent_i)){
            // agent A is know
            float stdev_x = sqrtf(_P[agent_i][agent_i].data[EKF_ST_X][EKF_ST_X]);
            float stdev_y = sqrtf(_P[agent_i][agent_i].data[EKF_ST_Y][EKF_ST_Y]);
            if (stdev_x < 0.1 && stdev_y < 0.1 && _mean_NIS[agent_i]<0.5){
                mlp.id_B = meas.id_A;
                mlp.range = meas.range;
                mlp.timestamp = meas.timestamp;
                mlp.x = _state[agent_i][EKF_ST_X];
                mlp.y = _state[agent_i][EKF_ST_Y];
                mlp.stdev_x = stdev_x;
                mlp.stdev_y = stdev_y;
            }
            _ag_init->add_multilat_point(meas.id_B, mlp);
        }
        if (get_index(meas.id_B, &agent_i)){
            // agent B is know
            float stdev_x = sqrtf(_P[agent_i][agent_i].data[EKF_ST_X][EKF_ST_X]);
            float stdev_y = sqrtf(_P[agent_i][agent_i].data[EKF_ST_Y][EKF_ST_Y]);
            if (stdev_x < 0.1 && stdev_y < 0.1 && _mean_NIS[agent_i]<0.5){
                mlp.id_B = meas.id_B;
                mlp.range = meas.range;
                mlp.timestamp = meas.timestamp;
                mlp.x = _state[agent_i][EKF_ST_X];
                mlp.y = _state[agent_i][EKF_ST_Y];
                mlp.stdev_x = stdev_x;
                mlp.stdev_y = stdev_y;
            }
            _ag_init->add_multilat_point(meas.id_A, mlp);
        }
    }
    return success;
}

void RelLocEstimator::validate_agent(const uint16_t index){
    bool state_invalid = false;
    if (_ids[index]==_self_id){
        // slot empty
        return;
    }
    // check state
    for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
        state_invalid |= isnan(_state[index][iState]);
        state_invalid |= isinf(_state[index][iState]);
    }

    // check covariance
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            for (uint16_t jState=0; jState<EKF_ST_DIM; jState++){
                state_invalid |= isnan(_P[iAgent][index].data[iState][jState]);
                state_invalid |= isinf(_P[iAgent][index].data[iState][jState]);
                state_invalid |= isnan(_P[index][iAgent].data[iState][jState]);
                state_invalid |= isinf(_P[index][iAgent].data[iState][jState]);
            }
        }
    }

    if (state_invalid){
        std::cout << _name << " " << _self_id << ": Invalid State id"<<_ids[index] << std::endl;
        remove_agent(_ids[index]);
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            _state[index][iState]=0;
        }
        for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                for (uint16_t jState=0; jState<EKF_ST_DIM; jState++){
                    _P[iAgent][index].data[iState][jState]=0;
                    _P[index][iAgent].data[iState][jState]=0;
                }
            }
        }
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
