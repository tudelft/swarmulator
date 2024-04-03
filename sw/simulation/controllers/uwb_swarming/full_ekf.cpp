#include "full_ekf.h"

#include <math.h>

#include <string>
#include <vector>
#include <chrono>

#include "main.h"
#include "ekf_math.h"
#include "draw.h"
#include "trigonometry.h"


// #define COVARIANCE_MIN_VALUE 1e-6
#define AGENT_TIMEOUT_MIN 2 // earliest time after which an agent is discarded [s]
#define MIRROR_START_TIME 1
#define MIRROR_END_TIME 30 // during initialization, a mirrored estimate is also calculated
#define INITIALIZATION_PERIOD 30 // don't use indirect measurements during initialization

static float proc_noise_velXY = MEAS_NOISE_VX;
static float meas_noise_uwb = MEAS_NOISE_UWB;

FullEKF::FullEKF(const uint16_t nagents, const uint16_t selfID, const bool decouple_agents, const std::string name)
        : RelLocEstimator(nagents, selfID, name)
{
    // always decouple for initialization 
    _decouple_agents = true;
    _stay_decoupled = decouple_agents;

    // initialize vectors for state and covariance matrix
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        _P.push_back(std::vector<MatrixFloat>());
        for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
            _P[iAgent].push_back(MatrixFloat(EKF_ST_DIM, EKF_ST_DIM));
        }
    }

    _ag_init = new AgentInitializer(_self_id);
    reset();
}

void FullEKF::reset(){
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

void FullEKF::remove_agent(const uint16_t agent_id){
    uint16_t idx;
    if (get_index(agent_id, &idx)){
        _ids[idx] = _self_id; // means no agent
        // No need to clear other vectors as the id is used to access them
    } else {
        std::cout << _name << ": Could not remove agent " << agent_id << ": Agent not found!" << std::endl;
    }
}

bool FullEKF::add_agent(const uint16_t agent_id, const float agent_rssi, const float x0, const float y0, const float var_x, const float var_y, uint16_t *idx){
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
        if (_last_seen[iIdx]-simtime_seconds>AGENT_TIMEOUT_MIN || _last_seen[iIdx]<best_last_seen){
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
        // TODO better state initialization (e.g. based on closest other agent?)
        // e.g. if we receive a direct message, check if there is a distance to another agent in there
        // atm initialize other agent in front at a distance based on message (most likely place to appear)
        
        // float range = -agent_rssi; // placeholder for a real case where rssi != -range
        // float norm_v = sqrtf(powf(rel_vx, 2) + powf(rel_vy, 2));

        // _state[best_idx][EKF_ST_X] = - range * (rel_vx/norm_v);
        // _state[best_idx][EKF_ST_Y] = - range * (rel_vy/norm_v);
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
        if (_stay_decoupled){
            std::cout << _name << _self_id << ": Added Ag" << agent_id 
                        << " at (" << x0 << "," << y0 << ") - variance ("
                        << var_x << "," << var_y << ")"  <<std::endl;
        }
        return true;
    } else {
        return false;
    }
}

void FullEKF::step(const float time, ekf_input_t &self_input){
    auto start = std::chrono::high_resolution_clock::now();

    if (time>=INITIALIZATION_PERIOD){
        _decouple_agents = _stay_decoupled;
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
            // _ag_init->add_velocities(_input_queue[iInput].id, _input_queue[iInput].vx, _input_queue[iInput].vy);
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
            // float rel_vx = _input_queue[iInput].vx - self_input.vx;
            // float rel_vy = _input_queue[iInput].vy - self_input.vy;
            // if (add_agent(_input_queue[iInput].id, _input_queue[iInput].rssi, rel_vx, rel_vy, &idx)){
            //     _input[idx][EKF_IN_VX] = _input_queue[iInput].vx;
            //     _input[idx][EKF_IN_VY] = _input_queue[iInput].vy;
            //     _rssi[idx] = _input_queue[iInput].rssi;
            //     _last_seen[idx] = _input_queue[iInput].timestamp;
            // }
        }
    }

    _input_queue.clear();
    if (time > _last_prediction_time + EKF_INTERVAL){
        predict(time, _state, _P);
        if (mirror_is_init){predict(time, _mirror_state, _mirror_P);}
        _last_prediction_time = time;

        // if (!assert_state_valid()){
        //     std::cout << _name << ": State Invalid" << std::endl;
        // }
        // if (!assert_covariance_valid()){
        //     std::cout << _name << ": Cov Invalid" << std::endl;
        // }
    }

    for (uint16_t iMeas=0; iMeas<_range_queue.size(); iMeas++){
        if (_range_queue[iMeas].id_A == _self_id){
            // std::cout << "Measurement Processed" << std::endl;
            update_with_direct_range(_range_queue[iMeas], _state, _P, &_direct_error);
            if(mirror_is_init){update_with_direct_range(_range_queue[iMeas], _mirror_state, _mirror_P, &_mirror_direct_error);}
        
        } else if (_range_queue[iMeas].id_B == _self_id) {
            _range_queue[iMeas].id_B = _range_queue[iMeas].id_A;
            _range_queue[iMeas].id_A = _self_id;
            update_with_direct_range(_range_queue[iMeas], _state, _P, &_direct_error);
            if(mirror_is_init){update_with_direct_range(_range_queue[iMeas], _mirror_state, _mirror_P, &_mirror_direct_error);}
        
        } else if (_decouple_agents == false) {
            update_with_indirect_range(_range_queue[iMeas], _state, _P);
            if(mirror_is_init){update_with_indirect_range(_range_queue[iMeas], _mirror_state, _mirror_P);}

        } else {
            continue;
        }
    }
    _range_queue.clear();
    clip_covariance(_P);
    if(mirror_is_init){clip_covariance(_mirror_P);}

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

    
    // // update the agent's state
    // for (uint16_t iAgent=0; iAgent<agent_data.size(); iAgent++){
    //     if (get_index(agent_data[iAgent].id, &idx)){
    //         agent_data[iAgent].state.relX = _state[idx][EKF_ST_X];
    //         agent_data[iAgent].state.relY = _state[idx][EKF_ST_Y];
    //         agent_data[iAgent].state.relPsi = _state[idx][EKF_ST_PSI];
    //     }
    // }

    // debugging
    // if (_self_id == 0){
    //     float det = fmat_det(_P);
    //     std::cout << "FullEKF: det=" << det << std::endl;
    // }

}


void FullEKF::predict(float time, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P){
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
            
            if((_decouple_agents == false) || (i==j)){
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


bool FullEKF::update_with_direct_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P, float* error_accum){
    bool success = false;
    if (meas.id_A != _self_id || meas.id_B == _self_id){
        return false;
    }
    uint16_t idx;
    if (get_index(meas.id_B, &idx)){
        float pred = std::sqrt(pow(state[idx][EKF_ST_X],2)+pow(state[idx][EKF_ST_Y],2));
        float error = meas.range-pred;
        *error_accum += abs(error);

        // Actually, H = [H0 .. Hi .. Hn], but only Hi != 0
        MatrixFloat HT(EKF_ST_DIM, 1);
        HT.data[EKF_ST_X][0] = state[idx][EKF_ST_X]/pred;
        HT.data[EKF_ST_Y][0] = state[idx][EKF_ST_Y]/pred;

        // when decoupled, only update state of the agent in question
        if(_decouple_agents){
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
        _ag_init->add_range(meas.id_B, meas.range, meas.timestamp);
    }
    return success;
}

bool FullEKF::update_with_indirect_range(const ekf_range_measurement_t &meas, std::vector<std::vector<float>> &state, std::vector<std::vector<MatrixFloat>> &P){
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

void FullEKF::clip_covariance(std::vector<std::vector<MatrixFloat>> &P){
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
            for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
                for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                    if (P[iAgent][jAgent].data[iRow][iCol] > COVARIANCE_MAX_VALUE){
                        P[iAgent][jAgent].data[iRow][iCol] = COVARIANCE_MAX_VALUE;
                    }
                }
            }
        }
    }
}

bool FullEKF::assert_covariance_valid(std::vector<std::vector<MatrixFloat>> &P){
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

void FullEKF::init_mirror() {
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

void FullEKF::select_mirror(){
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

