#include "single_ekf.h"

#include <math.h>

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>

#include "ekf_math.h"
#include "draw.h"
#include "trigonometry.h"
#include "log.h"

#define DEBUG_STATE_CHECK
#define AGENT_TIMEOUT_MIN 2 // earliest time after which an agent is discarded [s]

// static float stdev_initial_pos_xy = 100;
// static float stdev_initial_yaw = 3;

static float proc_noise_velXY = MEAS_NOISE_VX;
static float meas_noise_uwb = MEAS_NOISE_UWB;

SingleEKF::SingleEKF(const uint16_t nagents, const uint16_t selfID, const std::string name) 
            : RelLocEstimator(nagents, selfID, name){
    
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        // only add entries for other agents
        uint16_t id = iAgent;
        if (id>=_self_id){
            id++;
        }

        _ids[iAgent] = _self_id; // means no agent
        _P.push_back(MatrixFloat(EKF_ST_DIM, EKF_ST_DIM));

        _P[iAgent].data[EKF_ST_X][EKF_ST_X] = STDEV_INITIAL_POS_XY;
        _P[iAgent].data[EKF_ST_X][EKF_ST_Y] = 0;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_X] = 0;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] = STDEV_INITIAL_POS_XY;
        
        _state[iAgent][STATE_X] = 0;
        _state[iAgent][STATE_Y] = 0;
    }

    _ag_init = new AgentInitializer(_self_id);    
}

#ifdef DEBUG_STATE_CHECK
void SingleEKF::check_state_valid(){
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){

        if (isnan(_state[iAgent][EKF_ST_X]) ||
            isnan(_state[iAgent][EKF_ST_Y]) ||
            isinf(_state[iAgent][EKF_ST_X]) ||
            isinf(_state[iAgent][EKF_ST_Y])){
                throw std::runtime_error("Invalid state encoutered");
            }
        if (isnan(_P[iAgent].data[EKF_ST_X][EKF_ST_X]) ||
            isnan(_P[iAgent].data[EKF_ST_X][EKF_ST_Y]) ||
            isnan(_P[iAgent].data[EKF_ST_Y][EKF_ST_X]) ||
            isnan(_P[iAgent].data[EKF_ST_Y][EKF_ST_Y])){
                throw std::runtime_error("Invalid covariance encountered");
            }
    }
}
#else
void SingleEKF::check_state_valid(){
    return;
}
#endif

bool SingleEKF::add_agent(const uint16_t agent_id, const float agent_rssi, const float x0, const float y0, const float var_x, const float var_y, uint16_t *idx){
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
       
        _input[best_idx][EKF_IN_VX] = 0;
        _input[best_idx][EKF_IN_VY] = 0;
        
        for (uint16_t iRow=0; iRow<EKF_ST_DIM; iRow++){
            for (uint16_t iCol=0; iCol<EKF_ST_DIM; iCol++){
                _P[best_idx].data[iRow][iCol] = 0;
            }
        }

        _P[best_idx].data[EKF_ST_X][EKF_ST_X] = var_x;
        _P[best_idx].data[EKF_ST_Y][EKF_ST_Y] = var_y;

        std::cout << _name << _self_id << ": Added Ag" << agent_id
                    << " at (" << x0 << "," << y0 << ") - variance ("
                        << var_x << "," << var_y << ")" <<std::endl;
        return true;
    } else {
        return false;
    }
}

void SingleEKF::step(const float time, ekf_input_t &self_input){
    auto start = std::chrono::high_resolution_clock::now();
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
        }
    }
    _input_queue.clear();

    if (time > _last_prediction_time + EKF_INTERVAL){
        predict(time);
    }

    for (uint16_t iMeas=0; iMeas<_range_queue.size(); iMeas++){
        if (_range_queue[iMeas].id_A == _self_id){
            // std::cout << "Measurement Processed" << std::endl;
            update_with_direct_range(_range_queue[iMeas]);
        } else if (_range_queue[iMeas].id_B == _self_id) {
            _range_queue[iMeas].id_B = _range_queue[iMeas].id_A;
            _range_queue[iMeas].id_A = _self_id;
            update_with_direct_range(_range_queue[iMeas]);
        } else {
            continue;
        }
    }
    _range_queue.clear();

    clip_covariance();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    _performance.comp_time_us = duration.count();

    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        _cov[iAgent][EKF_ST_X] = _P[iAgent].data[EKF_ST_X][EKF_ST_X];
        _cov[iAgent][EKF_ST_Y] = _P[iAgent].data[EKF_ST_Y][EKF_ST_Y];
    }

    // // update the agent's state
    // for (uint16_t iAgent=0; iAgent<agent_data.size(); iAgent++){
    //     if (get_index(agent_data[iAgent].id, &idx)){
    //         agent_data[iAgent].state.relX = _state[idx][EKF_ST_X];
    //         agent_data[iAgent].state.relY = _state[idx][EKF_ST_Y];
    //         agent_data[iAgent].state.relPsi = _state[idx][EKF_ST_PSI];
    //     }
    // }
}

void SingleEKF::predict(float time){

    float dt = time-_last_prediction_time;
    _last_prediction_time = time;

    // Agent specific updates
    float d_state[EKF_ST_DIM];

    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        // Prediction Jacobian A = Identity
        
        // state change
        d_state[EKF_ST_X] = _input[iAgent][EKF_IN_VX] - _self_input[EKF_IN_VX];
        d_state[EKF_ST_Y] = _input[iAgent][EKF_IN_VY] - _self_input[EKF_IN_VY];
    
        _state[iAgent][EKF_ST_X] += dt * d_state[EKF_ST_X];
        _state[iAgent][EKF_ST_Y] += dt * d_state[EKF_ST_Y];

        // Propagate Covariance
        // Add own process noise
        _P[iAgent].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
        
        // Add other agent process noise
        _P[iAgent].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
    }

    check_state_valid();
}



bool SingleEKF::update_with_direct_range(const ekf_range_measurement_t &meas){
    bool success = false;
    
    if(meas.id_A != _self_id || meas.id_B == _self_id){
        return false;
    }

    uint16_t idx_i;
    if (get_index(meas.id_B, &idx_i)){
        float pred = std::sqrt(pow(_state[idx_i][EKF_ST_X],2)+pow(_state[idx_i][EKF_ST_Y],2));
        float error = meas.range-pred;
        
        MatrixFloat HT(EKF_ST_DIM, 1);
        HT.data[EKF_ST_X][0] = _state[idx_i][EKF_ST_X]/pred;
        HT.data[EKF_ST_Y][0] = _state[idx_i][EKF_ST_Y]/pred;

        MatrixFloat PHT(EKF_ST_DIM, 1);
        fmat_mult(_P[idx_i], HT, PHT);            

        float HPHT_R = meas_noise_uwb;
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            HPHT_R += HT.data[iState][0]*PHT.data[iState][0];
        }

        // state update (X = X + K(y-h), K=PH^T/(HPH^T+R)

        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            _state[idx_i][iState] += PHT.data[iState][0]*error/HPHT_R;
        }

        // Covariance update (P = P - KHP, KHP = PH^THP/(HPH^T+R))
        MatrixFloat tmpNN (EKF_ST_DIM, EKF_ST_DIM);
        for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
            for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                _P[idx_i].data[iRow][iCol] -= (PHT.data[iRow][0]*PHT.data[iCol][0])/HPHT_R;
            }
        }
        check_state_valid();
        success = true;
    } else {
        // agent not yet in state space
        _ag_init->add_range(meas.id_B, meas.range, meas.timestamp);
    }
    return success;
}


void SingleEKF::clip_covariance(){
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        for (unsigned int iRow=0; iRow<EKF_ST_DIM; iRow++){
            for (unsigned int iCol=0; iCol<EKF_ST_DIM; iCol++){
                if (_P[iAgent].data[iRow][iCol] > COVARIANCE_MAX_VALUE){
                    _P[iAgent].data[iRow][iCol] = COVARIANCE_MAX_VALUE;
                }
            }
        }
    }
}


void SingleEKF::animate(){
    // draw d;

    // // for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
    //     d.estimate(_state[EKF_ST_X], _state[EKF_ST_Y], _state[EKF_ST_PSI]);
    // // }

    // // test: calculate absolute position from estimate
    // float dx_g = agents[_other_id]->state[STATE_X] - agents[_self_id]->state[STATE_X];
    // float dy_g = agents[_other_id]->state[STATE_Y] - agents[_self_id]->state[STATE_Y];
    // float dyaw = agents[_other_id]->state[STATE_YAW] - agents[_self_id]->state[STATE_YAW];
    // float dx_l, dy_l;
    // rotate_g2l_xy(dx_g, dy_g, agents[_self_id]->state[STATE_YAW], dx_l, dy_l);
    // wrapToPi(dyaw);
}