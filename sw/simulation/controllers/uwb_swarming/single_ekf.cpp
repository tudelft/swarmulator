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

// static float stdev_initial_pos_xy = 100;
// static float stdev_initial_yaw = 3;

static float proc_noise_velXY = MEAS_NOISE_RHOX;
static float proc_noise_gyro = MEAS_NOISE_DPSI;

static float meas_noise_uwb = MEAS_NOISE_UWB;

SingleEKF::SingleEKF(const uint16_t nagents, const uint16_t selfID, const std::string name) 
            : RelLocEstimator(nagents, selfID, name){
    
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        // only add entries for other agents
        uint16_t id = iAgent;
        if (id>=_self_id){
            id++;
        }

        _ids[iAgent] = id;
        _P.push_back(MatrixFloat(EKF_ST_DIM, EKF_ST_DIM));

        _P[iAgent].data[EKF_ST_X][EKF_ST_X] = STDEV_INITIAL_POS_XY;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] = STDEV_INITIAL_POS_XY;
        _P[iAgent].data[EKF_ST_PSI][EKF_ST_PSI] = STDEV_INITIAL_YAW;
    }

    
#ifdef LOG
    std::stringstream header;
    header << "time, selfID, otherID, dx, dy, dpsi, est_dx, est_dy, est_dpsi, e_x, e_y, e_psi" << std::endl;
    _pFlogger->write_data(header);
#endif
    
}

#ifdef DEBUG_STATE_CHECK
void SingleEKF::check_state_valid(){
    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){

        if (isnan(_state[iAgent][EKF_ST_X]) ||
            isnan(_state[iAgent][EKF_ST_Y]) ||
            isnan(_state[iAgent][EKF_ST_PSI]) ||
            isinf(_state[iAgent][EKF_ST_X]) ||
            isinf(_state[iAgent][EKF_ST_Y]) ||
            isinf(_state[iAgent][EKF_ST_PSI])){
                throw std::runtime_error("Invalid state encoutered");
            }
        if (isnan(_P[iAgent].data[EKF_ST_X][EKF_ST_X]) ||
            isnan(_P[iAgent].data[EKF_ST_X][EKF_ST_Y]) ||
            isnan(_P[iAgent].data[EKF_ST_X][EKF_ST_PSI]) ||
            isnan(_P[iAgent].data[EKF_ST_Y][EKF_ST_X]) ||
            isnan(_P[iAgent].data[EKF_ST_Y][EKF_ST_Y]) ||
            isnan(_P[iAgent].data[EKF_ST_Y][EKF_ST_PSI]) ||
            isnan(_P[iAgent].data[EKF_ST_PSI][EKF_ST_X]) ||
            isnan(_P[iAgent].data[EKF_ST_PSI][EKF_ST_Y]) ||
            isnan(_P[iAgent].data[EKF_ST_PSI][EKF_ST_PSI])){
                throw std::runtime_error("Invalid covariance encountered");
            }
    }
}
#else
void SingleEKF::check_state_valid(){
    return;
}
#endif


void SingleEKF::step(const float time, ekf_input_t &self_input){
    auto start = std::chrono::high_resolution_clock::now();
    // add own input
    _self_input[EKF_IN_PX] = self_input.rhoX;
    _self_input[EKF_IN_PY] = self_input.rhoY;
    _self_input[EKF_IN_DPSI] = self_input.dPsi;

    // add other agent's input
    uint16_t idx;
    for (uint16_t iInput=0; iInput<_input_queue.size(); iInput++){
        if (get_index(_input_queue[iInput].id, &idx)){
            _input[idx][EKF_IN_PX] = _input_queue[iInput].rhoX;
            _input[idx][EKF_IN_PY] = _input_queue[iInput].rhoY;
            _input[idx][EKF_IN_DPSI] = _input_queue[iInput].dPsi;
        } else {
            std::cout << "SingleEKF: Couldn't add input for agent" << _input_queue[iInput].id << std::endl;
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
        } 
    }
    _range_queue.clear();

    clip_covariance();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    _performance.comp_time_us = duration.count();

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
    float sPsi, cPsi;
    MatrixFloat tmpSS1 (EKF_ST_DIM, EKF_ST_DIM);
    MatrixFloat tmpSS2 (EKF_ST_DIM, EKF_ST_DIM);

    for (uint16_t iAgent=0; iAgent<_n_agents; iAgent++){
        sPsi = std::sin(_state[iAgent][EKF_ST_PSI]);
        cPsi = std::cos(_state[iAgent][EKF_ST_PSI]);

        // Prediction Jacobian
        MatrixFloat A (EKF_ST_DIM, EKF_ST_DIM);
        A.data[EKF_ST_X][EKF_ST_X] = 1.0f;
        A.data[EKF_ST_X][EKF_ST_Y] = dt*_self_input[EKF_IN_DPSI];
        A.data[EKF_ST_X][EKF_ST_PSI] = dt*(-sPsi*_input[iAgent][EKF_IN_PX] - cPsi*_input[iAgent][EKF_IN_PY]);

        A.data[EKF_ST_Y][EKF_ST_X] = -dt*_self_input[EKF_IN_DPSI];
        A.data[EKF_ST_Y][EKF_ST_Y] = 1.0f;
        A.data[EKF_ST_Y][EKF_ST_PSI] = dt*(cPsi*_input[iAgent][EKF_IN_PX] - sPsi*_input[iAgent][EKF_IN_PY]);
        
        A.data[EKF_ST_PSI][EKF_ST_X] = 0.0f;
        A.data[EKF_ST_PSI][EKF_ST_Y] = 0.0f;
        A.data[EKF_ST_PSI][EKF_ST_PSI] = 1.0f;
        

        // Input Matrix
        // MatrixFloat B (EKF_ST_DIM, EKF_IN_DIM);
        // B.data[EKF_ST_X][EKF_IN_PX] = dt*cPsi;
        // B.data[EKF_ST_X][EKF_IN_PY] = -dt*sPsi;
        // B.data[EKF_ST_X][EKF_IN_DPSI] = 0.0f;

        // B.data[EKF_ST_Y][EKF_IN_PX] = dt*sPsi;
        // B.data[EKF_ST_Y][EKF_IN_PY] = dt*cPsi;
        // B.data[EKF_ST_Y][EKF_IN_DPSI] = 0.0f;

        // B.data[EKF_ST_PSI][EKF_IN_PX] = 0.0f;
        // B.data[EKF_ST_PSI][EKF_IN_PY] = 0.0f;
        // B.data[EKF_ST_PSI][EKF_IN_DPSI] = dt;


        // state change
        d_state[EKF_ST_X] = cPsi*_input[iAgent][EKF_IN_PX] - sPsi*_input[iAgent][EKF_IN_PY] - _self_input[EKF_IN_PX]
                            + _self_input[EKF_IN_DPSI]*_state[iAgent][EKF_ST_Y];
        d_state[EKF_ST_Y] = sPsi*_input[iAgent][EKF_IN_PX] + cPsi*_input[iAgent][EKF_IN_PY] - _self_input[EKF_IN_PY]
                            - _self_input[EKF_IN_DPSI]*_state[iAgent][EKF_ST_X];
        d_state[EKF_ST_PSI] = _input[iAgent][EKF_IN_DPSI] - _self_input[EKF_IN_DPSI]; 
    
        _state[iAgent][EKF_ST_X] += dt * d_state[EKF_ST_X];
        _state[iAgent][EKF_ST_Y] += dt * d_state[EKF_ST_Y];
        _state[iAgent][EKF_ST_PSI] += dt * d_state[EKF_ST_PSI];
        wrapTo2Pi(_state[iAgent][EKF_ST_PSI]);

        // Propagate Covariance
        fmat_mult(A, _P[iAgent], tmpSS1);   // Ai*Pij
        fmat_trans(A, tmpSS2);       // Aj^T
        fmat_mult(tmpSS1, tmpSS2, _P[iAgent]);             // Ai*Pij*Aj^T

        // Add own process noise
        _P[iAgent].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_PSI][EKF_ST_PSI] += proc_noise_gyro * dt * dt;
        
        // Add other agent process noise
        _P[iAgent].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
        _P[iAgent].data[EKF_ST_PSI][EKF_ST_PSI] += proc_noise_gyro * dt * dt;
    }

#ifdef LOG
    std::stringstream data;
    data << time << ", " << _self_id << ", " << _other_id << ", "
         << dx_l << ", " << dy_l << ", " << dyaw << ", "
         << _state[EKF_ST_X] << ", " << _state[EKF_ST_Y] << ", " << _state[EKF_ST_PSI] << ", "
         << ex << ", " << ey << ", " << epsi << std::endl;
    _pFlogger->write_data(data);
#endif
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
        HT.data[EKF_ST_PSI][0] = 0;

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