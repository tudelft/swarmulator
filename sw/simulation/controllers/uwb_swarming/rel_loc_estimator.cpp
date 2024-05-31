#include "rel_loc_estimator.h"

#include "draw.h"
#include "trigonometry.h"
#include "ekf_math.h"
#include "chi_squared_tables.h"

#define SECONDARY_OFF_INTERVAL  20
#define SECONDARY_OFF_TIME 10
static float next_secondary_switch = 0.0f;
static bool secondary_on = true;

#define EKF_INITIALIZATION_PERIOD 15 // don't use indirect measurements during initial startup
#define AGENT_INIT_PERIOD 5 // don't use indirect measurements from a recently added agent

#define REL_LOC_TIMEOUT 5 // time after which an agent can be discarded [s]
#define HYSTERESIS_FACTOR 1.1f // old agent must be that much further away than new agent to be discarded

#define NIS_WINDOW_SIZE 20 // Number of normalized innovation errors kept in memory
#define NIS_GRACE_PERIOD 10 // Time before agent can be removed based on bad NIS
#define COV_INFLATION_PERIOD 1000 // Covariance inflation only happens in the first few seconds

#define INPUT_TIMEOUT 1.0f // time after which input is set to 0
#define INPUT_MAX_NOISE 4.0f // maximum input covariance when input is unknown 

static float proc_noise_velXY = powf(MEAS_NOISE_VX, 2);
static float meas_noise_uwb_direct = powf(MEAS_NOISE_UWB, 2);
static float meas_noise_uwb_secondary = powf(MEAS_NOISE_UWB,2);

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
            _name = "reference ekf";
            _n_agents = nagents - 1;
            break;
        case ESTIMATOR_EKF_FULL:
            _name = "full ekf";
            _n_agents = nagents - 1;
            break;
        case ESTIMATOR_EKF_DYNAMIC:
            _name = "dynamic ekf";
            _n_agents = 10;
            break;
        case ESTIMATOR_EKF_DECOUPLED:
            _name = "decoupled ekf";
            _n_agents = nagents - 1;
            break;
        default:
            // _name = "none";
            // _n_agents = 0;
            break;
        }

    init();
}

RelLocEstimator::RelLocEstimator(const uint16_t self_id, uint8_t est_type, uint16_t n_agents, std::string name)
{
    _self_id = self_id;
    _est_type = est_type;
    if (est_type == ESTIMATOR_EKF_DYNAMIC){
        _n_agents = n_agents;
    } else {
        _n_agents = nagents - 1; // Only dynamic ekf can deal with varied n_agents
    }
    _name = name;

    init();
}

void RelLocEstimator::init(){
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
        _use_secondary_range.push_back(false);
        _ids.push_back(_self_id);
        _last_range.push_back(0.0f);
        _last_seen.push_back(0.0f);
        _agent_added_timestamp.push_back(0.0f);
        
        _P.push_back(std::vector<MatrixFloat>());
        for (uint16_t jAgent=0; jAgent<_n_agents; jAgent++){
            _P[iAgent].push_back(MatrixFloat(EKF_ST_DIM, EKF_ST_DIM));
        }

        _performance.mean_NIS_all = 1;
    }

    _ag_init = new AgentInitializer(_self_id);

    enable_all_ambiguity_improvements();
    reset();
}

/********************
 * Public Functions *
 ********************/

void RelLocEstimator::enable_all_ambiguity_improvements(){
    _enable_improved_initialization = true;
    _enable_selective_secondary_range = true;
    _enable_covariance_inflation = true;
    _enable_withhold_measurements = true;
    _enable_nis_agent_reset = true;
}

void RelLocEstimator::disable_all_ambiguity_improvements(){
    _enable_improved_initialization = false;
    _enable_selective_secondary_range = false;
    _enable_covariance_inflation = false;
    _enable_withhold_measurements = false;
    _enable_nis_agent_reset = false;
}

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
        if (ranges[iRange].id_A == _self_id || ranges[iRange].id_B == _self_id){        
            _direct_range_queue.push_back(ranges[iRange]);
        } else{
            _secondary_range_queue.push_back(ranges[iRange]);
        }
    }
}

void RelLocEstimator::step(const float time, ekf_input_t &self_input){
    auto start = std::chrono::high_resolution_clock::now();
    _current_time = time;

    // add own input
    _self_input[EKF_IN_VX] = self_input.vx;
    _self_input[EKF_IN_VY] = self_input.vy;

    // add other agent's input
    for (uint16_t iInput=0; iInput<_input_queue.size(); iInput++){
        process_input(_input_queue[iInput]);
    }
    _input_queue.clear();
    
    // Prediction step
    // if (_current_time > _last_prediction_time + EKF_INTERVAL){
        predict();
        _last_prediction_time = _current_time;
    // }

    // Measurement updates direct ranges first
    for (uint16_t iMeas=0; iMeas<_direct_range_queue.size(); iMeas++){
        update_with_direct_range(_direct_range_queue[iMeas]); 
    }
    _direct_range_queue.clear();

    // Measurement updates secondary ranges
    if ( (_est_type != ESTIMATOR_EKF_DECOUPLED)
        && (_current_time > _last_reset_time + EKF_INITIALIZATION_PERIOD)){
        if ((_enable_withhold_measurements == false) || secondary_on) {
            for (uint16_t iMeas=0; iMeas<_secondary_range_queue.size(); iMeas++){
                update_with_indirect_range(_secondary_range_queue[iMeas]);
            }
        }
    }
    _secondary_range_queue.clear();

    if (_current_time > next_secondary_switch){
        if (secondary_on){
            secondary_on = false;
            next_secondary_switch += SECONDARY_OFF_TIME;
        } else{
            secondary_on = true;
            next_secondary_switch += SECONDARY_OFF_INTERVAL;
        }
    }


    float nis_all = 0;
    uint16_t count = 0;
    for (uint16_t iAgent=0; iAgent < _n_agents; iAgent++){
        if ((_ids[iAgent] != _self_id) && (_current_time - _agent_added_timestamp[iAgent] > 2*NIS_GRACE_PERIOD)){
            nis_all += _mean_NIS[iAgent];
            count++;
        }
    }
    if (count > 0){
        _performance.mean_NIS_all = nis_all/count; // 20*_n_agents dof
        // _nis_all would have 20*_n_agents dof, so we're very conservative by using k=20
        // nis_all = nis_all/count;
        // if ( nis_all > CHI_SQUARED_20_0999/20){
        //     reset();
        // }
    }    

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
    reset_error_stats();
    float ex, ey, e_abs, dist, e_rel;
    uint16_t ekf_idx;

    float accumulator_eabs = 0.0f;
    float accumulator_erel = 0.0f;
    float e_max = 0;
    uint8_t count = 0;

    uint16_t N_icr = ids_in_comm_range_ordered.size();
    
    // calculate individual errors for agents in comm range
    for (uint16_t i=0; i<N_icr; i++){
        if(get_index(ids_in_comm_range_ordered[i], &ekf_idx)){
            if (_current_time > _agent_added_timestamp[ekf_idx] + 1.5f*AGENT_INIT_PERIOD){
                // Don't evaluate during init period
                ex = _state[ekf_idx][EKF_ST_X] - relX[i];
                ey = _state[ekf_idx][EKF_ST_Y] - relY[i];
                e_abs= sqrt(pow(ex, 2) + pow(ey, 2));
                
                dist = sqrtf(powf(relX[i],2)+powf(relY[i],2));
                e_rel = e_abs/dist;
                
                accumulator_eabs += e_abs;
                accumulator_erel += e_rel;
                count += 1;
                if (e_abs > e_max){
                    e_max = e_abs;
                }
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
            if (count > i){
                pErrorStats->abs_mean = accumulator_eabs/count;
                pErrorStats->rel_mean = accumulator_erel/count;
            } else {
                // no estimate for all N closest drones
                pErrorStats->abs_mean = -1;
                pErrorStats->rel_mean = -1;
            }
        }
    }
    // finally for ICR (all in comm range)
    _performance.icr.N = count;
    _performance.icr.max = e_max;
    if (count > 0){
        _performance.icr.abs_mean = accumulator_eabs/count;
        _performance.icr.rel_mean = accumulator_erel/count;
    } else {
        _performance.icr.abs_mean = -1;
        _performance.icr.rel_mean = -1;
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
            float new_agent_range;
            if (init_data.last_range > 0){
                new_agent_range = init_data.last_range;
            } else {
                new_agent_range = sqrtf(powf(init_data.x0, 2)+powf(init_data.y0, 2));
            }

            if (largest_range < HYSTERESIS_FACTOR*new_agent_range){
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
        if (_enable_improved_initialization){
            _use_secondary_range[best_idx] = false;
        }

        _agent_added_timestamp[best_idx] = init_data.timestamp;
        if (init_data.init_by_multilat){
            // If initialized with multilat, doesn't need as much time to converge
            _agent_added_timestamp[best_idx] -= AGENT_INIT_PERIOD/2;
            if (_self_id==0 && _est_type==ESTIMATOR_EKF_DYNAMIC){
                std::cout << (int)_current_time << " - [0dyn] Initialized " << init_data.id << ": Multilat" << std::endl;
            }
        } else {
            if (_self_id==0 && _est_type==ESTIMATOR_EKF_DYNAMIC){
                std::cout << (int)_current_time << " - [0dyn] Initialized " << init_data.id << ": Trajectory" << std::endl;
            }
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
    // init data for "lazy initialization"
    agent_initialization_data_t init_data;
    init_data.id = agent_id;
    init_data.timestamp = _current_time;

    switch (_est_type)
    {
    case ESTIMATOR_EKF_DECOUPLED:
        // use "lazy initialization"
        init_data.x0 = 0.0f;
        init_data.y0 = 0.0f;
        init_data.stdev_x = 100;
        init_data.stdev_y = 100;
        success = add_agent(init_data, idx);
        break;

    case ESTIMATOR_EKF_REF:
        // Perfect initialization
        init_data.x0 = agents[agent_id]->state[STATE_X] - agents[_self_id]->state[STATE_X];
        init_data.y0 = agents[agent_id]->state[STATE_Y] - agents[_self_id]->state[STATE_Y];
        init_data.stdev_x = 0.01;
        init_data.stdev_y = 0.01;
        success = add_agent(init_data, idx);
        break;
    
    default:
        if (_ag_init->get_initial_position(agent_id, _current_time, &init_data)){
            if (_enable_improved_initialization){
                // geometric initialization from multiple range measurements
                success = add_agent(init_data, idx);
            } else {
                // use "lazy initialization" (still need last range from init_data)
                init_data.x0 = 0.0f;
                init_data.y0 = 0.0f;
                init_data.stdev_x = 100;
                init_data.stdev_y = 100;
                success = add_agent(init_data, idx);
            }
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
        if (_current_time > _last_seen[iAgent] + INPUT_TIMEOUT){
            _input[iAgent][EKF_IN_VX] = 0;
            _input[iAgent][EKF_IN_VY] = 0;
        }

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
        if (_current_time < _last_seen[i] + INPUT_TIMEOUT){
            _P[i][i].data[EKF_ST_X][EKF_ST_X] += proc_noise_velXY * dt * dt;
            _P[i][i].data[EKF_ST_Y][EKF_ST_Y] += proc_noise_velXY * dt * dt;
        } else {
            _P[i][i].data[EKF_ST_X][EKF_ST_X] += INPUT_MAX_NOISE * dt * dt;
            _P[i][i].data[EKF_ST_Y][EKF_ST_Y] += INPUT_MAX_NOISE * dt * dt;
        }
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

        // Update NIS
        float HPHT = 0;
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            for (uint16_t jState=0; jState<EKF_ST_DIM; jState++){
                HPHT += HT.data[iState][0]*_P[idx][idx].data[iState][jState]*HT.data[jState][0];
            }
        }
        float nis = error*error/(HPHT + meas_noise_uwb_direct);
        
        for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE-1; iNIS++){
            _NIS[idx][iNIS] = _NIS[idx][iNIS+1];
        }
        _NIS[idx][NIS_WINDOW_SIZE-1] = nis;

        float sum_NIS = 0;
        for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE; iNIS++){
            sum_NIS += _NIS[idx][iNIS];
        }
        _mean_NIS[idx] = sum_NIS/NIS_WINDOW_SIZE;
        if ((_enable_selective_secondary_range) 
                && (_mean_NIS[idx] > CHI_SQUARED_20_0900/20.0f)) {
            _use_secondary_range[idx] = false;
        } else if ((_enable_improved_initialization) 
                && (_current_time < _agent_added_timestamp[idx]+AGENT_INIT_PERIOD)){
            _use_secondary_range[idx] = false;
        } else {
            _use_secondary_range[idx] = true;
        }

        // if ((_mean_NIS[idx] < CHI_SQUARED_20_0900/20.0f) &&
        //         (_current_time > _agent_added_timestamp[idx]+AGENT_INIT_PERIOD)){
        //     _use_secondary_range[idx] = true;
        // } else {
        //     _use_secondary_range[idx] = false;
        // }

        // if ( _est_type != ESTIMATOR_EKF_DECOUPLED && (sum_NIS > 45.3) && 
        if ( _enable_nis_agent_reset 
                && (_est_type == ESTIMATOR_EKF_FULL || _est_type == ESTIMATOR_EKF_DYNAMIC) 
                && (_current_time > _agent_added_timestamp[idx] + NIS_GRACE_PERIOD)
                && (sum_NIS > 150)){ 
            // remove agent to reinitialize
            remove_agent(id_B);
            if(_self_id == 0 && _est_type==ESTIMATOR_EKF_DYNAMIC){
                std::cout << (int) _current_time << " - [0dyn] Removed Ag" << id_B << ": mean NIS="<<_mean_NIS[idx]<<std::endl;
            }
            return false;
        }

        // Covariance inflation
        if ((_enable_covariance_inflation)
                && (_est_type == ESTIMATOR_EKF_FULL || _est_type == ESTIMATOR_EKF_DYNAMIC)){
            if (nis > CHI_SQUARED_1_0999){
                float beta = 2;
                if (nis < 1){
                    beta = powf((1+sqrtf(nis)),2)/(1+nis);
                }
                float U = beta * (meas_noise_uwb_direct + error*error);
                float alpha = U/HPHT;
                alpha = std::max(1.0f, alpha/9);
                fmat_scalar_mult(_P[idx][idx], alpha);
                _use_secondary_range[idx] = false;
            }
        }

        // when decoupled, only update state of the agent in question
        if(_est_type == ESTIMATOR_EKF_DECOUPLED){
            MatrixFloat PHT(EKF_ST_DIM,1);
            fmat_mult(_P[idx][idx], HT, PHT);

            float HPHT_R = meas_noise_uwb_direct;
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

            // // normalized innovation squared update
            // for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE-1; iNIS++){
            //     _NIS[idx][iNIS] = _NIS[idx][iNIS+1];
            // }
            // _NIS[idx][NIS_WINDOW_SIZE-1] = error*error/HPHT_R;

        } else{
            // PHT = [P0i HiT .. Pii HiT .. Pni HiT]T
            std::vector<MatrixFloat> PHT;
            for (uint16_t iRow=0; iRow<_n_agents; iRow++){
                PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
                fmat_mult(_P[iRow][idx], HT, PHT[iRow]);
            }

            float HPHT = 0;
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                HPHT += HT.data[iState][0]*PHT[idx].data[iState][0];
            }
            float HPHT_R = HPHT + meas_noise_uwb_direct;

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
        // Check if either agent still initializing
        if ( _enable_improved_initialization 
            && (meas.timestamp < _agent_added_timestamp[agent_i]+AGENT_INIT_PERIOD)
            && (meas.timestamp < _agent_added_timestamp[agent_j]+AGENT_INIT_PERIOD)){
                return false;
        }

        // // check if agent is still initializing
        // bool agent_i_valid = (meas.timestamp > _agent_added_timestamp[agent_i]+AGENT_INIT_PERIOD);
        // bool agent_j_valid = (meas.timestamp > _agent_added_timestamp[agent_j]+AGENT_INIT_PERIOD);
        
        // // Check if either agent has high NIS
        // agent_i_valid &= (_mean_NIS[agent_i] < CHI_SQUARED_20_0950/20.0f);
        // agent_j_valid &= (_mean_NIS[agent_j] < CHI_SQUARED_20_0950/20.0f);
        // if (agent_i_valid == false || agent_j_valid == false){
        //     return false;
        // }
        if ( (_use_secondary_range[agent_i] == false)
            && (_use_secondary_range[agent_j] == false) ){
            return false;
        }
        float pred = std::sqrt(pow(_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X],2)
                                +pow(_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y],2));
        
        float error = meas.range-pred;
        
        // H = [0 .. Hi .. 0 .. Hj .. 0]
        MatrixFloat HiT(EKF_ST_DIM, 1);
        fmat_set_zero(HiT);
        if(_use_secondary_range[agent_j]){
            // Agent j valid >> update agent i
            HiT.data[EKF_ST_X][0] = -(_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X])/pred;
            HiT.data[EKF_ST_Y][0] = -(_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y])/pred;
        }

        MatrixFloat HjT(EKF_ST_DIM, 1);
        fmat_set_zero(HjT);
        if(_use_secondary_range[agent_i]){
            // Agent i valid >> update agent j
            HjT.data[EKF_ST_X][0] = (_state[agent_j][EKF_ST_X]-_state[agent_i][EKF_ST_X])/pred;
            HjT.data[EKF_ST_Y][0] = (_state[agent_j][EKF_ST_Y]-_state[agent_i][EKF_ST_Y])/pred;
        }

        // Covariance inflation only during init period
        if ((_enable_covariance_inflation)
                && (_est_type == ESTIMATOR_EKF_FULL || _est_type == ESTIMATOR_EKF_DYNAMIC)){
            float HPHT_i = 0;
            float HPHT_2ij = 0;
            float HPHT_j = 0;
            for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
                for (uint16_t jState=0; jState<EKF_ST_DIM; jState++){
                    HPHT_i += HiT.data[iState][0]*_P[agent_i][agent_i].data[iState][jState]*HiT.data[jState][0];
                    HPHT_2ij += 2*HiT.data[iState][0]*_P[agent_i][agent_j].data[iState][jState]*HjT.data[jState][0];
                    HPHT_j += HjT.data[iState][0]*_P[agent_j][agent_j].data[iState][jState]*HjT.data[jState][0];
                }
            }

            float nis = error*error/(HPHT_i + HPHT_2ij + HPHT_j + meas_noise_uwb_secondary);
            // if (true){
            if (nis > CHI_SQUARED_1_0999){
                // 99% confidence
                float beta = 2;
                if (nis < 1){
                    beta = powf((1+sqrtf(nis)),2)/(1+nis);
                }
                float U = beta * (meas_noise_uwb_secondary + error*error);
                float alpha = U/(HPHT_i+HPHT_2ij+HPHT_j);
                // float alpha = (U-HPHT_2ij)/(HPHT_i+HPHT_j);
                // inflate so that measurement is in 2sigma
                alpha = std::max(1.0f, alpha/9);

                if (_use_secondary_range[agent_j]){
                    fmat_scalar_mult(_P[agent_i][agent_i], alpha);
                }
                if (_use_secondary_range[agent_i]){
                    fmat_scalar_mult(_P[agent_j][agent_j], alpha);
                }
                if (_use_secondary_range[agent_i] && _use_secondary_range[agent_j]){
                    fmat_scalar_mult(_P[agent_i][agent_j], alpha);
                    fmat_scalar_mult(_P[agent_j][agent_i], alpha);
                }
            }
        }
        std::vector<MatrixFloat> PHT;
        MatrixFloat tmp1(EKF_ST_DIM, 1);
        MatrixFloat tmp2(EKF_ST_DIM, 1);
        for (uint16_t iRow=0; iRow<_n_agents; iRow++){
            PHT.push_back(MatrixFloat(EKF_ST_DIM, 1));
            fmat_mult(_P[iRow][agent_i], HiT, tmp1);
            fmat_mult(_P[iRow][agent_j], HjT, tmp2);
            fmat_add(tmp1, tmp2, PHT[iRow]);
        }

        float HPHT = 0;
        for (uint16_t iState=0; iState<EKF_ST_DIM; iState++){
            HPHT += HiT.data[iState][0]*PHT[agent_i].data[iState][0];
            HPHT += HjT.data[iState][0]*PHT[agent_j].data[iState][0];
        }
        float HPHT_R = HPHT + meas_noise_uwb_secondary;

        // // inflate covariance to account for unmodelled correlations
        // float nis = error*error/HPHT_R;
        // float beta = 2;
        // if (nis<1){
        //     beta = powf((1+sqrtf(nis)),2)/(1+nis);
        // }
        // HPHT_R = beta*(HPHT + error*error) + meas_noise_uwb_direct;

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

        // // normalized innovation squared update
        // for (uint16_t iNIS=0; iNIS<NIS_WINDOW_SIZE-1; iNIS++){
        //     _NIS[agent_i][iNIS] = _NIS[agent_i][iNIS+1];
        //     _NIS[agent_j][iNIS] = _NIS[agent_j][iNIS+1];
        // }
        // _NIS[agent_i][NIS_WINDOW_SIZE-1] = error*error/HPHT_R;
        // _NIS[agent_j][NIS_WINDOW_SIZE-1] = error*error/HPHT_R;

        success = true;
    } else {
        // at least one agent not in state space
        multilat_point_t mlp;
        if (get_index(meas.id_A, &agent_i)){
            // agent A is know
            float stdev_x = sqrtf(_P[agent_i][agent_i].data[EKF_ST_X][EKF_ST_X]);
            float stdev_y = sqrtf(_P[agent_i][agent_i].data[EKF_ST_Y][EKF_ST_Y]);
            if (stdev_x < 0.2 && stdev_y < 0.2 && _mean_NIS[agent_i]<1){
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
            if (stdev_x < 0.2 && stdev_y < 0.2 && _mean_NIS[agent_i]<1){
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

void RelLocEstimator::reset_error_stats(){
    // set errors to negative value to make it clear when
    // there are not enough valid estimates
    _performance.c1.max = -1;
    _performance.c1.abs_mean = -1;
    _performance.c1.rel_mean = -1;
    _performance.c1.N = 0;

    _performance.c3.max = -1;
    _performance.c3.abs_mean = -1;
    _performance.c3.rel_mean = -1;
    _performance.c3.N = 0;

    _performance.c5.max = -1;
    _performance.c5.abs_mean = -1;
    _performance.c5.rel_mean = -1;
    _performance.c5.N = 0;

    _performance.icr.max = -1;
    _performance.icr.abs_mean = -1;
    _performance.icr.rel_mean = -1;
    _performance.icr.N = 0;
}
