#include "swarm_ranging.h"

#include <algorithm>

#include "main.h"
#include "uwb_channel.h"
#include "ekf_types.h"
#include "draw.h"
#include "trigonometry.h"

#define RANGING_EVALUATION_DT 1 // seconds
#define ANIMATION_TIMEOUT 0.5f


SwarmRanging::SwarmRanging(){
    for (uint8_t i=0; i<3; i++){
        _tx_history[i].full = 0;
        _seq_history[i] = 0;
    }
}

void SwarmRanging::init(const uint16_t ID){
    _self_id = ID;
    environment.uwb_channel.join(_self_id);
}


void SwarmRanging::order_by_rssi(){
    /**
     *  order by rssi (insertion sort since we assume the array is already
     *  close to ordered)
     */
    for (uint16_t i_tbs=1; i_tbs<_agents.size(); i_tbs++){
        uint16_t i_now = i_tbs;
        while (i_now>0 && (_agents[i_now].rssi>_agents[i_now-1].rssi)){
            std::swap(_agents[i_now], _agents[i_now-1]);
            i_now--;
        }
    }
}

uint16_t SwarmRanging::get_storage_index(const uint16_t agent_id){
    bool agent_known = false;
    uint16_t storage_idx;
    for (uint16_t idx=0; idx<_agents.size(); idx++){
        if ( _agents[idx].id == agent_id){
            storage_idx = idx;
            agent_known = true;
            break;
        }
    }

    if (!agent_known){
        ranging_table_t new_agent = {.id = agent_id};
        _agents.push_back(new_agent);
        storage_idx = _agents.size()-1;
    }
    return storage_idx;
}

bool SwarmRanging::get_tx_time(const uint8_t seq, uwb_time_t &tx_time){
    bool seq_is_known = false;
    tx_time.full = 0;
    
    for (uint8_t i_hist=0; i_hist<TX_HISTORY_LEN; i_hist++){
        if (seq == _seq_history[i_hist]){
            tx_time = _tx_history[i_hist];
            seq_is_known = true;
        };
    }

    return seq_is_known;
}

void SwarmRanging::add_timestamps_response(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block){
    // Always set reception timestamp to initiate new ranging sequence when poll is too far back
    agentB.seq[RESPONSE] = src.seq;
    memcpy(&agentB.rx_time[RESPONSE].raw, &msg_rx.raw, 5);

    if (get_tx_time(msg_agent_block.last_seq, agentB.tx_time[POLL])){
        agentB.seq[POLL] = msg_agent_block.last_seq;
        memcpy(&agentB.rx_time[POLL].raw, &msg_agent_block.last_rx, 5);
        // std::cout << _self_id << " - add response success " << agentB.id << "|" << (int) src.seq << std::endl;
    } else {
        // poll too far back, remove poll information from table
        agentB.tx_time[POLL].full = 0;
        agentB.rx_time[POLL].full = 0;
        // std::cout << _self_id << " - add response failed " << agentB.id << "|" << (int) src.seq << std::endl;
    }
}

void SwarmRanging::add_timestamps_report(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block){
    if (src.seq != agentB.seq[RESPONSE]+1){
        // message does not contain response tx timestamp, add as new response
        // std::cout << _self_id << " - add report failed " << agentB.id << "|" << (int) src.seq << std::endl;
        add_timestamps_response(agentB, msg_rx, src, msg_agent_block);
    } else {
        if (get_tx_time(msg_agent_block.last_seq, agentB.tx_time[FINAL])){
            // Add as report if tx timestamp of last msg still known
            agentB.seq[FINAL] = msg_agent_block.last_seq;
            agentB.seq[REPORT] = src.seq;

            memcpy(&agentB.tx_time[RESPONSE].raw, &src.last_tx, 5);
            memcpy(&agentB.rx_time[FINAL].raw, &msg_agent_block.last_rx, 5);
            memcpy(&agentB.rx_time[REPORT].raw, &msg_rx.raw, 5);
            // std::cout << _self_id << " - add report success " << agentB.id << "|" << (int) src.seq << std::endl;
        } else {
            // last transmission to long ago, clear ranging table and start again with this msg
            // std::cout << _self_id << " - add report failed " << agentB.id << "|" << (int) src.seq << std::endl;
            clear_timestamps(agentB);
            agentB.seq[RESPONSE] = src.seq;
            memcpy(&agentB.rx_time[RESPONSE].raw, &msg_rx.raw, 5);
        }
    } 
}

void SwarmRanging::clear_timestamps(ranging_table_t &agentB){
    for (uint8_t i=0; i<3; i++){
        agentB.tx_time[i].full = 0;
        agentB.rx_time[i].full = 0;
    }
    agentB.rx_time[4].full = 0;
}

void SwarmRanging::shift_timestamps(ranging_table_t &agentB){
    agentB.rx_time[POLL] = agentB.rx_time[FINAL];
    agentB.tx_time[POLL] = agentB.tx_time[FINAL];
    agentB.seq[POLL] = agentB.seq[FINAL];
    agentB.rx_time[RESPONSE] = agentB.rx_time[REPORT];
    agentB.tx_time[RESPONSE].full = 0;
    agentB.seq[RESPONSE] = agentB.seq[REPORT];
    agentB.rx_time[FINAL].full = 0;
    agentB.tx_time[FINAL].full = 0;
    agentB.rx_time[REPORT].full = 0;
    agentB.seq[REPORT]= 0;
}



void SwarmRanging::send_ranging_ping(float rhoX, float rhoY, float dPsi)
{
    // Initialize Swarm Ranging Ping
    swarm_ranging_ping_t tx_ping;
    memset(&tx_ping, 0, sizeof(swarm_ranging_ping_t));
    tx_ping.header_s.sourceAddress = (uint64_t) _self_id;
    
    tx_ping.sender.seq = _seq_history[0] + 1;
    memcpy(&tx_ping.sender.last_tx, _tx_history[0].raw, 5);
    
    // Convert inputs to 16 bit integers to save bandwidth
    tx_ping.sender.velocities[0] = (int16_t)(rhoX * 1000);
    tx_ping.sender.velocities[1] = (int16_t)(rhoY * 1000);
    tx_ping.sender.yawrate = (int16_t)(dPsi * 1000);

    // Pick agents to include in ping
    order_by_rssi();
    tx_ping.n_agents = std::min((uint16_t)(_agents.size()), (uint16_t) MAX_AGENTS_IN_PING);
    
    for (uint8_t i_agent=0; i_agent<tx_ping.n_agents; i_agent++){
        tx_ping.agents[i_agent].id = _agents[i_agent].id;
        tx_ping.agents[i_agent].last_range = _agents[i_agent].last_range_mm;
        _agents[i_agent].last_range_mm = 0; // Only send range once when it is fresh

        if (_agents[i_agent].rx_time[REPORT].full != 0){
            memcpy(&tx_ping.agents[i_agent].last_rx, _agents[i_agent].rx_time[REPORT].raw, 5);
            tx_ping.agents[i_agent].last_seq = _agents[i_agent].seq[REPORT];
        } else if (_agents[i_agent].rx_time[RESPONSE].full != 0){
            memcpy(&tx_ping.agents[i_agent].last_rx, _agents[i_agent].rx_time[RESPONSE].raw, 5);
            tx_ping.agents[i_agent].last_seq = _agents[i_agent].seq[RESPONSE];
        } else {
            memset(&tx_ping.agents[i_agent].last_rx, 0, 5);
            tx_ping.agents[i_agent].last_seq = 0;
        }
    }

    environment.uwb_channel.send_srp(tx_ping);

    // progress the tx history (on the drone, this should happen in the tx callback)
    for (uint8_t i=TX_HISTORY_LEN-1; i>0; i--){
        _tx_history[i] = _tx_history[i-1];
        _seq_history[i] = _seq_history[i-1];
    }
    _seq_history[0] = tx_ping.sender.seq;
    _tx_history[0].full = 1; // actual tx timestamp
}


void SwarmRanging::receive_new_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs)
{
    std::vector<swarm_ranging_ping_t> rx_pings;
    std::vector<float> rx_rssi;
    environment.uwb_channel.receive_srp(_self_id, &rx_pings, &rx_rssi);
    uwb_time_t rx_timestamp = {.full = 1};

    if (rx_pings.size() == rx_rssi.size()){
        for (uint16_t i_ping=0; i_ping<rx_pings.size(); i_ping++){
            process_ranging_ping(rx_timestamp, rx_pings[i_ping], rx_rssi[i_ping], inputs);
            _evaluator.add_message_by_payload(SRP_SIZE_BITS(rx_pings[i_ping].n_agents));
        }
    }

    calculate_direct_ranges();

    for (uint16_t i_range=0; i_range<_range_measurements.size(); i_range++){
        ranges.push_back(_range_measurements[i_range]);
        _animation_buffer.push_back(_range_measurements[i_range]);
    }
    _range_measurements.clear();
    
}

void SwarmRanging::receive_all_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs){
    std::vector<swarm_ranging_ping_t> rx_pings;
    environment.uwb_channel.receive_all_srp(&rx_pings);
    for (uint16_t iPing=0; iPing<rx_pings.size(); iPing++){
        if (rx_pings[iPing].header_s.sourceAddress != _self_id){
            ekf_input_t new_input = {.id = (uint16_t) rx_pings[iPing].header_s.sourceAddress,
                                .rhoX = rx_pings[iPing].sender.velocities[0]/1000.0f,
                                .rhoY = rx_pings[iPing].sender.velocities[1]/1000.0f,
                                .dPsi = rx_pings[iPing].sender.yawrate/1000.0f};
            inputs.push_back(new_input);
        }
        
        for (uint8_t iAgent=0; iAgent<rx_pings[iPing].n_agents; iAgent++){
            if (rx_pings[iPing].agents[iAgent].last_range != 0){
                ekf_range_measurement_t new_range = {.id_A = (uint16_t) rx_pings[iPing].header_s.sourceAddress,
                                                   .id_B = rx_pings[iPing].agents[iAgent].id,
                                                   .range = rx_pings[iPing].agents[iAgent].last_range/1000.0f,
                                                   .timestamp = simtime_seconds};
                ranges.push_back(new_range);
            }    
        }
    }
}


void SwarmRanging::process_ranging_ping(uwb_time_t &rx_time, const swarm_ranging_ping_t &msg, float rssi, std::vector<ekf_input_t> &inputs)
{
    uint16_t src_id = (uint16_t) msg.header_s.sourceAddress;
    uint16_t storage_idx = get_storage_index(src_id);

    _agents[storage_idx].rssi = rssi;
    _agents[storage_idx].t_last_seen = simtime_seconds;

    ekf_input_t new_input = {.id = (uint16_t) msg.header_s.sourceAddress,
                            .rhoX = msg.sender.velocities[0]/1000.0f,
                            .rhoY = msg.sender.velocities[1]/1000.0f,
                            .dPsi = msg.sender.yawrate/1000.0f,
                            .rssi = rssi,
                            .timestamp = simtime_seconds};
    inputs.push_back(new_input);

    bool is_not_ranging = (_agents[storage_idx].tx_time[POLL].full == 0);
    for (uint8_t i_agent=0; i_agent<msg.n_agents; i_agent++){
        if(msg.agents[i_agent].id == _self_id){
            is_not_ranging = false;
            if(_agents[storage_idx].rx_time[RESPONSE].full == 0){
                // First message from this agent
                clear_timestamps(_agents[storage_idx]);
                add_timestamps_response(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
            } else if(_agents[storage_idx].rx_time[REPORT].full != 0){
                // Ranging Table is full already
                shift_timestamps(_agents[storage_idx]);
                add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);                
            } else {
                add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
            }
        } //else {
            // enqueue secondary distance measurement
            if (msg.agents[i_agent].last_range != 0){
                ekf_range_measurement_t new_range = {.id_A = (uint16_t) msg.header_s.sourceAddress,
                                                   .id_B = msg.agents[i_agent].id,
                                                   .range = msg.agents[i_agent].last_range/1000.0f,
                                                   .timestamp = simtime_seconds};
                _range_measurements.push_back(new_range);
            }
        //}
    }

    if (is_not_ranging){
        // keep sequence number and rx timestamp
        _agents[storage_idx].seq[RESPONSE] = msg.sender.seq;
        memcpy(&_agents[storage_idx].rx_time[RESPONSE].raw, &rx_time.raw, 5);
    }
}


void SwarmRanging::calculate_direct_ranges()
{
    for (uint16_t i_agent=0; i_agent<_agents.size(); i_agent++){
        if(_agents[i_agent].tx_time[POLL].full == 0 ||
             _agents[i_agent].rx_time[POLL].full == 0 ||
             _agents[i_agent].tx_time[RESPONSE].full == 0 ||
             _agents[i_agent].rx_time[RESPONSE].full == 0 ||
             _agents[i_agent].tx_time[FINAL].full == 0 ||
             _agents[i_agent].rx_time[FINAL].full == 0 ||
             _agents[i_agent].rx_time[REPORT].full == 0){
            continue;
        } else {
            /*
            // Calculate tof & distance
            float roundA = _agents[i_agent].rx_time[RESPONSE] - _agents[i_agent].tx_time[POLL];
            float replyA = _agents[i_agent].tx_time[FINAL] - _agents[i_agent].rx_time[RESPONSE];

            float roundB = _agents[i_agent].rx_time[FINAL] - _agents[i_agent].tx_time[RESPONSE];
            float replyB = _agents[i_agent].tx_time[RESPONSE] - _agents[i_agent].rx_time[POLL];

            float tof = (roundA*roundB - replyA*replyB)/(roundA + replyA + roundB + replyB);
            
            _agents[i_agent].range = tof * SPEED_OF_LIGHT;
            */
            uint8_t id = _agents[i_agent].id;
            main_mutex.lock_shared();
            float range = sqrt(pow(agents[_self_id]->state[STATE_X] - agents[id]->state[STATE_X], 2)
                                        + pow(agents[_self_id]->state[STATE_Y] - agents[id]->state[STATE_Y], 2));
            main_mutex.unlock_shared();


            range += _rg.gaussian_float(0,MEAS_NOISE_UWB);
            _agents[i_agent].last_range_mm = (uint16_t) (range*1000);
            _agents[i_agent].t_last_range = simtime_seconds;

            ekf_range_measurement_t meas = {.id_A = _self_id,
                                        .id_B = _agents[i_agent].id,
                                        .range = range,
                                        .timestamp = simtime_seconds};
            _range_measurements.push_back(meas);
            shift_timestamps(_agents[i_agent]);
        }
    }    
}

void SwarmRanging::rel_loc_animation(const uint16_t ID){
    draw d;
    float dx_g, dy_g, dx_l, dy_l;
    float dxA, dyA, dxB, dyB;
    float own_yaw = agents[ID]->state[STATE_YAW];

    // draw pings
    bool has_ping, has_range;
    for (uint16_t i_agent=0; i_agent<_agents.size(); i_agent++){
        has_ping = (_agents[i_agent].t_last_seen>simtime_seconds-ANIMATION_TIMEOUT);
        has_range = (_agents[i_agent].t_last_range>simtime_seconds-ANIMATION_TIMEOUT);

        if(has_ping && !(has_range)){
            dx_g = agents[i_agent]->state[STATE_X] - agents[ID]->state[STATE_X];
            dy_g = agents[i_agent]->state[STATE_Y] - agents[ID]->state[STATE_Y];
            rotate_g2l_xy(dx_g, dy_g, own_yaw, dx_l, dy_l);
            d.segment(0.0f, 0.0f,dx_l, dy_l, orange);
        }
    }

    // draw ranging
    uint16_t idA, idB;
    for (int i_meas=_animation_buffer.size()-1; i_meas>=0; i_meas--){
        if (_animation_buffer[i_meas].timestamp>simtime_seconds-ANIMATION_TIMEOUT){
            idA = _animation_buffer[i_meas].id_A;
            idB = _animation_buffer[i_meas].id_B;

            dx_g = agents[idA]->state[STATE_X] - agents[ID]->state[STATE_X];
            dy_g = agents[idA]->state[STATE_Y] - agents[ID]->state[STATE_Y];
            rotate_g2l_xy(dx_g, dy_g, own_yaw, dxA, dyA);

            dx_g = agents[idB]->state[STATE_X] - agents[ID]->state[STATE_X];
            dy_g = agents[idB]->state[STATE_Y] - agents[ID]->state[STATE_Y];
            rotate_g2l_xy(dx_g, dy_g, own_yaw, dxB, dyB);

            d.segment(dxA, dyA, dxB, dyB, green);
        } else {
            // measurement is old
            _animation_buffer.erase(_animation_buffer.begin()+i_meas);
        }
    }
}


RangingEvaluator::RangingEvaluator()
{
    _airtime_accum_ns = 0;
    _total_airtime_ns = 0;

    _time_elapsed = 0;
    _last_load_calculation = 0;
    
    _current_load = 0;
    _avg_load = 0;
}

void RangingEvaluator::add_message_by_payload(uint16_t payload_bits){
    uint64_t airtime = (UWB_PHY_HEADER_TIME_ns + (uint64_t)(1e9*payload_bits/UWB_DATARATE_bps));
    _airtime_accum_ns += airtime;
    _total_airtime_ns += airtime;
}

void RangingEvaluator::calculate_load()
{
    if (_last_load_calculation < simtime_seconds - RANGING_EVALUATION_DT){
        float dt = (simtime_seconds - _last_load_calculation);
        _time_elapsed += dt;

        _current_load = (float)(1e-9 * _airtime_accum_ns) / dt;
        _avg_load = (float)(1e-9 * _total_airtime_ns) / _time_elapsed;

        _airtime_accum_ns = 0;
        _last_load_calculation = simtime_seconds;
    }
}