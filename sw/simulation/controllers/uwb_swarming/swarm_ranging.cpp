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
    _agents_in_tx_queue = 0;
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
        ranging_table_t new_agent = {.id = agent_id,
                                     .number_in_tx_queue = -1};
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

void SwarmRanging::process_remote_tx_time(ranging_table_t &agentB, const uwb_time_t &tx_time, const uint8_t seq){
    // remote tx time belongs to either response or report message
    if (agentB.seq[RESPONSE] == seq){
        memcpy(&agentB.tx_time[RESPONSE].raw, &tx_time, 5);
    } else if (agentB.seq[REPORT] == seq){
        memcpy(&agentB.tx_time[REPORT].raw, &tx_time, 5);
    }
}

void SwarmRanging::add_timestamps_response(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block){
    // Always set reception timestamp to initiate new ranging sequence when poll is too far back
    agentB.seq[RESPONSE] = src.seq;
    memcpy(&agentB.rx_time[RESPONSE].raw, &msg_rx.raw, 5);

    if (get_tx_time(msg_agent_block.last_seq, agentB.tx_time[POLL])){
        agentB.seq[POLL] = msg_agent_block.last_seq;
        memcpy(&agentB.rx_time[POLL].raw, &msg_agent_block.last_rx, 5);
    } else {
        // poll too far back, remove poll information from table
        agentB.tx_time[POLL].full = 0;
        agentB.rx_time[POLL].full = 0;
    }
}

void SwarmRanging::add_timestamps_report(ranging_table_t &agentB, const uwb_time_t &msg_rx, const srp_source_block_t &src, const srp_agent_block_t &msg_agent_block){
    if (agentB.tx_time[RESPONSE].full == 0){
        // We never got a tx time for the response, use this msg as response instead
        add_timestamps_response(agentB, msg_rx, src, msg_agent_block);
    } else {
        if (get_tx_time(msg_agent_block.last_seq, agentB.tx_time[FINAL])){
            // Add as report if tx timestamp of last msg still known
            agentB.seq[FINAL] = msg_agent_block.last_seq;
            agentB.seq[REPORT] = src.seq;

            // memcpy(&agentB.tx_time[RESPONSE].raw, &src.last_tx, 5);
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
    for (uint8_t i=0; i<MSG_TYPE_DIM; i++){
        agentB.tx_time[i].full = 0;
        agentB.rx_time[i].full = 0;
    }
}

void SwarmRanging::shift_timestamps(ranging_table_t &agentB){
    agentB.rx_time[POLL] = agentB.rx_time[FINAL];
    agentB.tx_time[POLL] = agentB.tx_time[FINAL];
    agentB.seq[POLL] = agentB.seq[FINAL];
    agentB.rx_time[RESPONSE] = agentB.rx_time[REPORT];
    agentB.tx_time[RESPONSE] = agentB.tx_time[REPORT];
    agentB.seq[RESPONSE] = agentB.seq[REPORT];
    agentB.rx_time[FINAL].full = 0;
    agentB.tx_time[FINAL].full = 0;
    agentB.rx_time[REPORT].full = 0;
    agentB.tx_time[REPORT].full = 0;
    // agentB.seq[REPORT]= 0;
}



void SwarmRanging::send_ranging_ping(float vx, float vy)
{
    // Initialize Swarm Ranging Ping
    swarm_ranging_ping_t tx_ping;
    memset(&tx_ping, 0, sizeof(swarm_ranging_ping_t));
    tx_ping.header_s.sourceAddress = (uint64_t) _self_id;
    
    tx_ping.sender.seq = _seq_history[0] + 1;
    memcpy(&tx_ping.sender.last_tx, _tx_history[0].raw, 5);
    
    // Convert inputs to 16 bit integers to save bandwidth
    tx_ping.sender.velocities[0] = (int16_t)(vx * 1000);
    tx_ping.sender.velocities[1] = (int16_t)(vy * 1000);

    // Pick agents to include in ping
 /*   
    order_by_rssi();
    tx_ping.n_agents = std::min((uint16_t)(_agents.size()), (uint16_t) MAX_AGENTS_IN_PING);
    for (uint8_t i_agent=0; i_agent<tx_ping.n_agents; i_agent++){
        tx_ping.agents[i_agent].id = _agents[i_agent].id;
        tx_ping.agents[i_agent].last_range = _agents[i_agent].last_range_mm;
        tx_ping.agents[i_agent].last_range = 0;
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
*/    

    tx_ping.n_agents = std::min(_agents_in_tx_queue, MAX_AGENTS_IN_PING);
    uint8_t ag_slot = 0;
    for (uint8_t iAgent=0; iAgent<_agents.size(); iAgent++){
        int n_tx_q = _agents[iAgent].number_in_tx_queue;
        if ((_agents[iAgent].number_in_tx_queue >=0)
                && (_agents[iAgent].number_in_tx_queue < tx_ping.n_agents) 
                && (ag_slot < tx_ping.n_agents)){
            // this agent needs to be in the message
            tx_ping.agents[ag_slot].id = _agents[iAgent].id;
            tx_ping.agents[ag_slot].last_range = _agents[iAgent].last_range_mm;
            _agents[iAgent].last_range_mm = 0; // Only send range once when it is fresh
            if (_agents[iAgent].rx_time[REPORT].full != 0){
                memcpy(&tx_ping.agents[ag_slot].last_rx, _agents[iAgent].rx_time[REPORT].raw, 5);
                tx_ping.agents[ag_slot].last_seq = _agents[iAgent].seq[REPORT];
            } else if (_agents[iAgent].rx_time[RESPONSE].full != 0){
                memcpy(&tx_ping.agents[ag_slot].last_rx, _agents[iAgent].rx_time[RESPONSE].raw, 5);
                tx_ping.agents[ag_slot].last_seq = _agents[iAgent].seq[RESPONSE];
            } else {
                memset(&tx_ping.agents[ag_slot].last_rx, 0, 5);
                tx_ping.agents[ag_slot].last_seq = 0;
            }

            _agents[iAgent].number_in_tx_queue = -1; // agent no longer in queue
            ag_slot++;  // move to next slot in ping
        } else if (_agents[iAgent].number_in_tx_queue > 0){
            // agent still in queue, but moves up
            _agents[iAgent].number_in_tx_queue -= tx_ping.n_agents;
        }
    }
    _agents_in_tx_queue -= tx_ping.n_agents;

    environment.uwb_channel.send_srp(tx_ping);

    // progress the tx history (on the drone, this should happen in the tx callback)
    for (uint8_t i=TX_HISTORY_LEN-1; i>0; i--){
        _tx_history[i] = _tx_history[i-1];
        _seq_history[i] = _seq_history[i-1];
    }
    _seq_history[0] = tx_ping.sender.seq;
    _tx_history[0].full = (uint64_t) (simtime_seconds*1000); // actual tx timestamp
}


void SwarmRanging::process_incoming_data(const float time_now)
{
    _direct_range_buffer.clear();
    _secondary_range_buffer.clear();
    _input_buffer.clear();

    // std::vector<swarm_ranging_ping_t> rx_pings;
    // std::vector<float> rx_rssi;
    // environment.uwb_channel.receive_srp(_self_id, &rx_pings, &rx_rssi);
    uwb_time_t rx_timestamp = {.full = (uint64_t) (time_now*1000)};

    // if (rx_pings.size() == rx_rssi.size()){
    //     for (uint16_t i_ping=0; i_ping<rx_pings.size(); i_ping++){
    //         process_ranging_ping(rx_timestamp, rx_pings[i_ping], rx_rssi[i_ping], _input_buffer, time_now);
    //         _evaluator.add_message_by_payload(SRP_SIZE_BITS(rx_pings[i_ping].n_agents));
    //     }
    // }
    _srp_buffer_mutex.lock();
    if (_srp_buffer.size() == _rssi_buffer.size()){
        for (uint16_t i_ping=0; i_ping<_srp_buffer.size(); i_ping++){
            process_ranging_ping(rx_timestamp, _srp_buffer[i_ping], _rssi_buffer[i_ping], _input_buffer, time_now);
            _evaluator.add_message_by_payload(SRP_SIZE_BITS(_srp_buffer[i_ping].n_agents));
        }
    }
    _srp_buffer.clear();
    _rssi_buffer.clear();
    _srp_buffer_mutex.unlock();

    calculate_direct_ranges(time_now);
    
    // update animation buffer
    for (uint16_t iRange=0; iRange<_direct_range_buffer.size(); iRange++){
        _animation_buffer.push_back(_direct_range_buffer[iRange]);
    }
    for (uint16_t iRange=0; iRange<_secondary_range_buffer.size(); iRange++){
        _animation_buffer.push_back(_secondary_range_buffer[iRange]);
    }
}

void SwarmRanging::get_new_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs)
{
    for (uint16_t iRange=0; iRange<_direct_range_buffer.size(); iRange++){
        ranges.push_back(_direct_range_buffer[iRange]);
    }

    for (uint16_t iRange=0; iRange<_secondary_range_buffer.size(); iRange++){
        ranges.push_back(_secondary_range_buffer[iRange]);
    }

    for(uint16_t iInput=0; iInput<_input_buffer.size(); iInput++){
        inputs.push_back(_input_buffer[iInput]);
    }
    
}

void SwarmRanging::get_all_ranging(std::vector<ekf_range_measurement_t> &ranges, std::vector<ekf_input_t> &inputs, const float time){
    // std::vector<swarm_ranging_ping_t> rx_pings;
    // environment.uwb_channel.receive_all_srp(&rx_pings);
    _srp_buffer_mutex.lock();
    for (uint16_t iPing=0; iPing<_srp_buffer_not_in_range.size(); iPing++){
        ekf_input_t new_input = {.id = (uint16_t) _srp_buffer_not_in_range[iPing].header_s.sourceAddress,
                            .vx = _srp_buffer_not_in_range[iPing].sender.velocities[0]/1000.0f,
                            .vy = _srp_buffer_not_in_range[iPing].sender.velocities[1]/1000.0f,
                            .timestamp = time};
        inputs.push_back(new_input);
    
        for (uint8_t iAgent=0; iAgent<_srp_buffer_not_in_range[iPing].n_agents; iAgent++){
            if (_srp_buffer_not_in_range[iPing].agents[iAgent].last_range != 0){
                ekf_range_measurement_t new_range = {.id_A = (uint16_t) _srp_buffer_not_in_range[iPing].header_s.sourceAddress,
                                                .id_B = _srp_buffer_not_in_range[iPing].agents[iAgent].id,
                                                .range = _srp_buffer_not_in_range[iPing].agents[iAgent].last_range/1000.0f,
                                                .timestamp = time};
                ranges.push_back(new_range);
            }    
        }
        
    }
    _srp_buffer_not_in_range.clear();
    _srp_buffer_mutex.unlock();
    // finally, add data from in-range pings
    get_new_ranging(ranges, inputs);
}


void SwarmRanging::process_ranging_ping(uwb_time_t &rx_time, const swarm_ranging_ping_t &msg, float rssi, std::vector<ekf_input_t> &inputs, const float time)
{
    uint16_t src_id = (uint16_t) msg.header_s.sourceAddress;
    uint16_t storage_idx = get_storage_index(src_id);

    _agents[storage_idx].rssi = rssi;
    _agents[storage_idx].t_last_seen = time;
    int n_tx_q = _agents[storage_idx].number_in_tx_queue;
    if (_agents[storage_idx].number_in_tx_queue < 0){
        // currently no answer scheduled, add to tx queue
        _agents[storage_idx].number_in_tx_queue = _agents_in_tx_queue;
        _agents_in_tx_queue++;
    }

    // Read sender block (ekf inputs)
    ekf_input_t new_input = {.id = (uint16_t) msg.header_s.sourceAddress,
                            .vx = msg.sender.velocities[0]/1000.0f,
                            .vy = msg.sender.velocities[1]/1000.0f,
                            //.rssi = rssi,
                            .timestamp = time};
    inputs.push_back(new_input);

    // Check if we need to know the tx timestamp of the last message
    uint8_t last_seq = msg.sender.seq-1;
    uwb_time_t last_tx = {.full = 0};
    memcpy(&last_tx, &msg.sender.last_tx, 5);
    process_remote_tx_time(_agents[storage_idx], last_tx, last_seq);

    // Process agent blocks (range measurements)
    for (uint8_t i_agent=0; i_agent<msg.n_agents; i_agent++){
        if(msg.agents[i_agent].id == _self_id){
            // this block is for me (timestamps)
            if(_agents[storage_idx].rx_time[RESPONSE].full == 0){
                // First message from this agent > RESPONSE type
                clear_timestamps(_agents[storage_idx]);
                add_timestamps_response(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
            } else if(_agents[storage_idx].rx_time[REPORT].full != 0){
                // Ranging Table is full already (2nd REPORT)
                shift_timestamps(_agents[storage_idx]);
                add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);                
            } else {
                // REPORT type
                add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
            }
        } else {
            // this block is for someone else
            // (but might contain a secondary range measurement)
            if (msg.agents[i_agent].last_range != 0){
                ekf_range_measurement_t new_range = {.id_A = (uint16_t) msg.header_s.sourceAddress,
                                                   .id_B = msg.agents[i_agent].id,
                                                   .range = msg.agents[i_agent].last_range/1000.0f,
                                                   .timestamp = time};
                _secondary_range_buffer.push_back(new_range);
            }
        }
    }

    if (_agents[storage_idx].rx_time[RESPONSE].full == 0){
        // There seem to be no valid timestamps for this agent yet
        // keep sequence number and rx timestamp to start ranging
        _agents[storage_idx].seq[RESPONSE] = msg.sender.seq;
        memcpy(&_agents[storage_idx].rx_time[RESPONSE].raw, &rx_time.raw, 5);
    }
}


void SwarmRanging::calculate_direct_ranges(const float time)
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
        } else if ((_agents[i_agent].rx_time[REPORT].full - _agents[i_agent].tx_time[POLL].full) > (1000 * RANGING_MAX_PERIOD)){
            // ranging took too long, only shift timestamps and wait for next message
            shift_timestamps(_agents[i_agent]);
        }else {
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
            _agents[i_agent].t_last_range = time;

            ekf_range_measurement_t meas = {.id_A = _self_id,
                                        .id_B = _agents[i_agent].id,
                                        .range = range,
                                        .timestamp = time};
            _direct_range_buffer.push_back(meas);
            shift_timestamps(_agents[i_agent]);
        }
    }    
}

void SwarmRanging::enqueue_srp(const swarm_ranging_ping_t* srp, const float rssi, bool in_range){
    if((uint16_t)srp->header_s.sourceAddress == _self_id){
        // failsafe to make sure we don't process our own pings
        return;
    }
    _srp_buffer_mutex.lock();
    if(in_range){
        _srp_buffer.push_back(*srp);
        _rssi_buffer.push_back(rssi);
    } else {
        _srp_buffer_not_in_range.push_back(*srp);
    }
    _srp_buffer_mutex.unlock();
}

void SwarmRanging::rel_loc_animation(const uint16_t ID){
    draw d;
    uint16_t idA, idB;
    float dx_g, dy_g, dx_l, dy_l;
    float dxA, dyA, dxB, dyB;
    float own_yaw = agents[ID]->state[STATE_YAW];

    // draw pings
    bool has_ping, has_range;
    for (uint16_t i_agent=0; i_agent<_agents.size(); i_agent++){
        has_ping = (_agents[i_agent].t_last_seen>simtime_seconds-ANIMATION_TIMEOUT);
        has_range = (_agents[i_agent].t_last_range>simtime_seconds-ANIMATION_TIMEOUT);

        if(has_ping){// && !(has_range)){
            idB = _agents[i_agent].id;
            dx_g = agents[idB]->state[STATE_X] - agents[ID]->state[STATE_X];
            dy_g = agents[idB]->state[STATE_Y] - agents[ID]->state[STATE_Y];
            rotate_g2l_xy(dx_g, dy_g, own_yaw, dx_l, dy_l);
            // d.segment(0.0f, 0.0f,dx_l, dy_l, orange);
            d.segment(0.0f, 0.0f,dx_g, dy_g, orange);
        }
    }

    // draw ranging
    for (int i_meas=_animation_buffer.size()-1; i_meas>=0; i_meas--){
        if (_animation_buffer[i_meas].timestamp>simtime_seconds-ANIMATION_TIMEOUT){
            idA = _animation_buffer[i_meas].id_A;
            idB = _animation_buffer[i_meas].id_B;

            dxA = agents[idA]->state[STATE_X] - agents[ID]->state[STATE_X];
            dyA = agents[idA]->state[STATE_Y] - agents[ID]->state[STATE_Y];

            dxB = agents[idB]->state[STATE_X] - agents[ID]->state[STATE_X];
            dyB = agents[idB]->state[STATE_Y] - agents[ID]->state[STATE_Y];
            
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

void RangingEvaluator::calculate_load(const float time)
{
    if (_last_load_calculation < time - RANGING_EVALUATION_DT){
        float dt = (time - _last_load_calculation);
        _time_elapsed += dt;

        _current_load = (float)(1e-9 * _airtime_accum_ns) / dt;
        _avg_load = (float)(1e-9 * _total_airtime_ns) / _time_elapsed;

        _airtime_accum_ns = 0;
        _last_load_calculation = time;
    }
}