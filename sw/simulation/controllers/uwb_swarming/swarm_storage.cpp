#include <stdint.h>
#include <vector>
#include <cstdlib>
#include <algorithm>

#include <string.h>

#include "main.h"

#include "swarm_storage.h"



SwarmStorage::SwarmStorage(){
    // for (uint8_t i=0; i<3; i++){
    //     _tx_history[i].full = 0;
    //     _seq_history[i] = 0;    
    // }
}

void SwarmStorage::init(const uint16_t ID){
    _self_id = ID;
}

bool SwarmStorage::is_agent_known(const uint16_t agent_id){
    bool agent_known = false;
    for (int idx=0; idx<agents_in_range(); idx++){
        if ( _agents[idx].id == agent_id){
            agent_known = true;
            break;
        }
    }
    return agent_known;
}

uint16_t SwarmStorage::get_storage_index(uint16_t agent_id){
    bool agent_known = false;
    uint16_t storage_idx;
    for (int idx=0; idx<agents_in_range(); idx++){
        if ( _agents[idx].id == agent_id){
            storage_idx = idx;
            agent_known = true;
            break;
        }
    }

    if (!agent_known){
        agent_info_t new_agent = {.id = agent_id};
        _agents.push_back(new_agent);
        storage_idx = _agents.size()-1;
    }
    return storage_idx;
}

void SwarmStorage::update_storage_state(const uint16_t agent_id, const agent_state_t &new_state){
    uint16_t storage_idx = get_storage_index(agent_id);

    _agents[storage_idx].state.rhoX = new_state.rhoX;
    _agents[storage_idx].state.rhoY = new_state.rhoY;
    _agents[storage_idx].state.dPsi = new_state.dPsi;

    _agents[storage_idx].state.relX = new_state.relX;
    _agents[storage_idx].state.relY = new_state.relY;
    _agents[storage_idx].state.relPsi = new_state.relPsi;
}

void SwarmStorage::update_storage_periodic(float current_time_seconds){
    // // check for timeouts
    // for (int idx=_agents.size()-1; idx>=0; idx--){
    //     if (current_time_seconds-_agents[idx].last_seen > LINK_TIMEOUT){
    //         _agents.erase(_agents.begin()+idx);
    //     }
    // }
}

// void SwarmStorage::order_by_rssi(){
//     /**
//      *  order by rssi (insertion sort since we assume the array is already
//      *  close to ordered)
//      */
//     for (int i_tbs=1; i_tbs<agents_in_range(); i_tbs++){
//         int i_now = i_tbs;
//         while (i_now>0 && (_agents[i_now].rssi>_agents[i_now-1].rssi)){
//             std::swap(_agents[i_now], _agents[i_now-1]);
//             i_now--;
//         }
//     }
// }

uint16_t SwarmStorage::agents_in_range(){
    return _agents.size();
}

uint16_t SwarmStorage::get_closest_agents(const uint16_t N_agents, std::vector<uint16_t> &agent_ids){
    uint16_t N_return = std::min((uint16_t) _agents.size(), N_agents);

    agent_ids.clear();
    for (int idx=0; idx<N_return; idx++){
        agent_ids.push_back(_agents[idx].id);
    }

    return N_return;
}

uint16_t SwarmStorage::get_random_agents(const uint16_t N_agents, std::vector<uint16_t> &agent_ids){
    uint16_t N_return = std::min((uint16_t) _agents.size(), N_agents);

    uint16_t N_max = agents_in_range();
    uint16_t new_id;
    
    agent_ids.clear();
    while (agent_ids.size()<N_return){
        new_id = _agents[rand() % N_max].id;
        bool id_not_yet_picked = true;
        for (uint16_t i_vec = 0; i_vec<agent_ids.size(); i_vec++){
            if (agent_ids[i_vec]==new_id){
                id_not_yet_picked = false;
                break;
            }
        }
        if (id_not_yet_picked){
            agent_ids.push_back(new_id);
        }
    }
    return N_return;
}

uint16_t SwarmStorage::get_all_agents(std::vector<uint16_t> &agent_ids){
    uint16_t N_return = agents_in_range();

    agent_ids.clear();
    for (uint16_t idx=0; idx<N_return; idx++){
        agent_ids.push_back(_agents[idx].id);
    }
    return N_return;
}


// uint16_t SwarmStorage::get_all_rssi(std::vector<uint16_t> &agent_ids, std::vector<float> &rssi){
//     uint16_t N_return = agents_in_range();

//     agent_ids.clear();
//     rssi.clear();

//     for (uint16_t idx=0; idx<N_return; idx++){
//         agent_ids.push_back(_agents[idx].id);
//         rssi.push_back(_agents[idx].rssi);
//     }
//     return N_return;
// }




// void SwarmStorage::process_ranging_ping(const float current_time_seconds, const uwb_time_t rx_time, const swarm_ranging_ping_t msg, const float rssi){
//     uint16_t src_id = (uint16_t) msg.header_s.sourceAddress;
//     uint16_t storage_idx = get_storage_index(src_id);

//     _agents[storage_idx].rssi = rssi;
//     _agents[storage_idx].last_seen = current_time_seconds;

//     // update velocities
//     _agents[storage_idx].state.rhoX = msg.sender.velocities[0]/1000.0f;
//     _agents[storage_idx].state.rhoY = msg.sender.velocities[1]/1000.0f;
//     // _agents[storage_idx].state.rhoZ = msg.sender.velocities[2]/1000.0f;
//     _agents[storage_idx].state.dPsi = msg.sender.yawrate / 1000.0f;

//     bool is_not_ranging = (_agents[storage_idx].tx_time[POLL].full == 0);
//     for (uint8_t i_agent=0; i_agent<msg.n_agents; i_agent++){
//         if(msg.agents[i_agent].id == _self_id){
//             is_not_ranging = false;
//             if(_agents[storage_idx].rx_time[RESPONSE].full == 0){
//                 // First message from this agent
//                 clear_timestamps(_agents[storage_idx]);
//                 add_timestamps_response(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
//             } else if(_agents[storage_idx].rx_time[REPORT].full != 0){
//                 // Ranging Table is full already
//                 shift_timestamps(_agents[storage_idx]);
//                 add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);                
//             } else {
//                 add_timestamps_report(_agents[storage_idx], rx_time, msg.sender, msg.agents[i_agent]);
//             }
//         } else {
//             // enqueue secondary distance measurement
//             if (msg.agents[i_agent].last_range != 0){
//                 ekf_range_measurement_t new_range = {.id_A = (uint16_t) msg.header_s.sourceAddress,
//                                                    .id_B = msg.agents[i_agent].id,
//                                                    .range = msg.agents[i_agent].last_range/1000.0f,
//                                                    .timestamp = current_time_seconds};
//                 _indirect_ranges.push_back(new_range);
//                 _range_measurements.push_back(new_range);
//             }
//         }
//     }

//     if (is_not_ranging){
//         // keep sequence number and rx timestamp
//         _agents[storage_idx].seq[RESPONSE] = msg.sender.seq;
//         memcpy(&_agents[storage_idx].rx_time[RESPONSE].raw, &rx_time.raw, 5);
//     }
// }

// void SwarmStorage::create_ranging_ping(swarm_ranging_ping_t &tx_ping, ekf_input_t &own_input){
//     memset(&tx_ping.header, 0, 21);
//     tx_ping.header_s.sourceAddress = (uint64_t) _self_id;
    
//     tx_ping.sender.seq = _seq_history[0] + 1;
//     memcpy(&tx_ping.sender.last_tx, _tx_history[0].raw, 5);
    
//     tx_ping.sender.velocities[0] = (int16_t)(own_input.rhoX * 1000);
//     tx_ping.sender.velocities[1] = (int16_t)(own_input.rhoY * 1000);
//     tx_ping.sender.yawrate = (int16_t)(own_input.dPsi * 1000);

//     order_by_rssi();
//     tx_ping.n_agents = std::min(agents_in_range(), (uint16_t) MAX_AGENTS_IN_PING);
    
//     for (uint8_t i_agent=0; i_agent<tx_ping.n_agents; i_agent++){
//         tx_ping.agents[i_agent].id = _agents[i_agent].id;
//         tx_ping.agents[i_agent].last_range = _agents[i_agent].range * 1000; // in mm
//         _agents[i_agent].range = 0; // Only send range once when it is fresh

//         if (_agents[i_agent].rx_time[REPORT].full != 0){
//             memcpy(&tx_ping.agents[i_agent].last_rx, _agents[i_agent].rx_time[REPORT].raw, 5);
//             tx_ping.agents[i_agent].last_seq = _agents[i_agent].seq[REPORT];
//         } else if (_agents[i_agent].rx_time[RESPONSE].full != 0){
//             memcpy(&tx_ping.agents[i_agent].last_rx, _agents[i_agent].rx_time[RESPONSE].raw, 5);
//             tx_ping.agents[i_agent].last_seq = _agents[i_agent].seq[RESPONSE];
//         } else {
//             memset(&tx_ping.agents[i_agent].last_rx, 0, 5);
//             tx_ping.agents[i_agent].last_seq = 0;
//         }
//     }

//     // progress the tx history (on the drone, this should happen in the tx callback)
//     for (uint8_t i=TX_HISTORY_LEN-1; i>0; i--){
//         _tx_history[i] = _tx_history[i-1];
//         _seq_history[i] = _seq_history[i-1];
//     }
//     _seq_history[0] = tx_ping.sender.seq;
//     _tx_history[0].full = 1; // actual tx timestamp
// }

// void SwarmStorage::calculate_range_measurements(float current_time_seconds, std::vector<ekf_range_measurement_t> *measurements){
//     for (int i_agent=0; i_agent<_agents.size(); i_agent++){
//         if(_agents[i_agent].tx_time[POLL].full == 0 ||
//              _agents[i_agent].rx_time[POLL].full == 0 ||
//              _agents[i_agent].tx_time[RESPONSE].full == 0 ||
//              _agents[i_agent].rx_time[RESPONSE].full == 0 ||
//              _agents[i_agent].tx_time[FINAL].full == 0 ||
//              _agents[i_agent].rx_time[FINAL].full == 0 ||
//              _agents[i_agent].rx_time[REPORT].full == 0){
//             continue;
//         } else {
//             /*
//             // Calculate tof & distance
//             float roundA = _agents[i_agent].rx_time[RESPONSE] - _agents[i_agent].tx_time[POLL];
//             float replyA = _agents[i_agent].tx_time[FINAL] - _agents[i_agent].rx_time[RESPONSE];

//             float roundB = _agents[i_agent].rx_time[FINAL] - _agents[i_agent].tx_time[RESPONSE];
//             float replyB = _agents[i_agent].tx_time[RESPONSE] - _agents[i_agent].rx_time[POLL];

//             float tof = (roundA*roundB - replyA*replyB)/(roundA + replyA + roundB + replyB);
            
//             _agents[i_agent].range = tof * SPEED_OF_LIGHT;
//             */
//             uint8_t id = _agents[i_agent].id;
//             main_mutex.lock_shared();
//             _agents[i_agent].range = sqrt(pow(agents[_self_id]->state[STATE_X] - agents[id]->state[STATE_X], 2)
//                                         + pow(agents[_self_id]->state[STATE_Y] - agents[id]->state[STATE_Y], 2));
//             main_mutex.unlock_shared();

//             _agents[i_agent].range += rg.gaussian_float(0,MEAS_NOISE_UWB);
//             _agents[i_agent].last_range = current_time_seconds;
//             ekf_range_measurement_t meas = {.id_A = _self_id,
//                                         .id_B = _agents[i_agent].id,
//                                         .range = _agents[i_agent].range,
//                                         .timestamp = current_time_seconds};
//             measurements->push_back(meas);
//             shift_timestamps(_agents[i_agent]);
//         }
//     }    
//     for (int i=0; i<_range_measurements.size(); i++){
//         // add indirect measurements
//         measurements->push_back(_range_measurements[i]);
//     }
// }