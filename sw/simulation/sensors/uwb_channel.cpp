#include "uwb_channel.h"

#include <algorithm>

#include "omniscient_observer.h"
#include "trigonometry.h"
#include "main.h"
#include "draw.h"

bool assert_rssi_valid_uwb(float rssi){
    if (rssi<0){
        return true;
    } else {
        return false;
    }
}

std::shared_mutex uwb_mutex;        // protects messages & distance matrix
std::shared_mutex uwb_status_mutex; // protects ranging_status & last_status_change

UltraWidebandChannel::UltraWidebandChannel(float comm_range){
    this->uwb_range = comm_range;
}

void UltraWidebandChannel::join(const uint16_t ID){
    uwb_mutex.lock();
    uwb_status_mutex.lock();
    // increase size of vectors and matrices to accomodate new agent, in most cases, this should only run once
    while (this->agent_joined.size()<=ID){
        // add new line to vectors/matrices
        this->agent_joined.push_back(false);
        this->delivery_queues.push_back(std::vector<swarm_ranging_ping_t>());
        this->ranging_status.push_back(std::vector<uint8_t>());
        this->last_status_change.push_back(std::vector<float>());
        this->distance_matrix.push_back(std::vector<float>());
        for (uint16_t i=0; i<ID; i++){
            this->ranging_status[ID].push_back(RANGING_STATUS_LOST);
            this->last_status_change[ID].push_back(0);
            this->distance_matrix[ID].push_back(-1);
        }
        // add new column for matrices
        for (uint16_t i=0; i<=ID; i++){
            this->ranging_status[i].push_back(RANGING_STATUS_LOST);
            this->last_status_change[i].push_back(0);
            this->distance_matrix[i].push_back(-1);
        }
    }

    // enable the respective ID
    this->agent_joined[ID] = true;

    uwb_status_mutex.unlock();
    uwb_mutex.unlock();

}


void UltraWidebandChannel::send_srp(const swarm_ranging_ping_t &srp){
    uint16_t srcID = (uint16_t) srp.header_s.sourceAddress;
    uwb_mutex.lock();
    all_pings.push_back(srp);
    for (uint i_agent=0; i_agent<delivery_queues.size(); i_agent++){
        if (agent_joined[i_agent] && i_agent != srcID &&
                distance_matrix[i_agent][srcID] <= this->uwb_range){
            delivery_queues[i_agent].push_back(srp);
        }
    }
    uwb_mutex.unlock();
}

bool UltraWidebandChannel::receive_srp(const uint16_t receiverID, std::vector<swarm_ranging_ping_t> *srp, std::vector<float> *rssi){
    bool message_received = false;
    uint16_t srcID;
    srp->clear();
    rssi->clear();

    uwb_mutex.lock();
    for (uint16_t i=0; i<delivery_queues[receiverID].size(); i++){
        srcID = delivery_queues[receiverID][i].header_s.sourceAddress;
        
        srp->push_back(delivery_queues[receiverID][i]);
        
        float rssi_value = -this->distance_matrix[receiverID][srcID]+rg.gaussian_float(0,MEAS_NOISE_RSSI);
        rssi->push_back(rssi_value);
        assert_rssi_valid_uwb(rssi_value);

        message_received = true;
        this->update_ranging_status(receiverID, srcID, RANGING_STATUS_PASSIVE);
    
    }
    delivery_queues[receiverID].clear();
    uwb_mutex.unlock();
    return message_received;
}


bool UltraWidebandChannel::receive_all_srp(std::vector<swarm_ranging_ping_t> *srp){
    bool message_received = false;
    srp->clear();
    uwb_mutex.lock_shared();
    for (uint16_t i=0; i<this->all_pings.size();i++){
        srp->push_back(this->all_pings[i]);
        message_received = true;
    }
    uwb_mutex.unlock_shared();
    return message_received;
}


void UltraWidebandChannel::channel_update(){
    uwb_mutex.lock();

    this->all_pings.clear();

    float dist;
    main_mutex.lock_shared();
    for (uint16_t i=0; i<this->agent_joined.size(); i++){
        for (uint16_t j=0; j<=i; j++){
            dist = sqrt(pow(agents[i]->state[STATE_X] - agents[j]->state[STATE_X], 2)
                        + pow(agents[i]->state[STATE_Y] - agents[j]->state[STATE_Y], 2));
            this->distance_matrix[i][j] = dist;
            this->distance_matrix[j][i] = dist;
            
            this->update_ranging_status(i, j, RANGING_STATUS_LOST);
        }
    }
    uwb_mutex.unlock();
    main_mutex.unlock_shared();
}

void UltraWidebandChannel::update_ranging_status(const uint16_t ID_A, const uint16_t ID_B, uint8_t status){
    uwb_status_mutex.lock();
    if (this->ranging_status[ID_A][ID_B] == status){
        // keep status
        this->last_status_change[ID_A][ID_B] = simtime_seconds;
    } 
    else if (this->ranging_status[ID_A][ID_B] < status){
        // always upgrade status
        this->ranging_status[ID_A][ID_B] = status;
        this->last_status_change[ID_A][ID_B] = simtime_seconds;
    } 
    else if (simtime_seconds-this->last_status_change[ID_A][ID_B]>=STATUS_TIMEOUT){
        // downgrade after timeout
        this->ranging_status[ID_A][ID_B] = status;
        this->last_status_change[ID_A][ID_B] = simtime_seconds;
    }

    // make sure matrices are symmetric
    this->ranging_status[ID_B][ID_A] = this->ranging_status[ID_A][ID_B];
    this->last_status_change[ID_B][ID_A] = this->last_status_change[ID_A][ID_B];
    uwb_status_mutex.unlock();
}

void UltraWidebandChannel::animation()
{
    draw d;
    color3ub green = {83, 255, 48};
    // color3ub yellow = {245, 220, 34};
    color3ub orange = {255, 147, 42};
    // color3ub red = {255, 0, 0};
    uwb_status_mutex.lock_shared();
    for (uint i=0; i<this->ranging_status.size(); i++){
        for (uint j=0; j<i; j++){
            if (this->ranging_status[i][j] == RANGING_STATUS_PASSIVE){
                d.segment(agents[i]->state[STATE_Y], agents[i]->state[STATE_X],
                        agents[j]->state[STATE_Y], agents[j]->state[STATE_X], orange);
            }
        }
    }

    // Draw active ranging on top of passive ranging
    for (uint i=0; i<this->ranging_status.size(); i++){
        for (uint j=0; j<i; j++){
            if (this->ranging_status[i][j] == RANGING_STATUS_ACTIVE){
                d.segment(agents[i]->state[STATE_Y], agents[i]->state[STATE_X],
                        agents[j]->state[STATE_Y], agents[j]->state[STATE_X], green);
            }
        }
    }
    uwb_status_mutex.unlock_shared();
}
