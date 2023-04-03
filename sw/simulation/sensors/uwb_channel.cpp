#include "uwb_channel.h"

#include <algorithm>

#include "omniscient_observer.h"
#include "trigonometry.h"
#include "main.h"
#include "draw.h"

std::shared_mutex uwb_mutex;

UltraWidebandChannel::UltraWidebandChannel(float comm_range){
    this->uwb_range = comm_range;
}

void UltraWidebandChannel::join(const uint16_t ID){
    uwb_mutex.lock();
    // increase size of vectors and matrices to accomodate new agent, in most cases, this should only run once
    while (this->agent_joined.size()<=ID){
        // add new line to vectors/matrices
        this->agent_joined.push_back(false);
        this->ranging_status.push_back(std::vector<uint8_t>());
        this->ticks_since_status.push_back(std::vector<uint8_t>());
        this->distance_matrix.push_back(std::vector<float>());
        for (uint16_t i=0; i<ID; i++){
            this->ranging_status[ID].push_back(RANGING_STATUS_LOST);
            this->ticks_since_status[ID].push_back(0);
            this->distance_matrix[ID].push_back(-1);
        }
        // add new column for matrices
        for (uint16_t i=0; i<=ID; i++){
            this->ranging_status[i].push_back(RANGING_STATUS_LOST);
            this->ticks_since_status[i].push_back(0);
            this->distance_matrix[i].push_back(-1);
        }
    }

    // enable the respective ID
    this->agent_joined[ID] = true;
    uwb_mutex.unlock();
}

void UltraWidebandChannel::send_ping(const uint16_t sourceID, float rhoX, float rhoY, float dPsi){
    if (sourceID<this->agent_joined.size() && this->agent_joined[sourceID]){
        ranging_message newPing = {.type=UWB_MSG_TYPE_PING,
                                .id_source=sourceID,
                                .id_dest=sourceID,
                                .distanceMeasurement=-1,
                                .source_rhox=rhoX,
                                .source_rhoy=rhoY,
                                .source_dpsi=dPsi};
        uwb_mutex.lock();
        this->messages.push_back(newPing);
        uwb_mutex.unlock();
    }
}

void UltraWidebandChannel::range_to(const uint16_t sourceID, const uint16_t destID){
    if (std::max(sourceID,destID)<this->agent_joined.size() && 
            this->agent_joined[sourceID] && this->agent_joined[destID]){

        uwb_mutex.lock();
        if (this->distance_matrix[sourceID][destID] <= this->uwb_range){ 
            float dist = this->distance_matrix[sourceID][destID] + rg.gaussian_float(0,MEAS_NOISE_UWB);
            ranging_message newDist = {.type=UWB_MSG_TYPE_DIST,
                                    .id_source=sourceID,
                                    .id_dest=destID,
                                    .distanceMeasurement=dist,
                                    .source_rhox=0,
                                    .source_rhoy=0,
                                    .source_dpsi=0};
            this->messages.push_back(newDist);
            // return message from receiver
            newDist = { .id_source=destID,
                        .id_dest=sourceID};
            this->messages.push_back(newDist);
        }
        uwb_mutex.unlock();
    }
}

bool UltraWidebandChannel::receive(const uint16_t receiverID, std::vector<ranging_message> *uwb, std::vector<float> *rssi){
    bool message_received = false;
    uint16_t srcID, destID;
    uwb_mutex.lock_shared();
    for (uint i=0; i<this->messages.size();i++){
        srcID = this->messages[i].id_source;
        destID = this->messages[i].id_dest;

        if (srcID==receiverID && destID == receiverID){
            // receiver's own ping
            continue;
        }

        if (destID==receiverID){
            uwb->push_back(this->messages[i]);
            rssi->push_back(this->distance_matrix[receiverID][srcID]+rg.gaussian_float(0,MEAS_NOISE_RSSI));
            message_received = true;
            this->update_ranging_status(receiverID, srcID, RANGING_STATUS_ACTIVE);
        }
        else if (this->distance_matrix[receiverID][srcID] <= this->uwb_range){
            uwb->push_back(this->messages[i]);
            rssi->push_back(this->distance_matrix[receiverID][srcID]+rg.gaussian_float(0,MEAS_NOISE_RSSI));
            message_received = true;
            this->update_ranging_status(receiverID, srcID, RANGING_STATUS_PASSIVE);
        }
    }
    uwb_mutex.unlock_shared();
    return message_received;
}

bool UltraWidebandChannel::receive_all(std::vector<ranging_message> *uwb){
    bool message_received = false;
    uwb_mutex.lock_shared();
    for (uint i=0; i<this->messages.size();i++){
        uwb->push_back(this->messages[i]);
        message_received = true;
    }
    uwb_mutex.unlock_shared();
    return message_received;
}

void UltraWidebandChannel::channel_update(){
    uwb_mutex.lock();
    this->messages.clear();
    uwb_mutex.unlock();

    float dist;
    main_mutex.lock_shared();
    for (uint i=0; i<this->agent_joined.size(); i++){
        for (uint j=0; j<=i; j++){
            dist = sqrt(pow(agents[i]->state[STATE_X] - agents[j]->state[STATE_X], 2)
                        + pow(agents[i]->state[STATE_Y] - agents[j]->state[STATE_Y], 2));
            this->distance_matrix[i][j] = dist;
            this->distance_matrix[j][i] = dist;
            
            this->ticks_since_status[i][j] += 1;
            this->update_ranging_status(i, j, RANGING_STATUS_LOST);
        }
    }
    main_mutex.unlock_shared();
}

void UltraWidebandChannel::update_ranging_status(const uint16_t ID_A, const uint16_t ID_B, uint8_t status){
    if (this->ranging_status[ID_A][ID_B] == status){
        // keep status
        this->ticks_since_status[ID_A][ID_B] = 0;
    } 
    else if (this->ranging_status[ID_A][ID_B] < status){
        // always upgrade status
        this->ranging_status[ID_A][ID_B] = status;
        this->ticks_since_status[ID_A][ID_B] = 0;
    } 
    else if (this->ticks_since_status[ID_A][ID_B]>=STATUS_TIMEOUT){
        // downgrade after timeout
        this->ranging_status[ID_A][ID_B] = status;
        this->ticks_since_status[ID_A][ID_B] = 0;
    }

    // make sure matrices are symmetric
    this->ranging_status[ID_B][ID_A] = this->ranging_status[ID_A][ID_B];
    this->ticks_since_status[ID_B][ID_A] = this->ticks_since_status[ID_A][ID_B];
}

void UltraWidebandChannel::animation()
{
    draw d;
    color3ub green = {83, 255, 48};
    color3ub yellow = {245, 220, 34};
    color3ub orange = {255, 147, 42};
    uwb_mutex.lock_shared();
    for (uint i=0; i<this->ranging_status.size(); i++){
        for (uint j=0; j<i; j++){
            if (this->ranging_status[i][j] == RANGING_STATUS_PASSIVE){
                d.segment(agents[i]->state[STATE_Y], agents[i]->state[STATE_X],
                        agents[j]->state[STATE_Y], agents[j]->state[STATE_X], orange);
            }

            if (this->ranging_status[i][j] == RANGING_STATUS_ACTIVE){
                d.segment(agents[i]->state[STATE_Y], agents[i]->state[STATE_X],
                        agents[j]->state[STATE_Y], agents[j]->state[STATE_X], green);
            }
        }
    }
    uwb_mutex.unlock_shared();
}
