#include "uwb_channel.h"
#include "omniscient_observer.h"
#include "trigonometry.h"
#include "main.h"
#include "draw.h"

std::shared_mutex uwb_mutex;

UltraWidebandChannel::UltraWidebandChannel(float comm_range){
    this->uwb_range = comm_range;
}

void UltraWidebandChannel::join(uint16_t ID){
    uwb_mutex.lock();
    // increase size of vectors and matrices to accomodate new agent, in most cases, this should only run once
    while (this->agent_joined.size()<=ID){
        this->agent_joined.push_back(false);
        this->in_range.push_back(std::vector<bool>());
        this->ranging_matrix.push_back(std::vector<float>());
        for (uint16_t i=0; i<=ID; i++){
            this->in_range[i].push_back(false);
            this->ranging_matrix[i].push_back(-1);
        }
    }

    // enable the respective ID
    this->agent_joined[ID] = true;
    uwb_mutex.unlock();
}

void UltraWidebandChannel::send_ping(const uint16_t sourceID, float rhoX, float rhoY, float dPsi){
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

void UltraWidebandChannel::range_to(const uint16_t sourceID, const uint16_t destID){
    uwb_mutex.lock();
    if (this->in_range[sourceID][destID]){
        ranging_message newDist = {.type=UWB_MSG_TYPE_DIST,
                                   .id_source=sourceID,
                                   .id_dest=destID,
                                   .distanceMeasurement=this->ranging_matrix[sourceID][destID],
                                   .source_rhox=0,
                                   .source_rhoy=0,
                                   .source_dpsi=0};
        this->messages.push_back(newDist);
    }
    uwb_mutex.unlock();
}

bool UltraWidebandChannel::receive(const uint16_t receiverID, std::vector<ranging_message> *uwb){
    uint16_t srcID, destID;
    bool message_received = false;
    uwb_mutex.lock_shared();
    for (uint i=0; i<this->messages.size();i++){
        srcID = this->messages[i].id_source;
        destID = this->messages[i].id_dest;
        if (this->in_range[receiverID][srcID] || this->in_range[receiverID][destID]){
            uwb->push_back(this->messages[i]);
            message_received = true;
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

    float dist, noise;
    main_mutex.lock_shared();
    for (uint i=0; i<this->agent_joined.size(); i++){
        for (uint j=0; j<=i; j++){
            dist = sqrt(pow(agents[i]->state[STATE_X] - agents[j]->state[STATE_X], 2)
                        + pow(agents[i]->state[STATE_Y] - agents[j]->state[STATE_Y], 2));
            
            if (dist<this->uwb_range){
                this->in_range[i][j] = true;
                this->in_range[j][i] = true;
                noise = rg.gaussian_float(0, MEAS_NOISE_UWB);
                this->ranging_matrix[i][j] = dist + noise;
                this->ranging_matrix[j][i] = dist + noise;
            } else{
                this->in_range[i][j] = false;
                this->in_range[j][i] = false;
            }
        }
    }
    main_mutex.unlock_shared();
}

void UltraWidebandChannel::animation()
{
    draw d;
    color3ub green = {83, 255, 48};
    color3ub yellow = {245, 220, 34};
    color3ub orange = {255, 147, 42};
    for (uint i=0; i<this->in_range.size(); i++){
        for (uint j=0; j<i; j++){
            if (this->in_range[i][j]){
                d.segment(agents[i]->state[STATE_Y], agents[i]->state[STATE_X],
                        agents[j]->state[STATE_Y], agents[j]->state[STATE_X], orange);
            }
        }
    }
}
