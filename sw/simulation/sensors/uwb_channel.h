#ifndef UWB_CHANNEL_H
#define UWB_CHANNEL_H

#include <stdint.h>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "randomgenerator.h"
#include "settings.h"

#define UWB_MSG_TYPE_PING 0
#define UWB_MSG_TYPE_DIST 1

#define RANGING_STATUS_LOST 0
#define RANGING_STATUS_PASSIVE 1
#define RANGING_STATUS_ACTIVE 2

#define STATUS_TIMEOUT 20

struct ranging_message {
    uint8_t type;
    uint16_t id_source;
    uint16_t id_dest;
    float distanceMeasurement;  // -1 for N/A
    float source_rhox;
    float source_rhoy;
    float source_dpsi;
};

extern std::shared_mutex uwb_mutex;

class UltraWidebandChannel
{
    random_generator rg;
    float uwb_range;
private:
    std::vector<uint8_t> agent_joined;
    std::vector<ranging_message> messages;
    std::vector<std::vector<uint8_t>> ranging_status;
    std::vector<std::vector<uint8_t>> ticks_since_status;
    std::vector<std::vector<float>> distance_matrix; // matrix with ranging measurements
    void update_ranging_status(const uint16_t ID_A, const uint16_t ID_B, uint8_t status);
public:
    UltraWidebandChannel(float comm_range = COMMUNICATION_RANGE);

    ~UltraWidebandChannel() {};
    
    /**
     * Join the channel to receive messages that originate in range
     * of the joining agent
     */
    void join(const uint16_t ID);

    /**
     * Send a ping message to agents in range, including body velocity and yaw rate
     */
    void send_ping(const uint16_t sourceID, float rhoX, float rhoY, float dPsi);

    /**
     * Initiate ranging process to receive a distance measurement
     */
    void range_to(const uint16_t sourceID, const uint16_t destID);
    
    /**
     * Receive a vector with all messages that originated in communication range of
     * the receiver
     * returns true if at least one message was received.
     */
    bool receive(const uint16_t receiverID, std::vector<ranging_message> *uwb, std::vector<float> *rssi);
    
    /**
     * Receive a vector with all messages that were exchanged on the channel
     */
    bool receive_all(std::vector<ranging_message> *uwb);

    /**
     * Propagate the channel, creating new measurements
     */
    void channel_update();

    void animation();
};

#endif // UWB_CHANNEL_H