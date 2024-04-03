#ifndef UWB_CHANNEL_H
#define UWB_CHANNEL_H

#include <stdint.h>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "randomgenerator.h"
#include "settings.h"
#include "swarm_ranging.h"

#include "terminalinfo.h"


#define UWB_MSG_TYPE_PING 0
#define UWB_MSG_TYPE_DIST 1

#define RANGING_STATUS_LOST 0
#define RANGING_STATUS_PASSIVE 1
#define RANGING_STATUS_ACTIVE 2

#define STATUS_TIMEOUT 0.5f

#define UWB_PHY_HEADER_TIME_ns 141192
#define UWB_DATARATE_bps 6800000 

struct ranging_message {
    uint8_t type;
    uint16_t id_source;
    uint16_t id_dest;
    float distanceMeasurement;  // -1 for N/A
    float source_rhox;
    float source_rhoy;
    float source_dpsi;
    float timestamp;            // timestamp(seconds)
};

extern std::shared_mutex uwb_mutex;

class UltraWidebandChannel
{
    random_generator rg;
    float uwb_range;
    terminalinfo console_print;
private:
    std::vector<uint8_t> agent_joined;
    std::vector<std::vector<uint8_t>> ranging_status;
    std::vector<std::vector<float>> last_status_change;
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
     * Send a swarm ranging ping
     */
    void send_srp(const swarm_ranging_ping_t &srp);
    
    /**
     * Propagate the channel, creating new measurements
     */
    void channel_update();

    /**
     * Animate uwb links
     */
    void animation();
};

#endif // UWB_CHANNEL_H